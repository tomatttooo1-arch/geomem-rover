// 对齐-no_ack 只提示一次（真桥）：夹具占住 hub 锁 5 s，页面立即发 find → 桥 3.5 s 回 no_ack。
// 阳性对照：同流程把 /src/transport.js 换成对齐前版本（586045d），提示里不含「结果未知」。预注册 STAGE1_INTEG_PREREG.md。
import { execFileSync } from 'node:child_process';
import { startFixture, startBridge, waitLog, waitHttp, stopAll, tokenFile, ctl, launch, wait, waitPill, hooks, toastsSince, toastMark, readJson, verdict, out as mk, APP, PY313, WT } from './tb.mjs';
const [dir] = process.argv.slice(2); mk(dir);
const tok = tokenFile(dir);
const fx = startFixture(PY313, tok, dir); await waitLog(fx, /fixture up/);
startBridge(PY313, tok, dir, 'noack'); await waitHttp(`${APP}/`);
const OLD = execFileSync('git', ['-C', WT, 'show', '586045d:ui/geomem/app/src/transport.js']);
async function arm(old) {
  await ctl('op=clear'); await ctl('op=task&state=idle');
  const { browser, page } = await launch({ init: hooks() });
  if (old) await page.route('**/src/transport.js', r => r.fulfill({ status: 200, contentType: 'text/javascript', body: OLD }));
  await page.goto(`${APP}/?source=live`, { waitUntil: 'networkidle' }); await waitPill(page); await wait(1500);
  await page.fill('#search-input', '瓶子'); await page.click('#find-button'); await wait(200);
  const m = await toastMark(page);
  await ctl('op=hold&s=5'); await page.click('[data-find]');
  await wait(9000);
  const ts = await toastsSince(page, m); const st = await ctl('op=state'); const hint = await page.locator('#query-hint').innerText();
  await page.screenshot({ path: `${dir}/noack_${old ? 'old' : 'new'}.png` });
  const pe = page.errors.length; await browser.close();
  await ctl('op=task&state=cancelled&reason=user_cancelled'); await wait(500);
  return { arm: old ? '对齐前 transport.js（586045d）' : '本棒', toasts: ts, queryReachedHub: st.published.some(p => p[0] === '/fpga/query'), hintAfter: hint, pageerrors: pe };
}
const a = await arm(false), b = await arm(true);
console.log(JSON.stringify(a)); console.log(JSON.stringify(b));
await stopAll();
console.log(JSON.stringify({ bridgeCounts: readJson(`${dir}/bridge_stats_noack.json`).counts }));
const pass = a.toasts.length === 1 && /结果未知/.test(a.toasts[0]) && !/被拒/.test(a.toasts[0]) && a.queryReachedHub && a.pageerrors === 0;
verdict('no_ack', b.toasts.length >= 1 && !b.toasts.some(t => /结果未知/.test(t)), pass);
