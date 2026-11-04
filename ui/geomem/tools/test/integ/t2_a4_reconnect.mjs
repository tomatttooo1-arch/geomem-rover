// A4 重连不重发（真桥）：页面发 find 后按 pid 停桥、起新桥，连做 3 次。预注册 STAGE1_INTEG_PREREG.md。
import { startFixture, startBridge, waitLog, waitHttp, stopPid, stopAll, tokenFile, ctl, launch, wait, waitPill, hooks, toastsSince, toastMark, readJson, verdict, out as mk, APP, PY313 } from './tb.mjs';
const [dir] = process.argv.slice(2); mk(dir);
const tok = tokenFile(dir);
const fx = startFixture(PY313, tok, dir); await waitLog(fx, /fixture up/);
let br = startBridge(PY313, tok, dir, 'a4_0'); await waitHttp(`${APP}/`);
await ctl('op=runner&on=0');
const { browser, page } = await launch({ init: hooks() });
await page.goto(`${APP}/?source=live`, { waitUntil: 'networkidle' }); await waitPill(page); await wait(1500);
const rows = [];
for (let k = 1; k <= 3; k++) {
  await page.fill('#search-input', '瓶子'); await page.click('#find-button'); await wait(200);
  const m = await toastMark(page);
  await page.click('[data-find]'); await wait(300);
  await stopPid(br); const tKill = Date.now();
  await wait(500); br = startBridge(PY313, tok, dir, `a4_${k}`);
  const back = await waitPill(page, /^实车已连接$/, 15000).catch(() => null);
  const reconnectMs = back ? Date.now() - tKill : null;
  await wait(800);
  const ts = await toastsSince(page, m);
  rows.push({ cycle: k, reconnectMs, unknownToast: ts.some(t => /结果未知/.test(t)), toasts: ts });
  if (k === 2) await page.screenshot({ path: `${dir}/a4_cycle2.png` });
}
await wait(1500);
// 阳性对照：夹具直接喂进行中任务，手点停止
await ctl('op=runner&on=1'); await ctl('op=task&state=navigating&reason=reobserve&target=object-0003');
await wait(1500); await page.click('#stop-task'); await wait(2500);
const st = await ctl('op=state');
await page.screenshot({ path: `${dir}/a4_control_stop.png` });
const pe = page.errors.length; await browser.close(); await stopAll();
const stats = [0, 1, 2, 3].map(k => readJson(`${dir}/bridge_stats_a4_${k}.json`).counts);
const queries = st.published.filter(p => p[0] === '/fpga/query').length, cancels = st.published.filter(p => p[0] === '/fpga/cancel').length;
for (const r of rows) console.log(JSON.stringify(r));
console.log(JSON.stringify({ perBridgeBrowserCmd: stats.map(c => c.browser_cmd || 0), perBridgeCmdKinds: stats.map(c => Object.fromEntries(Object.entries(c).filter(([k]) => k.startsWith('browser_cmd:')))), hubQueries: queries, hubCancels: cancels, pageerrors: pe }));
// 每个桥：0..2 各收到 1 条故意的 find；第 3 个（最后）只收到对照的 1 条 cancel
const noResend = stats.slice(0, 3).every(c => (c.browser_cmd || 0) === 1 && (c['browser_cmd:find'] || 0) === 1) && queries === 3;
const ctlHolds = (stats[3].browser_cmd || 0) === 1 && (stats[3]['browser_cmd:cancel'] || 0) === 1 && cancels === 1;
verdict('A4', ctlHolds, noResend && rows.every(r => r.reconnectMs !== null && r.reconnectMs <= 6000 && r.unknownToast) && pe === 0);
