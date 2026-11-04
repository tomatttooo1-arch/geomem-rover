// A1 时钟无关（真桥）＋本台阳性对照（SIGSTOP 夹具 4 s）＋T2-link 链路断开（SIGSTOP 7 s）。预注册 STAGE1_INTEG_PREREG.md。
import { startFixture, startBridge, waitLog, waitHttp, stopAll, tokenFile, RawClient, ctl, launch, snap, wait, waitPill, hooks, readJson, verdict, out as mk, APP, PY313 } from './tb.mjs';
const [dir, durS = '20'] = process.argv.slice(2); mk(dir);
const tok = tokenFile(dir);
const fx = startFixture(PY313, tok, dir); await waitLog(fx, /fixture up/);
startBridge(PY313, tok, dir, 'a1'); await waitHttp(`${APP}/`);
const c = await new RawClient().open();
const DUR = Number(durS) * 1000; let allPass = true;
for (const off of [3600e3, -3600e3]) {
  const { browser, page } = await launch({ init: hooks({ dateOffset: off }) });
  await page.goto(`${APP}/?source=live`, { waitUntil: 'networkidle' }); await waitPill(page); await wait(1000);
  const skew = await page.evaluate(() => (Date.now() - performance.timeOrigin - performance.now()) / 1000);
  const labels = new Map(); let maxDiff = 0, n = 0, stale = 0, cmp = 0; const t0 = Date.now();
  while (Date.now() - t0 < DUR) {
    const [pill, seen, st] = await Promise.all([page.locator('#connection-pill').innerText(),
      page.evaluate(() => [...document.querySelectorAll('[data-seen-id]')].map(e => [e.dataset.seenId, e.querySelector('.seen')?.dataset.seconds])), ctl('op=state')]);
    n++; labels.set(pill, (labels.get(pill) || 0) + 1); if (/过期/.test(pill)) stale++;
    for (const [id, sec] of seen) { if (!sec) continue; const o = st.objects.find(o => o.id === id); if (!o) continue; maxDiff = Math.max(maxDiff, Math.abs(Number(sec) - o.ageS)); cmp++; }
    await wait(500);
  }
  await page.screenshot({ path: `${dir}/a1_offset_${off > 0 ? 'plus' : 'minus'}3600.png` });
  const pass = stale === 0 && [...labels.keys()].every(l => l === '实车已连接') && maxDiff <= 1 && cmp > 0 && Math.abs(Math.abs(skew) - 3600) <= 1 && page.errors.length === 0;
  allPass &&= pass;
  console.log(JSON.stringify({ offsetS: off / 1000, measuredDateSkewS: +skew.toFixed(2), samples: n, labels: Object.fromEntries(labels), staleSamples: stale, seenCompared: cmp, maxSeenDiffS: +maxDiff.toFixed(3), pass, pageerrors: page.errors.length }));
  await browser.close();
}
// 本台阳性对照：SIGSTOP 夹具 4 s
const { browser, page } = await launch({ init: hooks() });
await page.goto(`${APP}/?source=live`, { waitUntil: 'networkidle' }); await waitPill(page); await wait(1000);
process.kill(fx.pid, 'SIGSTOP'); const ts = Date.now(); let leftMs = null, leftPill = '';
while (Date.now() - ts < 4000) { const p = await page.locator('#connection-pill').innerText(); if (p !== '实车已连接' && leftMs === null) { leftMs = Date.now() - ts; leftPill = p; } await wait(100); }
await page.screenshot({ path: `${dir}/a1_control_sigstop.png` });
process.kill(fx.pid, 'SIGCONT'); let back = null; try { back = await waitPill(page, /^实车已连接$/, 6000); } catch {}
const ctlHolds = leftMs !== null && leftMs <= 3500 && /过期|断开/.test(leftPill) && !!back;
console.log(JSON.stringify({ control: 'SIGSTOP 4 s', leftAfterMs: leftMs, leftPill, backAfterMs: back?.ms ?? null, controlHolds: ctlHolds }));
verdict('A1', ctlHolds, allPass, '(v1 界面对照沿用前端棒)');
// T2-link：停前 find 正常；SIGSTOP 7 s 到 down
await wait(1500);
const f0 = await c.command('find', { text: '瓶子' }); await wait(500); await c.command('cancel', { taskId: null }); await wait(1500);
process.kill(fx.pid, 'SIGSTOP'); const t2 = Date.now(); let downMs = null, sawStale = false;
while (Date.now() - t2 < 9000) { const s = c.last('snapshot'); if (s.link.state === 'stale') sawStale = true; if (s.link.state === 'down') { downMs = Date.now() - t2; break; } await wait(100); }
const fd = await c.command('find', { text: '瓶子' }, 3000); const cd = await c.command('cancel', { taskId: null }, 9000).catch(e => ({ ack: { accepted: null, reasonCode: String(e) } }));
const pillDown = (await snap(page)).pill; await page.screenshot({ path: `${dir}/t2_link_down.png` });
process.kill(fx.pid, 'SIGCONT'); let back2 = null; try { back2 = await waitPill(page, /^实车已连接$/, 10000); } catch {}
const linkRow = { findBefore: { accepted: f0.ack.accepted, result: f0.result?.status }, sawStale, downAfterMs: downMs, findDown: fd.ack.reasonCode, cancelDown: { accepted: cd.ack.accepted, code: cd.ack.reasonCode ?? null, result: cd.result?.status ?? null }, pillDown, backAfterMs: back2?.ms ?? null };
console.log(JSON.stringify({ part: 'T2-link', ...linkRow }));
verdict('T2-link', f0.ack.accepted === true, downMs !== null && sawStale && fd.ack.reasonCode === 'link_down' && cd.ack.reasonCode !== 'link_down' && pillDown !== '实车已连接' && !!back2);
await browser.close(); c.close(); await stopAll();
console.log(JSON.stringify({ bridgeCounts: readJson(`${dir}/bridge_stats_a1.json`).counts }));
