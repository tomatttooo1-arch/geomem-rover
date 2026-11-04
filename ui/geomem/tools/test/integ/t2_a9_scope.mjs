// A9 作用域（真桥）：换 runtime_boot_id → 详情框关、提示「记忆已重置」；旧 scope 的 select 被拒，当前 scope 被转发。
import { startFixture, startBridge, waitLog, waitHttp, stopAll, tokenFile, RawClient, ctl, launch, snap, wait, waitPill, hooks, toastsSince, toastMark, readJson, verdict, out as mk, APP, PY313 } from './tb.mjs';
const [dir] = process.argv.slice(2); mk(dir);
const tok = tokenFile(dir);
const fx = startFixture(PY313, tok, dir); await waitLog(fx, /fixture up/);
startBridge(PY313, tok, dir, 'a9'); await waitHttp(`${APP}/`);
const c = await new RawClient().open();
const { browser, page } = await launch({ init: hooks() });
await page.goto(`${APP}/?source=live`, { waitUntil: 'networkidle' }); await waitPill(page); await wait(1500);
const S1 = c.last('snapshot').memory.scope;
await page.locator('[data-object="object-0001"]').first().click(); await wait(800);
const openBefore = (await snap(page)).detailOpen;
await page.screenshot({ path: `${dir}/a9_detail_open.png` });
const m = await toastMark(page);
const S2 = 'scope-B-integ'; await ctl(`op=scope&id=${S2}`); const t0 = Date.now(); let closedMs = null;
while (Date.now() - t0 < 3000) { if (!(await snap(page)).detailOpen) { closedMs = Date.now() - t0; break; } await wait(100); }
await wait(300); const ts = await toastsSince(page, m);
await page.screenshot({ path: `${dir}/a9_after_scope.png` });
while (c.last('snapshot').memory.scope !== S2) await wait(100);
await ctl('op=clear');
const f = await c.command('find', { text: '杯子' }); await wait(800);
const task = c.last('snapshot').task;
const old = await c.command('select', { taskId: task.id, candidateId: 'object-0002', scope: S1 }); await wait(500);
const pubOld = (await ctl('op=state')).published;
const cur = await c.command('select', { taskId: task.id, candidateId: 'object-0002', scope: S2 }); const tc = Date.now(); let tgtMs = null;
while (Date.now() - tc < 3000) { if (c.last('snapshot').task.targetId === 'object-0002') { tgtMs = Date.now() - tc + cur.ackMs; break; } await wait(50); }
const pubCur = (await ctl('op=state')).published;
await c.command('cancel', { taskId: null });
const pe = page.errors.length; await browser.close(); c.close(); await stopAll();
const row = { S1, S2, openBefore, detailClosedMs: closedMs, toasts: ts, find: f.ack.accepted, taskState: task.state, nCands: task.candidates.length,
  oldScopeSelect: old.ack.reasonCode, publishedAfterOld: pubOld, curScopeSelect: { accepted: cur.ack.accepted, result: cur.result?.status }, publishedAfterCur: pubCur, targetIdMs: tgtMs, pageerrors: pe };
console.log(JSON.stringify(row)); console.log(JSON.stringify({ bridgeCounts: readJson(`${dir}/bridge_stats_a9.json`).counts }));
const pass = openBefore && closedMs !== null && ts.some(t => /记忆已重置/.test(t)) && task.state === 'ambiguous' && old.ack.reasonCode === 'scope_changed'
  && !pubOld.some(p => p[0] === '/fpga/select_candidate') && pe === 0;
const ctlHolds = cur.ack.accepted === true && pubCur.some(p => p[0] === '/fpga/select_candidate' && p[1] === 'object-0002') && tgtMs !== null && tgtMs <= 3000;
verdict('A9', ctlHolds, pass);
