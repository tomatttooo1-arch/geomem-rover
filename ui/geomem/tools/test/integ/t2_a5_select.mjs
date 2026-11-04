// A5-合成：ambiguous → select → cancel 的界面流程（真桥＋夹具替身 runner），1920×1080 与 390×844 各 3 次。
// 只证明桥＋前端管道；任务事件由夹具按脚本给，不证明 runner 行为（真 runner 版是 T3）。预注册 STAGE1_INTEG_PREREG.md。
import { startFixture, startBridge, waitLog, waitHttp, stopAll, tokenFile, RawClient, ctl, launch, wait, waitPill, hooks, readJson, verdict, out as mk, APP, PY313 } from './tb.mjs';
const [dir] = process.argv.slice(2); mk(dir);
const tok = tokenFile(dir);
const fx = startFixture(PY313, tok, dir); await waitLog(fx, /fixture up/);
startBridge(PY313, tok, dir, 'a5'); await waitHttp(`${APP}/`);
const c = await new RawClient().open();
const untilSnap = async (pred, ms) => { const t0 = Date.now(); while (Date.now() - t0 < ms) { const s = c.last('snapshot'); if (s && pred(s)) return Date.now() - t0; await wait(30); } return null; };
const visible = page => page.evaluate(() => { const vis = el => { const r = el.getBoundingClientRect(); let ok = r.top >= 0 && r.bottom <= innerHeight && r.left >= 0 && r.right <= innerWidth && r.width > 0; for (let p = el.parentElement; p; p = p.parentElement) { const cs = getComputedStyle(p); if (/(auto|scroll|hidden)/.test(cs.overflowY + cs.overflowX)) { const q = p.getBoundingClientRect(); if (r.top < q.top - 1 || r.bottom > q.bottom + 1 || r.left < q.left - 1 || r.right > q.right + 1) ok = false; } } return { ok, top: Math.round(r.top), bottom: Math.round(r.bottom) }; };
  return [...document.querySelectorAll('[data-candidate]')].map(b => ({ id: b.dataset.candidate, ...vis(b) })); });
async function findCup(page) { await page.fill('#search-input', '杯子'); await page.click('#find-button'); await wait(200); await page.click('[data-find]'); }
const rows = []; let all = true;
for (const vp of [{ width: 1920, height: 1080 }, { width: 390, height: 844 }]) {
  const { browser, page } = await launch({ viewport: vp, init: hooks() });
  await page.goto(`${APP}/?source=live`, { waitUntil: 'networkidle' }); await waitPill(page); await wait(1500);
  for (let k = 1; k <= 3; k++) {
    await findCup(page);
    const ambMs = await untilSnap(s => s.task.state === 'ambiguous' && s.task.candidates.length === 2, 4000);
    await page.waitForSelector('[data-candidate]', { timeout: 5000 }).catch(() => {});
    await page.waitForFunction(() => [...document.querySelectorAll('[data-candidate]')].length === 2 && [...document.querySelectorAll('[data-candidate]')].every(b => !b.disabled), null, { timeout: 5000 }).catch(() => {});
    await page.locator('#task-candidates').scrollIntoViewIfNeeded(); await wait(400);
    const cands = await visible(page);
    if (k === 1) await page.screenshot({ path: `${dir}/a5_candidates_${vp.width}.png` });
    const second = cands[1]?.id;
    const t1 = Date.now(); await page.locator(`[data-candidate="${second}"]`).click();
    const tgtMs = await untilSnap(s => s.task.targetId === second, 3000); const tgtTotal = tgtMs === null ? null : Date.now() - t1;
    await page.waitForFunction(() => !document.getElementById('stop-task').disabled && !document.querySelector('.task-controls').hidden, null, { timeout: 4000 }).catch(() => {});
    const t2 = Date.now(); await page.click('#stop-task');
    const canMs = await untilSnap(s => s.task.state === 'cancelled', 2000); const canTotal = canMs === null ? null : Date.now() - t2;
    const permit = (await ctl('op=state')).permit, navPermit = null;
    const ok = ambMs !== null && cands.length === 2 && cands.every(x => x.ok) && tgtTotal !== null && tgtTotal <= 3000 && canTotal !== null && canTotal <= 2000 && permit === false;
    all &&= ok; rows.push({ viewport: `${vp.width}x${vp.height}`, rep: k, ambMs, cands, selected: second, targetMs: tgtTotal, cancelMs: canTotal, fixturePermit: permit, ok });
    if (k === 3) await page.screenshot({ path: `${dir}/a5_cancelled_${vp.width}.png` });
    await wait(1200);
  }
  all &&= page.errors.length === 0; await browser.close();
}
// 阳性对照：只放一个杯子
await ctl('op=objects&set=single'); await wait(1500);
const { browser, page } = await launch({ init: hooks() });
await page.goto(`${APP}/?source=live`, { waitUntil: 'networkidle' }); await waitPill(page); await wait(1500);
await findCup(page); const navMs = await untilSnap(s => s.task.state === 'navigating', 3000); await wait(800);
const sn = c.last('snapshot'); const candDom = await page.evaluate(() => document.querySelectorAll('[data-candidate]').length);
await page.screenshot({ path: `${dir}/a5_control_single.png` });
const ctlRow = { navMs, state: sn.task.state, targetId: sn.task.targetId, candidates: sn.task.candidates.length, candDom };
await page.click('#stop-task').catch(() => {}); await wait(800); await browser.close(); c.close(); await stopAll();
for (const r of rows) console.log(JSON.stringify(r));
console.log(JSON.stringify({ control: ctlRow, bridgeCounts: readJson(`${dir}/bridge_stats_a5.json`).counts }));
verdict('A5-合成', navMs !== null && sn.task.targetId === 'object-0001' && sn.task.candidates.length === 0 && candDom === 0, all);
