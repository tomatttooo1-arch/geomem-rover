// T3 A5 补测「停止」（第 3 棒）：car_up sim 真车栈＋真桥＋真前端。预注册 STAGE1_INTEG_PREREG.md 第 3 棒 §4.1/§4.2。
// 在车的网络命名空间里的试验台容器中跑（bridge/testbed/t3_tb.sh run）。GEOMEM_HUB=本趟 hub 地址，参数：<输出目录> <口令文件>。
// 第二只瓶由宿主侧放（§4.4：P1/P2 = i 1/2）：本脚本写 <dir>/need_bottle_<i>，宿主放好后写 <dir>/bottle_<i>.done。
// runner 首选「第一次看见时的位姿」作观察位（都是起点），所以先用搜「书包」让车离开起点再选（§4.4）。发起一律用页面按钮。
import fs from 'node:fs';
import { startBridge, startPy, waitLog, waitHttp, stopAll, stopPid, RawClient, launch, wait, waitPill, hooks, toastsSince, toastMark, readJson, verdict, out as mk, APP, PY313, WT, HUB_ADDR } from './tb.mjs';
const [dir, tok] = process.argv.slice(2); mk(dir);
const log = (...a) => console.log(new Date().toISOString(), ...a);
const PILL = /仿真数据/;

startBridge(PY313, tok, dir, 'stop'); await waitHttp(`${APP}/`, 30000);
const c = await new RawClient().open();
await c.next(m => m.type === 'hello' && m.bridge.link === 'up', 20000).catch(() => null);
const ctlHello = c.last('hello');
log('hello', JSON.stringify(ctlHello));
if (!ctlHello || ctlHello.capabilities.control !== 'controller') { verdict('A5-STOP', false, false, '桥不是控制者'); await stopAll(); process.exit(0); }
const tapFile = `${dir}/tap.jsonl`;
const tap = startPy(PY313, [`${WT}/ui/geomem/bridge/testbed/t3_state_tap.py`, HUB_ADDR, tok, tapFile], `${dir}/tap.log`); await waitLog(tap, /tap up/);
const tapRows = () => fs.readFileSync(tapFile, 'utf8').trim().split('\n').filter(Boolean).map(l => JSON.parse(l)).filter(r => r.seq !== undefined);
const lastTap = () => tapRows().at(-1);
const snaps = (from = 0, to = Infinity) => c.msgs.filter(m => m.type === 'snapshot' && m._t >= from && m._t <= to);
const lastSnap = () => c.last('snapshot');
const bottles = () => (lastSnap()?.objects || []).filter(o => o.className === 'bottle');
const untilSnap = async (pred, ms, from = Date.now()) => { const end = from + ms; while (Date.now() < end + 300) { const hit = snaps(from).find(pred); if (hit) return hit._t - from; await wait(30); } const hit = snaps(from, end).find(pred); return hit ? hit._t - from : null; };
async function waitFor(fn, ms, step = 250) { const t0 = Date.now(); while (Date.now() - t0 < ms) { const v = await fn(); if (v) return v; await wait(step); } return null; }
const visible = page => page.evaluate(() => { const vis = el => { const r = el.getBoundingClientRect(); let ok = r.top >= 0 && r.bottom <= innerHeight && r.left >= 0 && r.right <= innerWidth && r.width > 0; for (let p = el.parentElement; p; p = p.parentElement) { const cs = getComputedStyle(p); if (/(auto|scroll|hidden)/.test(cs.overflowY + cs.overflowX)) { const q = p.getBoundingClientRect(); if (r.top < q.top - 1 || r.bottom > q.bottom + 1 || r.left < q.left - 1 || r.right > q.right + 1) ok = false; } } return { ok, top: Math.round(r.top), bottom: Math.round(r.bottom) }; };
  return [...document.querySelectorAll('[data-candidate]')].map(b => ({ id: b.dataset.candidate, ...vis(b) })); });
const robotOf = s => s?.robot ? { x: +s.robot.position[0].toFixed(3), y: +s.robot.position[1].toFixed(3), pose: s.robot.pose, speed: s.robot.speed } : null;

const big = await launch({ viewport: { width: 1920, height: 1080 }, init: hooks() });
const small = await launch({ viewport: { width: 390, height: 844 }, init: hooks() });
for (const x of [big, small]) { await x.page.goto(`${APP}/?source=live`, { waitUntil: 'networkidle' }); log('pill', JSON.stringify(await waitPill(x.page, PILL, 60000))); }
await wait(1500);
const bp = big.page;
const pageText = () => bp.evaluate(() => ({ badge: document.getElementById('pose-badge')?.hidden ? '' : document.getElementById('pose-badge')?.innerText || '', hint: document.getElementById('query-hint')?.innerText || '' }));

// ---- a-T3（§4.1）：页面上小车位置与找物按钮
const aRobot = await waitFor(() => robotOf(lastSnap())?.pose === 'odometry' ? robotOf(lastSnap()) : null, 60000, 500);
await bp.fill('#search-input', '瓶子'); await bp.click('#find-button'); await wait(500);
const nFind = await bp.locator('[data-find]').count();
const findEnabled = nFind ? !(await bp.locator('[data-find]').first().isDisabled()) : false;
const aTxt = await pageText();
const aT3 = { robot: aRobot, badge: aTxt.badge, hint: aTxt.hint, findButtons: nFind, findEnabled,
  holds: !!aRobot && /仅里程计定位/.test(aTxt.badge) && findEnabled && /仅里程计定位/.test(aTxt.hint) };
await bp.screenshot({ path: `${dir}/a_t3_page.png` });
log('A-T3', JSON.stringify(aT3));

// 页面发起找物；按钮不可用或 1 s 内没发出 → null（该次试验台无效，不走原始客户端）
async function find(text = '瓶子') {
  await bp.fill('#search-input', text); await bp.click('#find-button'); await wait(300);
  const n = await bp.locator('[data-find]').count(); const btn = bp.locator('[data-find]').first();
  const disabled = n ? await btn.isDisabled() : true;
  const note = (await pageText()).hint; const mark = await toastMark(bp);
  if (disabled) return { t: Date.now(), via: null, note };
  const t = Date.now(); await btn.click();
  const sent = await waitFor(async () => (await toastsSince(bp, mark)).length || snaps(t).some(s => !['idle', 'cancelled', 'succeeded', 'failed'].includes(s.task.state)), 1000, 50);
  return { t, via: sent ? 'page' : null, note, toasts: await toastsSince(bp, mark) };
}
const isActive = s => ['planning', 'navigating', 'confirming', 'searching', 'ambiguous'].includes(s.task.state);
async function stopTask(label) {
  const before = lastTap(), rBefore = robotOf(lastSnap()); const t = Date.now();
  await bp.click('#stop-task', { timeout: 3000 }).catch(e => log('stop click failed', e.message));
  await wait(2500);
  const can = snaps(t).find(s => s.task.state === 'cancelled'); const per = tapRows().find(r => r.t * 1000 >= t && r.permit === false);
  const cancelMs = can ? can._t - t : null, permitMs = per ? Math.round(per.t * 1000 - t) : null;
  const after = tapRows().filter(r => r.t * 1000 >= t && r.t * 1000 <= t + 2500);
  return { label, cancelMs, permitMs, stopMs: cancelMs === null || permitMs === null ? null : Math.max(cancelMs, permitMs), permitBefore: before?.permit, taskBefore: before?.task,
    robotAtClick: rBefore, robotAfter2s: robotOf(lastSnap()), odomVAfter: after.slice(-5).map(r => r.odom_v), t };
}
const seq = (from, to) => { const out = []; for (const s of snaps(from, to)) { const k = `${s.task.state}:${s.task.targetId ?? ''}:${s.task.candidates.length}`; if (out.at(-1)?.k !== k) out.push({ k, dt: s._t - from }); } return out; };
const tapSeq = (from, to) => { const out = []; for (const r of tapRows().filter(r => r.t * 1000 >= from && r.t * 1000 <= to)) { const k = `${r.task}:${r.target ?? ''}:${r.permit}`; if (out.at(-1)?.k !== k) out.push({ k, dt: Math.round(r.t * 1000 - from) }); } return out; };
async function settle() { await wait(5000); await waitFor(() => lastTap()?.permit === false, 30000); }
const stats = () => { try { return readJson(`${dir}/bridge_stats_stop.json`).counts; } catch { return null; } };

// ---- C0 单瓶对照（页面发起）
const c0 = await waitFor(() => bottles().length === 1 ? bottles() : null, 180000, 500);
log('C0 bottles', JSON.stringify(bottles().map(o => [o.id, o.state, o.position])));
let control = { guard: !!c0 };
if (c0) {
  const B1 = c0[0].id;
  const f = await find();
  const hitMs = f.via ? await untilSnap(s => ['planning', 'navigating', 'confirming', 'succeeded'].includes(s.task.state) && s.task.targetId === B1, 5000, f.t) : null;
  const between = snaps(f.t, f.t + (hitMs ?? 5000));
  const noAmb = between.every(s => s.task.state !== 'ambiguous' && s.task.candidates.length === 0);
  const candDom = await bp.evaluate(() => document.querySelectorAll('[data-candidate]').length);
  const st = isActive(lastSnap()) ? await stopTask('control') : null;
  control = { guard: true, B1, find: f, hitMs, noAmb, candDom, seq: seq(f.t, f.t + 8000), stop: st, holds: !!f.via && hitMs !== null && noAmb && candDom === 0 };
}
log('CONTROL', JSON.stringify(control));
if (!control.holds) { verdict('A5-STOP', false, false, '单瓶对照不成立'); await finish(); }
await settle();
const B1 = control.B1;

// ---- 第二只瓶（P1、P2，起点视场内）
const START = robotOf(lastSnap());
log('START', JSON.stringify(START));
let placed = null;
for (const i of [1, 2]) {
  fs.writeFileSync(`${dir}/need_bottle_${i}`, `${Date.now()}\n`); log('need bottle', i);
  const done = await waitFor(() => fs.existsSync(`${dir}/bottle_${i}.done`), 90000, 500);
  log('bottle placed', i, done ? fs.readFileSync(`${dir}/bottle_${i}.done`, 'utf8').trim() : 'TIMEOUT');
  if (done && await waitFor(() => bottles().length === 2, 120000, 500)) { placed = i; break; }
}
const B2 = bottles().find(o => o.id !== B1)?.id;
log('G2', placed, B2, JSON.stringify(bottles().map(o => [o.id, o.state, o.position])));
if (!placed || !B2) { verdict('A5-STOP', false, false, 'P1/P2 都没进记忆'); await finish(); }

// ---- 离开起点（§4.4，搭场景，不判）：搜「书包」→ runner 搜索巡视 → 离起点 ≥ 0.8 m 时停止
const away = () => { const r = robotOf(lastSnap()); return r && START ? Math.hypot(r.x - START.x, r.y - START.y) : null; };
const moves = [];
async function moveAway(tag) {
  const f = await find('书包');
  if (!f.via) { moves.push({ tag, find: f, ok: false }); return false; }
  const hit = await waitFor(() => (away() ?? 0) >= 0.8, 60000, 100);
  const st = isActive(lastSnap()) ? await stopTask(`away-${tag}`) : null;
  moves.push({ tag, ok: !!hit, away_m: away(), robot: robotOf(lastSnap()), seq: seq(f.t, Date.now()), stop: st });
  log('AWAY', JSON.stringify(moves.at(-1)));
  await settle();
  return !!hit;
}
if (!await moveAway('0')) { verdict('A5-STOP', false, false, '车离不开起点'); await finish(); }

// ---- 正式 3 次
const rows = []; let all = true, invalid = false;
for (let k = 1; k <= 3; k++) {
  for (let m = 0; m < 2 && (away() ?? 0) < 0.6; m++) await moveAway(`${k}.${m}`);
  if ((away() ?? 0) < 0.6) { rows.push({ rep: k, invalid: '车离起点 < 0.6 m', away_m: away() }); invalid = true; continue; }
  const g = await waitFor(() => bottles().length === 2, 60000, 500);
  if (!g) { rows.push({ rep: k, guard: false, bottles: bottles().length }); invalid = true; log('rep', k, 'guard fail'); continue; }
  let f, ambMs, tries = 0;
  do {
    tries++; f = await find();
    ambMs = f.via ? await untilSnap(s => s.task.state === 'ambiguous' && s.task.candidates.length === 2 && s.task.candidates.every(x => x.className === 'bottle'), 5000, f.t) : null;
    const tz = (f.toasts || []).join(' ') + (await toastsSince(bp, 0)).slice(-2).join(' ');
    if (ambMs === null && /还有任务或导航在进行|cancel current operation/.test(tz) && tries < 4) { log('rep', k, 'query rejected busy, retry'); await wait(3000); continue; }
    break;
  } while (true);
  if (!f.via) { rows.push({ rep: k, find: f, invalid: '页面发不出找物' }); invalid = true; log('REP', JSON.stringify(rows.at(-1))); await settle(); continue; }
  const vis = {};
  for (const [name, pg] of [['1920', bp], ['390', small.page]]) {
    await pg.waitForFunction(() => { const b = [...document.querySelectorAll('[data-candidate]')]; return b.length === 2 && b.every(x => !x.disabled); }, null, { timeout: 5000 }).catch(() => {});
    await pg.locator('#task-candidates').scrollIntoViewIfNeeded().catch(() => {}); await wait(400);
    vis[name] = await visible(pg);
    if (k === 1) await pg.screenshot({ path: `${dir}/stop_candidates_${name}.png` });
  }
  const r0 = robotOf(lastSnap());
  const t1 = Date.now(); await bp.locator(`[data-candidate="${B2}"]`).click({ timeout: 3000 }).catch(e => log('select click failed', e.message));
  const tgtMs = await untilSnap(s => s.task.targetId === B2, 3000, t1);
  // 前往阶段阳性对照：navigating ∧ targetId==B2（桥快照）∧ motion_permit==true（旁听）同时成立
  const goT = await waitFor(() => { const s = lastSnap(), r = lastTap(); return s?.task.state === 'navigating' && s.task.targetId === B2 && r?.permit === true && r.t * 1000 >= t1 ? Date.now() : null; }, 10000, 50);
  const goMs = goT ? goT - t1 : null;
  if (k === 1 && goT) await bp.screenshot({ path: `${dir}/stop_navigating_1920.png` });
  let st = null;
  if (goT) st = await stopTask(`rep${k}`);
  else { log('rep', k, 'no navigating phase'); if (isActive(lastSnap())) st = await stopTask(`rep${k}-late`); }
  const visOk = ['1920', '390'].every(n => vis[n].length === 2 && vis[n].every(x => x.ok));
  const ok = ambMs !== null && visOk && tgtMs !== null && tgtMs <= 3000 && goT !== null && st?.stopMs !== null && st?.stopMs <= 2000;
  if (!goT) invalid = true;
  all &&= ok;
  rows.push({ rep: k, tries, ambMs, candidates: snaps(f.t).find(s => s.task.state === 'ambiguous')?.task.candidates.map(x => x.id), vis, selected: B2, targetMs: tgtMs, goMs,
    moved_m: r0 && st?.robotAtClick ? +Math.hypot(st.robotAtClick.x - r0.x, st.robotAtClick.y - r0.y).toFixed(3) : null, robotAtSelect: r0, stop: st,
    seqBridge: seq(t1, (st?.t ?? Date.now()) + 2500), seqTap: tapSeq(t1, (st?.t ?? Date.now()) + 2500), statsCounts: stats(), ok });
  log('REP', JSON.stringify(rows.at(-1)));
  if (k === 3) await bp.screenshot({ path: `${dir}/stop_cancelled_1920.png` });
  await settle();
}
const perr = big.page.errors.length + small.page.errors.length;
all &&= perr === 0;
for (const r of rows) console.log(JSON.stringify(r));
console.log(JSON.stringify({ aT3, control, placed, B1, B2, START, moves, pageerrors: perr }));
verdict('A-T3', true, aT3.holds);
verdict('A5-STOP', !invalid, all);
await finish();

async function finish() {
  for (const x of [big, small]) await x.browser.close().catch(() => {});
  c.close(); await stopPid(tap); await stopAll();
  try { console.log(JSON.stringify({ bridgeCounts: readJson(`${dir}/bridge_stats_stop.json`).counts })); } catch {}
  process.exit(0);
}
