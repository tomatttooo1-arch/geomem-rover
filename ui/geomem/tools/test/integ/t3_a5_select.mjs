// T3 A5 真 runner 版（第 2 棒）：car_up sim 的真车栈＋真桥＋真前端。预注册 STAGE1_INTEG_PREREG.md 第 2 棒 §3.1/§3.4。
// 在车的网络命名空间里的试验台容器中跑（bridge/testbed/t3_tb.sh run）。GEOMEM_HUB=本趟 hub 地址，参数：<输出目录> <口令文件>。
// 第二只瓶由宿主侧放：本脚本写 <dir>/need_bottle_<i>，宿主 t3_tb.sh bottle 放好后写 <dir>/bottle_<i>.done。
import fs from 'node:fs';
import { startBridge, startPy, waitLog, waitHttp, stopAll, stopPid, RawClient, launch, wait, waitPill, hooks, toastsSince, toastMark, readJson, verdict, out as mk, APP, PY313, WT, HUB_ADDR } from './tb.mjs';
const [dir, tok] = process.argv.slice(2); mk(dir);
const log = (...a) => console.log(new Date().toISOString(), ...a);
const PILL = /仿真数据/;

startBridge(PY313, tok, dir, 'a5'); await waitHttp(`${APP}/`, 30000);
const c = await new RawClient().open();
const hello = await c.next(m => m.type === 'hello' && m.bridge.link === 'up', 20000).catch(() => null);
const ctlHello = c.last('hello');
log('hello', JSON.stringify(ctlHello));
if (!ctlHello || ctlHello.capabilities.control !== 'controller') { verdict('A5-T3', false, false, '桥不是控制者'); await stopAll(); process.exit(0); }
const tapFile = `${dir}/tap.jsonl`;
const tap = startPy(PY313, [`${WT}/ui/geomem/bridge/testbed/t3_state_tap.py`, HUB_ADDR, tok, tapFile], `${dir}/tap.log`); await waitLog(tap, /tap up/);
const tapRows = () => fs.readFileSync(tapFile, 'utf8').trim().split('\n').filter(Boolean).map(l => JSON.parse(l)).filter(r => r.seq !== undefined);
const lastTap = () => tapRows().at(-1);
const snaps = (from = 0, to = Infinity) => c.msgs.filter(m => m.type === 'snapshot' && m._t >= from && m._t <= to);
const bottles = () => (c.last('snapshot')?.objects || []).filter(o => o.className === 'bottle');
const untilSnap = async (pred, ms, from = Date.now()) => { const end = from + ms; while (Date.now() < end + 300) { const hit = snaps(from).find(pred); if (hit) return hit._t - from; await wait(30); } const hit = snaps(from, end).find(pred); return hit ? hit._t - from : null; };
async function waitFor(fn, ms, step = 250) { const t0 = Date.now(); while (Date.now() - t0 < ms) { const v = await fn(); if (v) return v; await wait(step); } return null; }
const visible = page => page.evaluate(() => { const vis = el => { const r = el.getBoundingClientRect(); let ok = r.top >= 0 && r.bottom <= innerHeight && r.left >= 0 && r.right <= innerWidth && r.width > 0; for (let p = el.parentElement; p; p = p.parentElement) { const cs = getComputedStyle(p); if (/(auto|scroll|hidden)/.test(cs.overflowY + cs.overflowX)) { const q = p.getBoundingClientRect(); if (r.top < q.top - 1 || r.bottom > q.bottom + 1 || r.left < q.left - 1 || r.right > q.right + 1) ok = false; } } return { ok, top: Math.round(r.top), bottom: Math.round(r.bottom) }; };
  return [...document.querySelectorAll('[data-candidate]')].map(b => ({ id: b.dataset.candidate, ...vis(b) })); });

const big = await launch({ viewport: { width: 1920, height: 1080 }, init: hooks() });
const small = await launch({ viewport: { width: 390, height: 844 }, init: hooks() });
for (const x of [big, small]) { await x.page.goto(`${APP}/?source=live`, { waitUntil: 'networkidle' }); log('pill', JSON.stringify(await waitPill(x.page, PILL, 60000))); }
await wait(1500);
const bp = big.page;

// 发起找物：先在页面上点；被挡（§3.4）就由原始客户端发同一条 find。返回 { t, via, note, rejected }
async function find() {
  await bp.fill('#search-input', '瓶子'); await bp.click('#find-button'); await wait(300);
  const btn = bp.locator('[data-find]').first();
  const n = await bp.locator('[data-find]').count(); const disabled = n ? await btn.isDisabled() : true;
  const note = await bp.evaluate(() => (document.getElementById('query-hint')?.innerText || '') + ' | ' + (document.getElementById('start-note')?.innerText || ''));
  const mark = await toastMark(bp);
  if (!disabled) {
    const before = c.msgs.length; const t = Date.now(); await btn.click();
    const sent = await waitFor(async () => (await toastsSince(bp, mark)).length || snaps(t).some(s => s.task.state !== 'idle' && s.task.state !== 'cancelled' && s.task.state !== 'succeeded' && s.task.state !== 'failed'), 1000, 50);
    if (sent) return { t, via: 'page', note };
    log('page find sent nothing in 1 s', note, JSON.stringify(await toastsSince(bp, mark)));
  }
  const t = Date.now(); const r = await c.command('find', { text: '瓶子' });
  return { t, via: 'raw', note, pageDisabled: disabled, ack: r.ack, result: r.result };
}
const isActive = s => ['planning', 'navigating', 'confirming', 'searching', 'ambiguous'].includes(s.task.state);
async function stopTask(label) {
  await bp.waitForFunction(() => !document.getElementById('stop-task').disabled && !document.querySelector('.task-controls').hidden, null, { timeout: 4000 }).catch(() => {});
  const before = lastTap(); const t = Date.now(); await bp.click('#stop-task', { timeout: 3000 }).catch(e => log('stop click failed', e.message));
  await wait(2500);
  const can = snaps(t).find(s => s.task.state === 'cancelled'); const per = tapRows().find(r => r.t * 1000 >= t && r.permit === false);
  const cancelMs = can ? can._t - t : null, permitMs = per ? Math.round(per.t * 1000 - t) : null;
  return { label, cancelMs, permitMs, stopMs: cancelMs === null || permitMs === null ? null : Math.max(cancelMs, permitMs), permitBefore: before?.permit, taskBefore: before?.task, t };
}
const seq = (from, to) => { const out = []; for (const s of snaps(from, to)) { const k = `${s.task.state}:${s.task.targetId ?? ''}:${s.task.candidates.length}`; if (out.at(-1)?.k !== k) out.push({ k, dt: s._t - from }); } return out; };
const tapSeq = (from, to) => { const out = []; for (const r of tapRows().filter(r => r.t * 1000 >= from && r.t * 1000 <= to)) { const k = `${r.task}:${r.target ?? ''}:${r.permit}`; if (out.at(-1)?.k !== k) out.push({ k, dt: Math.round(r.t * 1000 - from) }); } return out; };
async function settle() { await wait(5000); await waitFor(() => lastTap()?.permit === false, 30000); }

// ---- 阳性对照：恰 1 只瓶
const c0 = await waitFor(() => bottles().length === 1 ? bottles() : null, 180000, 500);
log('C0 bottles', JSON.stringify(bottles().map(o => [o.id, o.state, o.position])));
let control = { guard: !!c0 };
if (c0) {
  const B1 = c0[0].id;
  const f = await find();
  const hitMs = await untilSnap(s => ['planning', 'navigating', 'confirming', 'succeeded'].includes(s.task.state) && s.task.targetId === B1, 5000, f.t);
  const between = snaps(f.t, f.t + (hitMs ?? 5000));
  const noAmb = between.every(s => s.task.state !== 'ambiguous' && s.task.candidates.length === 0);
  const candDom = await bp.evaluate(() => document.querySelectorAll('[data-candidate]').length);
  await bp.screenshot({ path: `${dir}/a5t3_control_single.png` });
  const st = isActive(c.last('snapshot')) ? await stopTask('control') : null;
  control = { guard: true, B1, find: { via: f.via, note: f.note, ack: f.ack?.reasonCode ?? f.ack?.accepted }, hitMs, noAmb, candDom, seq: seq(f.t, f.t + 8000), stop: st, holds: hitMs !== null && noAmb && candDom === 0 };
}
log('CONTROL', JSON.stringify(control));
if (!control.holds) { verdict('A5-T3', false, false, '阳性对照不成立'); await finish(); }
await settle();

// ---- 第二只瓶（宿主放）
let placed = null;
for (const i of [1, 2]) {
  fs.writeFileSync(`${dir}/need_bottle_${i}`, `${Date.now()}\n`); log('need bottle', i);
  const done = await waitFor(() => fs.existsSync(`${dir}/bottle_${i}.done`), 90000, 500);
  log('bottle placed', i, done ? fs.readFileSync(`${dir}/bottle_${i}.done`, 'utf8').trim() : 'TIMEOUT');
  if (done && await waitFor(() => bottles().length === 2, 120000, 500)) { placed = i; break; }
  log('after P' + i, JSON.stringify(bottles().map(o => [o.id, o.position])));
}
log('G2', placed, JSON.stringify(bottles().map(o => [o.id, o.state, o.position])));
if (!placed) { fs.writeFileSync(`${dir}/need_fallback`, '1\n'); verdict('A5-T3', false, false, '两个位置都没进记忆（走退路：两瓶世界）'); await finish(); }

// ---- 正式 3 次
const rows = []; let all = true;
for (let k = 1; k <= 3; k++) {
  const g = await waitFor(() => bottles().length === 2, 60000, 500);
  if (!g) { rows.push({ rep: k, guard: false, bottles: bottles().length }); all = false; log('rep', k, 'guard fail'); continue; }
  let f, ambMs, tries = 0;
  do {
    tries++; f = await find();
    ambMs = await untilSnap(s => s.task.state === 'ambiguous' && s.task.candidates.length === 2 && s.task.candidates.every(x => x.className === 'bottle'), 5000, f.t);
    const rej = f.result?.reasonCode || ''; const tz = (await toastsSince(bp, 0)).slice(-2).join(' ');
    if (ambMs === null && (/cancel current operation/.test(rej) || /还有任务或导航在进行/.test(tz)) && tries < 4) { log('rep', k, 'query rejected busy, retry'); await wait(3000); continue; }
    break;
  } while (true);
  const vis = {};
  for (const [name, pg] of [['1920', bp], ['390', small.page]]) {
    await pg.waitForFunction(() => { const b = [...document.querySelectorAll('[data-candidate]')]; return b.length === 2 && b.every(x => !x.disabled); }, null, { timeout: 5000 }).catch(() => {});
    await pg.locator('#task-candidates').scrollIntoViewIfNeeded().catch(() => {}); await wait(400);
    vis[name] = await visible(pg);
    if (k === 1) await pg.screenshot({ path: `${dir}/a5t3_candidates_${name}.png` });
  }
  const second = vis['1920'][1]?.id;
  const t1 = Date.now(); await bp.locator(`[data-candidate="${second}"]`).click({ timeout: 3000 }).catch(e => log('select click failed', e.message));
  const tgtMs = await untilSnap(s => s.task.targetId === second, 3000, t1);
  await wait(500);
  const st = await stopTask(`rep${k}`);
  const visOk = ['1920', '390'].every(n => vis[n].length === 2 && vis[n].every(x => x.ok));
  const ok = ambMs !== null && visOk && second && tgtMs !== null && tgtMs <= 3000 && st.stopMs !== null && st.stopMs <= 2000;
  all &&= ok;
  rows.push({ rep: k, findVia: f.via, tries, ambMs, candidates: c.msgs.filter(m => m.type === 'snapshot' && m._t >= f.t).find(s => s.task.state === 'ambiguous')?.task.candidates.map(x => x.id), vis, selected: second, targetMs: tgtMs, stop: st, seqBridge: seq(t1, st.t + 2500), seqTap: tapSeq(t1, st.t + 2500), ok });
  log('REP', JSON.stringify(rows.at(-1)));
  if (k === 3) await bp.screenshot({ path: `${dir}/a5t3_cancelled_1920.png` });
  await settle();
}
const perr = big.page.errors.length + small.page.errors.length;
all &&= perr === 0;
for (const r of rows) console.log(JSON.stringify(r));
console.log(JSON.stringify({ control, placed, pageerrors: perr }));
verdict('A5-T3', true, all, rows.some(r => r.findVia === 'raw') || control.find?.via === 'raw' ? '（发起一步走原始客户端，§3.4）' : '');
await finish();

async function finish() {
  for (const x of [big, small]) await x.browser.close().catch(() => {});
  c.close(); await stopPid(tap); await stopAll();
  try { console.log(JSON.stringify({ bridgeCounts: readJson(`${dir}/bridge_stats_a5.json`).counts })); } catch {}
  process.exit(0);
}
