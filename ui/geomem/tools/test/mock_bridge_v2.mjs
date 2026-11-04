// GeoMem 协议第二版模拟桥（只供测试；不是 ui/geomem/bridge/ 的实现）。
// WS: ws://127.0.0.1:<port>/ws ；控制: http://127.0.0.1:<port+1>/ctl?...
// 控制参数（可组合）：
//   caps=<json 片段，合并进能力表>  source=live|replay|sim|inconsistent|unknown  link=up|stale|down
//   tier=map|odom|odomrobot   points=N（云点数）  objects=N（物体数，前 2 个固定为两只杯子）  rate=Hz
//   op=pause_stream|resume_stream|once|drop|reset|scope|cloud|hello|active
//   noack=1|0  reject=<code>|0  result=accepted|rejected|unknown|none  inject=<kind>
//   GET /ctl?op=state 返回 {cmdTotal, cmdThisConn[], serverTime, objects:[{id,lastSeen}], ...}
import http from 'node:http';
import { WebSocketServer } from 'ws';
import fs from 'node:fs';
const PORT = Number(process.argv[2] || 19190);
const LOG = process.argv[3] || '/dev/null';
const log = (...a) => fs.appendFileSync(LOG, new Date().toISOString() + ' ' + a.join(' ') + '\n');
const T0 = process.hrtime.bigint();
// 车上 runner 的 ROS 时间轴：故意用一个与墙钟无关的数（模拟 Nano 无时钟电池、墙钟停在 1970 附近）。
const serverNow = () => 86400 + Number(process.hrtime.bigint() - T0) / 1e9;
const cfg = { caps: { findText: true, findById: false, select: true, cancel: true, pause: false, resume: false, estop: false, control: 'controller' },
  source: 'live', link: 'up', tier: 'map', points: 3000, objects: 6, rate: 5, stream: true, noack: false, reject: null, result: 'accepted' };
let scope = 'boot-aaaaaaaa-1111', scopeN = 1, revision = 1, querySeq = 0, cloudRev = 1;
let task = null, path = [], robot = { position: [4.0, 1.0, 0], yaw: 0.3, speed: 0, status: '待命' };
let validCloudN = 0;
let cmdTotal = 0, ticks = 0, target = null;
const baseSeen = serverNow();
function makeObjects(n) {
  const out = [
    { id: 'object-0001', name: null, className: 'cup', color: 'blue', aliases: null, location: null, position: [9.5, 2.6, 0.78], confidence: 0.66, lastSeen: baseSeen - 12, state: 'current_confirmed' },
    { id: 'object-0002', name: null, className: 'cup', color: 'white', aliases: null, location: null, position: [13.0, 6.0, 0.9], confidence: null, lastSeen: baseSeen - 40, state: 'historical_memory' },
  ];
  const classes = ['backpack', 'book', 'mouse', 'keyboard', 'bottle', 'cell phone', 'chair', 'laptop'];
  let s = 11; const r = () => ((s = (s * 16807) % 2147483647) - 1) / 2147483646;
  for (let i = 2; i < n; i++) out.push({ id: `object-${String(i + 1).padStart(4, '0')}`, name: null, className: classes[i % classes.length], color: null, aliases: null, location: null,
    position: [+(2 + r() * 12).toFixed(2), +(-1 + r() * 8).toFixed(2), +(r() * 1.2).toFixed(2)], confidence: +(0.3 + r() * 0.6).toFixed(2), lastSeen: baseSeen - 5 - i * 3, state: i % 7 === 3 ? 'position_invalid' : i % 7 === 5 ? 'temporarily_occluded' : 'historical_memory' });
  return out;
}
let objects = makeObjects(cfg.objects);
const ZH = { cup: '杯子', bottle: '瓶子', backpack: '书包', book: '书', mouse: '鼠标', keyboard: '键盘', 'cell phone': '手机', chair: '椅子', laptop: '笔记本电脑' };
// 二进制云：房间 12 m × 8 m，偏离原点（检验自动框视角）。
function encodeCloud({ n, rev, frame, mapId, source = 'grid', kind = 'room' }, corrupt = null) {
  const pos = new Float32Array(n * 3), col = new Uint8Array(n * 3);
  let s = 7; const r = () => ((s = (s * 16807) % 2147483647) - 1) / 2147483646;
  for (let i = 0; i < n; i++) {
    let x, y, z;
    if (kind === 'rolling') { x = robot.position[0] - 2 + r() * 4; y = robot.position[1] - 2 + r() * 4; z = (i % 4) * 0.1 + 0.05; }
    else if (kind === 'empty') { x = r() * 8 - 4; y = r() * 8 - 4; z = 0; }
    else { const k = i % 4; if (k === 0) { x = 2 + r() * 12; y = -1 + r() * 8; z = 0; } else if (k === 1) { x = 2 + r() * 12; y = 7; z = r() * 2; } else if (k === 2) { x = 2; y = -1 + r() * 8; z = r() * 2; } else { x = 8 + r() * 2; y = 2 + r(); z = 0.7 + r() * 0.05; } }
    pos.set([x, y, z], i * 3); col.set([80 + (i % 120), 120 + (i % 80), 170], i * 3);
  }
  const header = { type: 'cloud', version: 2, mapId, rev, frame, count: n, source, encoding: 'f32xyz+u8rgb' };
  if (corrupt === 'badheader') header.version = 1;
  if (corrupt === 'toomany') header.count = 60001;
  if (corrupt === 'nonfinite') pos[5] = Infinity;
  const hb = Buffer.from(JSON.stringify(header), 'utf8');
  const off = Math.ceil((4 + hb.length) / 4) * 4;
  let total = off + 15 * n; if (corrupt === 'toomany') total = off + 15 * 60001;
  if (corrupt === 'short') total -= 1;
  const buf = Buffer.alloc(total);
  buf.writeUInt32LE(hb.length, 0); hb.copy(buf, 4);
  if (corrupt === 'toomany') return buf;
  Buffer.from(pos.buffer).copy(buf, off, 0, Math.min(pos.byteLength, total - off));
  if (corrupt !== 'short') Buffer.from(col.buffer).copy(buf, off + 12 * n);
  return buf;
}
const MAP_ID = 'a'.repeat(64);
function tierInfo() {
  if (cfg.tier === 'map') return { map: { id: MAP_ID, name: '模拟桥实验室', frame: 'map', units: 'm', cloudRev: `global:${cloudRev}`, placeholder: false }, pose: 'map' };
  return { map: { id: null, name: null, frame: 'odom', units: 'm', cloudRev: `rolling:${cloudRev}`, placeholder: true }, pose: cfg.tier === 'odomrobot' ? 'odometry' : null };
}
function cloudsFor() {
  if (cfg.tier === 'map') return [encodeCloud({ n: cfg.points, rev: `global:${cloudRev}`, frame: 'map', mapId: MAP_ID })];
  // odom 档：先发替身全局图（应被前端拒画），再发滚动栅格（应画出）
  return [encodeCloud({ n: 400, rev: 'global:9', frame: 'odom', mapId: null, kind: 'empty' }), encodeCloud({ n: 1600, rev: `rolling:${cloudRev}`, frame: 'odom', mapId: null, kind: 'rolling' })];
}
function hello() { return { type: 'hello', version: 2, bridge: { id: 'mockbridge', hubBootId: 'hub-boot-1', link: cfg.link }, source: cfg.source, capabilities: cfg.caps }; }
function snapshot(ws, over = {}) {
  ws.seq = (ws.seq ?? -1) + 1;
  const t = tierInfo();
  const r = t.pose ? { position: [...robot.position], yaw: robot.yaw, speed: robot.speed, pose: t.pose, poseAgeMs: 60, status: robot.status } : null;
  return { type: 'snapshot', version: 2, seq: ws.seq, serverTime: serverNow(), source: cfg.source, link: { state: cfg.link, hubAgeMs: 40 }, map: t.map, robot: r,
    memory: { scope, revision, ageMs: 180 }, objects, path, task, ...over };
}
const nameOf = o => `${ZH[o.className] || o.className}`;   // 10-22: no colour in names
function step() {
  if (!task || !['searching', 'navigating', 'confirming'].includes(task.state)) return;
  ticks++;
  if (ticks <= 2) { task.state = 'searching'; task.reasonCode = 'search'; task.message = '正在按搜索路线寻找'; }
  else if (ticks <= 10) { task.state = 'navigating'; task.reasonCode = 'reobserve'; task.message = '正在前往目标附近'; robot.speed = 0.25; robot.status = '行驶中'; const f = (ticks - 2) / 8; robot.position = [4 + f * (target.position[0] - 4.8), 1 + f * (target.position[1] - 1.6), 0]; path = [[...robot.position], [target.position[0] - .8, target.position[1] - .6, 0]]; }
  else if (ticks <= 13) { task.state = 'confirming'; task.reasonCode = 'fresh_visual_evidence_required'; task.message = '已到观察点，正在现场确认'; robot.speed = 0; robot.status = '确认目标'; }
  else { task.state = 'succeeded'; task.reasonCode = 'fresh_matching_observation'; task.message = '已在现场再次看到目标'; robot.status = '待命'; path = []; }
}
const wss = new WebSocketServer({ host: '127.0.0.1', port: PORT, path: '/ws', maxPayload: 1 << 20 });
const conns = [];
function sendClouds(c) { for (const b of cloudsFor()) c.send(b); }
wss.on('connection', (ws, req) => {
  ws.cmds = 0; conns.push(ws);
  log('connect origin=' + req.headers.origin + ' conn#' + conns.length);
  ws.send(JSON.stringify(hello()));
  sendClouds(ws);
  ws.send(JSON.stringify(snapshot(ws)));
  ws.on('message', raw => {
    ws.cmds++; cmdTotal++;
    let m; try { m = JSON.parse(raw); } catch { log('bad cmd'); return; }
    log('cmd ' + JSON.stringify(m));
    if (cfg.noack) return;
    const ack = (accepted, code, reason) => ws.send(JSON.stringify({ type: 'ack', version: 2, id: m.id, accepted, status: accepted ? 'forwarded' : 'rejected', ...(code ? { code } : {}), ...(reason ? { reason } : {}) }));
    const result = (status, reason = '', reasonCode = null) => setTimeout(() => { for (const c of wss.clients) c.send(JSON.stringify(snapshot(c))); if (cfg.result !== 'none') ws.send(JSON.stringify({ type: 'result', version: 2, id: m.id, status, reason, reasonCode })); }, 300);
    if (cfg.reject) return ack(false, cfg.reject, '');
    const p = m.payload || {};
    if (m.action === 'find') {
      if (task && ['ambiguous', 'searching', 'navigating', 'confirming'].includes(task.state)) return ack(true), result('rejected', '还有任务或导航在进行，请先停止', 'query_rejected');
      const q = String(p.text || '').toLowerCase();
      const hits = objects.filter(o => q && (o.className === q || ZH[o.className] === q || q.includes(o.className)));
      task = { id: `hub-boot-1:${++querySeq}`, state: 'idle', reasonCode: null, message: '', targetId: null, ageMs: 120, candidates: [] };
      if (hits.length > 1) Object.assign(task, { state: 'ambiguous', reasonCode: 'candidate_selection_required', message: `记忆里有 ${hits.length} 个符合的物品，请选择要找哪一个`, candidates: hits.map(o => ({ id: o.id, name: nameOf(o), className: o.className, color: o.color, position: o.position, state: o.state })) });
      else if (hits.length === 1) { target = hits[0]; ticks = 0; Object.assign(task, { state: 'searching', reasonCode: 'search', targetId: target.id, message: '正在按搜索路线寻找' }); }
      else { target = { position: [10, 4, 0] }; ticks = 0; Object.assign(task, { state: 'searching', reasonCode: 'search', message: '正在按搜索路线寻找' }); }
      ack(true); result(cfg.result === 'accepted' ? 'accepted' : cfg.result);
    } else if (m.action === 'select') {
      if (p.scope !== undefined && p.scope !== scope) return ack(false, 'scope_changed', '记忆已重置，请重新选择');
      if (!task || task.id !== p.taskId) return ack(false, 'stale_task');
      if (task.state !== 'ambiguous') return ack(false, 'not_ambiguous');
      const c = task.candidates.find(c => c.id === p.candidateId); if (!c) return ack(false, 'no_such_candidate');
      target = objects.find(o => o.id === c.id); ticks = 0;
      Object.assign(task, { state: 'searching', reasonCode: 'search', targetId: c.id, candidates: [], message: '正在按搜索路线寻找' });
      ack(true); result('accepted');
    } else if (m.action === 'cancel') {
      if (task && !['succeeded', 'failed', 'cancelled'].includes(task.state)) Object.assign(task, { state: 'cancelled', reasonCode: 'user_cancelled', message: '已按要求停止', candidates: [] });
      robot.speed = 0; robot.status = '待命'; path = []; ack(true); result('done');
    } else if ((m.action === 'pause' || m.action === 'resume') && !cfg.caps.pause) { ack(false, 'not_supported'); }
    else ack(false, 'bad_args');
  });
});
let lastStep = Date.now();
function loop() {
  if (Date.now() - lastStep >= 1000) { step(); lastStep = Date.now(); }
  if (cfg.stream) for (const c of wss.clients) if (c.readyState === 1) c.send(JSON.stringify(snapshot(c)));
  setTimeout(loop, 1000 / cfg.rate);
}
loop();
// 非法帧（每个都应被拒、画面不变）
const injections = {
  version1: ws => JSON.stringify({ ...snapshot(ws), version: 1, map: { ...tierInfo().map, name: 'BAD-version1' } }),
  nantext: ws => JSON.stringify(snapshot(ws, { map: { ...tierInfo().map, name: 'BAD-nan' } })).replace(/"yaw":[-0-9.e]+/, '"yaw":NaN'),
  infinity: ws => JSON.stringify(snapshot(ws, { map: { ...tierInfo().map, name: 'BAD-inf' } })).replace(/"yaw":[-0-9.e]+/, '"yaw":1e400'),
  badframe: ws => JSON.stringify(snapshot(ws, { map: { ...tierInfo().map, frame: 'base_link', name: 'BAD-frame' } })),
  dupid: ws => JSON.stringify(snapshot(ws, { map: { ...tierInfo().map, name: 'BAD-dup' }, objects: [...objects, { ...objects[0] }] })),
  toomanyobjects: ws => JSON.stringify(snapshot(ws, { map: { ...tierInfo().map, name: 'BAD-1001' }, objects: Array.from({ length: 1001 }, (_, i) => ({ ...objects[0], id: 'o' + i })) })),
  pointsinsnapshot: ws => JSON.stringify(snapshot(ws, { map: { ...tierInfo().map, name: 'BAD-points' }, points: [[0, 0, 0, 1, 1, 1]] })),
  seqback: ws => { const s = snapshot(ws, { map: { ...tierInfo().map, name: 'BAD-seqback' } }); s.seq = Math.max(0, ws.seq - 5); return JSON.stringify(s); },
  notjson: () => '{"type":"snapshot", oops',
  cloud_badheader: () => encodeCloud({ n: 500, rev: tierInfo().map.cloudRev, frame: 'map', mapId: MAP_ID }, 'badheader'),
  cloud_short: () => encodeCloud({ n: 500, rev: tierInfo().map.cloudRev, frame: 'map', mapId: MAP_ID }, 'short'),
  cloud_nonfinite: () => encodeCloud({ n: 500, rev: tierInfo().map.cloudRev, frame: 'map', mapId: MAP_ID }, 'nonfinite'),
  cloud_toomany: () => encodeCloud({ n: 500, rev: tierInfo().map.cloudRev, frame: 'map', mapId: MAP_ID }, 'toomany'),
  cloud_valid: () => encodeCloud({ n: 700 + (++validCloudN), rev: tierInfo().map.cloudRev, frame: 'map', mapId: MAP_ID }),
  // 合法的「可空字段全为 null」帧（应被采用）
  nullfields: ws => JSON.stringify(snapshot(ws, { map: { ...tierInfo().map, name: 'NULLFIELDS-OK' }, objects: objects.map(o => ({ ...o, name: null, aliases: null, location: null, confidence: null, lastSeen: null })) })),
  // 合法帧：map.name 带标记，用来证明紧随其后的合法帧被采用
  valid: ws => JSON.stringify(snapshot(ws, { map: { ...tierInfo().map, name: 'VALID-' + ws.seq } })),
};
http.createServer((req, res) => {
  const u = new URL(req.url, 'http://x'); const q = Object.fromEntries(u.searchParams);
  if (q.caps) Object.assign(cfg.caps, JSON.parse(q.caps));
  if (q.source) cfg.source = q.source;
  if (q.link) cfg.link = q.link;
  if (q.rate) cfg.rate = Number(q.rate);
  if (q.objects) { cfg.objects = Number(q.objects); objects = makeObjects(cfg.objects); }
  if (q.noack) cfg.noack = q.noack === '1';
  if (q.reject) cfg.reject = q.reject === '0' ? null : q.reject;
  if (q.result) cfg.result = q.result;
  let resendCloud = false;
  if (q.points) { cfg.points = Number(q.points); cloudRev++; resendCloud = true; }
  if (q.tier) { cfg.tier = q.tier; cloudRev++; resendCloud = true; }
  if (q.caps || q.source || q.link) for (const c of wss.clients) c.send(JSON.stringify(hello()));
  if (resendCloud || q.op === 'cloud') for (const c of wss.clients) sendClouds(c);
  if (q.op === 'pause_stream') cfg.stream = false;
  if (q.op === 'resume_stream') cfg.stream = true;
  if (q.op === 'once') for (const c of wss.clients) c.send(JSON.stringify(snapshot(c)));
  if (q.op === 'drop') for (const c of wss.clients) c.terminate();
  if (q.op === 'reset') { task = null; path = []; robot.position = [4, 1, 0]; robot.status = '待命'; robot.speed = 0; }
  if (q.op === 'active') { task = { id: `hub-boot-1:${++querySeq}`, state: 'navigating', reasonCode: 'reobserve', message: '正在前往目标附近', targetId: objects[0].id, ageMs: 100, candidates: [] }; target = objects[0]; ticks = 3; }
  if (q.op === 'scope') { scope = `boot-${(++scopeN).toString(16).padStart(8, 'b')}-2222`; revision = 1; }
  if (q.inject) { const fn = injections[q.inject]; for (const c of wss.clients) c.send(fn(c)); }
  if (q.op === 'state') { res.end(JSON.stringify({ cmdTotal, cmdThisConn: conns.map(c => c.cmds), clients: wss.clients.size, serverTime: serverNow(), objects: objects.map(o => ({ id: o.id, lastSeen: o.lastSeen })), task, scope, cfg })); return; }
  log('ctl ' + req.url);
  res.end(JSON.stringify({ ok: true, clients: wss.clients.size, task, scope }));
}).listen(PORT + 1, '127.0.0.1');
log('listening ' + PORT);
