// GeoMem 展台页（10-17）：只读，连同源桥（或 ?ws=…），不发任何命令。
// 数据：协议 v2 快照（model.js 校验）＋ §2.10 展台附加字段 map.basemap / fpga / task.queryText（缺省一律按「没有」显示）。
// 可配置（URL 参数，外观待 owner 定，默认按最贴近效果稿 k_fv.png）：
//   labels=5          除目标外带标签的钉子数（离小车最近的 N 个）
//   invalid=hide|show 「上次位置已确认不在」的物体画不画钉子（默认不画；效果稿里画了）
//   cams=left|above   运镜两按钮放在切换按钮左侧 / 上方
//   roiMaxAgeMs=1000  核对记忆物体行：结果比这旧就显示「—」并改行名
//   view=first        开页就进第一视角；bench=N 底图到后连画 N 帧测帧时间（结果在 window.__kiosk.bench）
import { LiveAdapter } from './transport.js';
import { defaultEndpoint, endpointURL, snapshotFresh, secondsSince, sourceBanner, classZh, TERMINAL, activeTask } from './model.js';
import { KioskScene, KIOSK_ZH } from './kiosk_scene.js';

const Q = new URLSearchParams(location.search);
const num = (k, d) => { const v = Number(Q.get(k)); return Q.has(k) && Number.isFinite(v) ? v : d; };
export const CONFIG = {
  labels: Math.max(0, Math.min(30, Math.round(num('labels', 5)))),
  invalid: Q.get('invalid') === 'show' ? 'show' : 'hide',
  cams: Q.get('cams') === 'above' ? 'above' : 'left',
  roiMaxAgeMs: num('roiMaxAgeMs', 1000),
  mergeM: 0.35,                      // 显示合并：同类 0.35 m 内只画一个（效果稿 convert.py 同值，只为显示）
  bench: Math.max(0, Math.round(num('bench', 0))),
};
const $ = id => document.getElementById(id);
const zh = c => KIOSK_ZH[c] || classZh(c);
const scene = new KioskScene($('kiosk-canvas'), { onCamMode: m => markCam(m) });
const K = window.__kiosk = { scene, config: CONFIG, snapshot: null, receivedAt: 0, connection: 'connecting', basemap: { status: null, loaded: null, error: null }, bench: null };

if (CONFIG.cams === 'above') $('dock').classList.add('above');

// ------------------------------------------------------------------ view toggle and camera buttons
function setView(v) {
  document.body.dataset.view = v;
  for (const b of $('toggle').querySelectorAll('button')) b.classList.toggle('on', b.dataset.view === v);
  scene.setView(v, $('fp-mini'));
  render();
}
$('toggle').addEventListener('click', e => { const b = e.target.closest('button'); if (b) setView(b.dataset.view); });
$('cam-overview').addEventListener('click', () => scene.overview());
$('cam-follow').addEventListener('click', () => { if (!scene.follow()) flashNote('小车位置未知，暂时不能跟随'); });
function markCam(m) { $('cam-overview').classList.toggle('on', m === 'overview'); $('cam-follow').classList.toggle('on', m === 'follow'); $('kiosk-canvas').dataset.camMode = m; }
markCam('overview');

// ------------------------------------------------------------------ notes under the status card
function note(id, text, level = '') {
  const el = $(id); el.hidden = !text; el.textContent = text || ''; el.className = 'note' + (id === 'basemap-note' ? ' third' : '') + (level ? ' ' + level : '');
}
let flash = null;                          // 一次性提示（如「小车位置未知，暂时不能跟随」），显示 3 s
function flashNote(text, level = 'warn', ms = 3000) { flash = { text, level, until: performance.now() + ms }; render(); }
function placeNotes() { const r = $('status').getBoundingClientRect(); $('notes').style.top = `${r.bottom + 10 * kScale()}px`; }
function kScale() { return Math.min(innerWidth / 1920, innerHeight / 1080); }
function rescale() { document.documentElement.style.setProperty('--k', String(kScale())); placeNotes(); }
addEventListener('resize', rescale); rescale();

// ------------------------------------------------------------------ base map
const BASEMAP_WHY = { not_localized: { odom: '仅里程计定位', sim: '仿真数据', unknown: '判不出定位档' }, no_map_id: '桥没配置地图 id', mismatch: '当前地图与底图包不符' };
// 画底图的两种依据（§2.10）：定位档——底图包 sha256 须等于 map.id；里程档（10-18 owner）——桥标 basis
// mapping_start（车从建图起点同朝向出发，map->odom 恒等，与里程档导航同一假设），map.id 在里程档按协议为 null，
// 改核 tier 为 odom；两种都还要核清单的源库 sha256。
function basemapWant(s) {
  const bm = s.map?.basemap;
  if (!bm || bm.status !== 'ok' || typeof bm.sha256 !== 'string') return null;
  if (bm.basis === 'mapping_start') return bm.tier === 'odom' ? bm.sha256 : null;
  return bm.sha256 === s.map.id ? bm.sha256 : null;
}
async function syncBasemap(s) {
  const bm = s.map?.basemap, want = basemapWant(s);
  K.basemap.status = bm?.status ?? null;
  // 注意 want 可能是 null：加载完成后 loading 也回到 null，若不先判 want，「null === null」会让底图在
  // 里程档/库不符时永远清不掉（第 3 棒 K3b 第 1 趟判据抓到的缺陷）。
  if (want && (want === K.basemap.loaded || want === K.basemap.loading)) return;
  if (!want) { K.basemap.loading = null; if (K.basemap.loaded) { scene.clearBasemap(); K.basemap.loaded = null; } return; }
  K.basemap.loading = want; K.basemap.error = null;
  try {
    const [man, cells] = await Promise.all([fetch(bm.url).then(r => { if (!r.ok) throw Error(`清单 HTTP ${r.status}`); return r.json(); }),
      fetch(bm.cellsUrl).then(r => { if (!r.ok) throw Error(`格数据 HTTP ${r.status}`); return r.arrayBuffer(); })]);
    if (man?.format !== 'geomem-basemap' || man.version !== 1 || man.source_db?.sha256 !== want) throw Error('底图包清单与地图 id 不符');
    if (K.basemap.loading !== want) return;
    scene.setBasemap(man, cells); K.basemap.loaded = want; K.basemap.name = man.source_db.name;
    $('kiosk-canvas').dataset.basemapCells = String(man.files.cells.count); $('kiosk-canvas').dataset.basemapSha = want;
    if (CONFIG.bench) { await new Promise(r => requestAnimationFrame(r)); K.bench = scene.bench(CONFIG.bench); }
  } catch (e) { K.basemap.error = String(e.message || e); }
  finally { if (K.basemap.loading === want) K.basemap.loading = null; render(); }
}
function basemapNote(s) {
  const bm = s?.map?.basemap;
  const basis = `底图校验依据：${bm?.idSource === 'car' ? '车上上报的地图 id' : '操作员配置'}`;
  if (K.basemap.loaded && bm?.status === 'ok' && bm.basis === 'mapping_start') return [`底图：${K.basemap.name || '地图库'} · 按建图起点对齐，未做定位校正 · ${basis}`, ''];
  if (K.basemap.loaded && bm?.status === 'ok') return [`底图：${K.basemap.name || '地图库'} · ${basis}`, ''];
  if (K.basemap.loading) return ['正在加载底图…', 'info'];
  if (K.basemap.error) return [`未加载底图（${K.basemap.error}）`, 'warn'];
  if (!bm) return ['未加载底图（桥没有配置底图包）', 'warn'];
  const why = bm.status === 'not_localized' ? BASEMAP_WHY.not_localized[bm.tier] || '未定位'
    : bm.status === 'ok' ? '底图包与地图 id 不一致'
    : bm.status === 'mismatch' && Array.isArray(bm.loaded) && !bm.loaded.length ? '还没有底图包，建图结束后生成'
    : BASEMAP_WHY[bm.status] || bm.status;
  return [`未加载底图（${why}）`, 'warn'];
}

// ------------------------------------------------------------------ pins and labels
function pinsFor(s, robot) {
  const t = s.task, target = t && !['idle'].includes(t.state) ? t.targetId : null;
  // 「上次位置已确认不在」默认不画（目标也不例外：那个位置已知是空的，画光柱会误导；与实时语义一致，控制器 10-17 裁定）
  let objs = (s.objects || []).filter(o => CONFIG.invalid === 'show' || o.state !== 'position_invalid');
  objs = objs.slice().sort((a, b) => (b.id === target) - (a.id === target) || (b.lastSeen ?? -Infinity) - (a.lastSeen ?? -Infinity) || (a.id < b.id ? -1 : 1));
  const kept = [];
  for (const o of objs) if (o.id === target || !kept.some(k => k.className === o.className && Math.hypot(k.position[0] - o.position[0], k.position[1] - o.position[1]) < CONFIG.mergeM)) kept.push(o);
  const others = kept.filter(o => o.id !== target);
  const near = robot ? others.slice().sort((a, b) => Math.hypot(a.position[0] - robot.x, a.position[1] - robot.y) - Math.hypot(b.position[0] - robot.x, b.position[1] - robot.y)) : others;
  const labeled = new Set(near.slice(0, CONFIG.labels).map(o => o.id));
  return kept.map(o => {
    const isT = o.id === target;
    let sub = o.state === 'historical_memory' ? '历史记忆' : '记忆';
    if (isT) { const sec = secondsSince(s, o.lastSeen); sub = sec === null ? '记忆位置' : `记忆位置 · ${sec < 60 ? `${Math.floor(sec)} 秒` : sec < 3600 ? `${Math.floor(sec / 60)} 分钟` : `${Math.floor(sec / 3600)} 小时`}前见过`; }
    return { id: o.id, className: o.className, position: o.position, isTarget: isT, labeled: isT || labeled.has(o.id), name: (isT ? '目标 · ' : '') + zh(o.className), sub };
  });
}

// ------------------------------------------------------------------ status card / FPGA panel
const STATE_WORD = { planning: '正在规划路线', navigating: '正在前往', searching: '按搜索路线寻找', confirming: '正在现场确认', ambiguous: '记忆里有多个，请在控制台选择', paused: '已暂停' };
// N（「记忆中有 N 个<物名>」）＝地图上实际画出的该类钉子数：用与画钉子同一份合并/过滤后的 pins 计数，
// 不用快照未合并条数——评委看到的两处数字必须对得上（控制器 10-17 裁定）。
function statusCard(s, robot, pins) {
  const t = s?.task, objs = s?.objects || [];
  const tgt = t?.targetId ? objs.find(o => o.id === t.targetId) : null;
  const what = tgt ? zh(tgt.className) : (t?.queryText || null);
  const drawn = cls => (pins || []).filter(p => p.className === cls).length;
  const tgtPin = tgt ? (pins || []).find(p => p.id === tgt.id) : null;
  const parts = [];
  if (tgt) parts.push(`记忆中有 ${drawn(tgt.className)} 个${zh(tgt.className)}`);
  if (tgtPin && robot && robot.pose === 'map') parts.push(`距小车 ${Math.hypot(tgt.position[0] - robot.x, tgt.position[1] - robot.y).toFixed(1)} m`);
  if (!t || t.state === 'idle') return { dot: 'idle', k: '小车待命', v: '等待寻物指令', s: `记忆中共有 ${(pins || []).length} 个物体` };   // 同 N：数画出的钉子
  if (activeTask(t)) { parts.push(STATE_WORD[t.state] || t.message);
    const v = t.state === 'ambiguous' ? `请选择目标${what ? '：' + what : ''}` : t.state === 'searching' && !tgt ? `正在搜索${what ? '：' + what : ''}` : `正在寻找${what ? '：' + what : ''}`;
    return { dot: '', k: '寻物任务进行中', v, s: parts.join(' · ') }; }
  if (TERMINAL.has(t.state)) {
    const v = t.state === 'succeeded' ? `已找到${what ? '：' + what : ''}` : t.state === 'cancelled' ? '寻物已停止' : `未能找到${what ? '：' + what : ''}`;
    return { dot: t.state === 'succeeded' ? 'ok' : 'bad', k: '寻物任务结束', v, s: [t.message, ...parts.slice(1)].filter(Boolean).join(' · ') }; }
  return { dot: 'idle', k: '小车状态', v: t.message || '—', s: '' };
}
function fmtMs(v) { return v < 10 ? v.toFixed(1) : String(Math.round(v)); }
function fpgaPanel(s, fresh) {
  const f = s?.fpga, set = (id, k, v, cls) => { const row = $(id); if (k) row.querySelector('.k').textContent = k; const el = row.querySelector('.v'); el.textContent = v; el.className = 'v' + (cls ? ' ' + cls : ''); };
  const up = !!(f && f.linkOk && fresh);
  $('fpga-title').textContent = up || !f ? 'FPGA（KV260）实时' : 'FPGA（KV260）· 链路断开';
  set('fpga-geo', null, up && typeof f.geometryMs === 'number' ? `${fmtMs(f.geometryMs)} ms` : '—', up && typeof f.geometryMs === 'number' ? '' : 'na');
  const roiFresh = up && typeof f.roiRows === 'number' && typeof f.roiAgeMs === 'number' && f.roiAgeMs <= CONFIG.roiMaxAgeMs;
  set('fpga-roi', roiFresh ? '本帧核对记忆物体' : '最近一次核对物体', roiFresh ? `${f.roiRows} 个` : '—', roiFresh ? '' : 'na');
  const hb = up ? f.heartbeatOn : null;
  set('fpga-hb', null, hb === true ? '正常' : hb === false ? '断开' : '未知', hb === true ? 'ok' : hb === false ? 'bad' : 'na');
}

// ------------------------------------------------------------------ render everything from the newest snapshot
function robotOf(s) { const r = s?.robot; return r && r.pose !== 'none' ? { x: r.position[0], y: r.position[1], yaw: r.yaw, pose: r.pose } : null; }
let lastScope = null;
function render() {
  const s = K.snapshot, fresh = snapshotFresh(s, K.receivedAt);
  const robot = s ? robotOf(s) : null;
  // banner: connection / freshness / data source / pose
  let ban = null;
  if (K.connection !== 'connected') ban = [K.connection === 'connecting' ? '正在连接桥接服务…' : '与桥接服务的连接中断，正在重连', 'error'];
  else if (!s) ban = ['等待小车数据…', 'info'];
  else if (!fresh) ban = [s.link?.state !== 'up' ? '桥与小车的连接中断，画面停在最后一帧' : '数据已过期，画面停在最后一帧', 'error'];
  else if (sourceBanner(s.source, s.sourceDetail)) ban = sourceBanner(s.source, s.sourceDetail);
  else if (s.robot && s.robot.pose === 'none') ban = ['小车尚未定位（不显示小车位置）', 'warn'];
  else if (!s.robot) ban = ['小车位置未知（不显示小车位置）', 'warn'];
  else if (s.robot?.pose === 'odometry') ban = ['仅里程计定位：小车位置是里程推算，可能有偏差', 'info'];
  if (flash && performance.now() < flash.until) ban = [flash.text, flash.level]; else flash = null;
  note('banner', ban?.[0] || '', ban?.[1] || '');
  const [bmText, bmLevel] = basemapNote(s); note('basemap-note', bmText, bmLevel);
  if (s) {
    const scope = `${s.memory?.scope}|${s.map.frame}|${s.map.id}`;
    if (scope !== lastScope) { if (lastScope !== null) scene.resetTrail(); lastScope = scope; }
    scene.setRobot(robot);
  }
  const pins = s ? pinsFor(s, robot) : [];
  if (s) scene.setPins(pins);
  const card = statusCard(s, robot, pins);
  $('status-dot').className = 'dot' + (card.dot ? ' ' + card.dot : '');
  $('status-k').textContent = document.body.dataset.view === 'first' ? '小车第一视角' : card.k;
  $('status-v').textContent = document.body.dataset.view === 'first' ? '车载画面未接入' : card.v;
  $('status-s').textContent = document.body.dataset.view === 'first' ? [card.v, card.s].filter(Boolean).join(' · ') : card.s;
  fpgaPanel(s, fresh);
  placeNotes();
  const cv = $('kiosk-canvas').dataset;
  cv.trailPoints = String(scene.trail.length); cv.cloudPoints = String(scene.cloudPoints); cv.pins = String(scene.pins.children.length);
  cv.pinClasses = JSON.stringify(pins.reduce((a, p) => (a[p.className] = (a[p.className] || 0) + 1, a), {}));
  if (!scene.basemap) { cv.basemapCells = '0'; cv.basemapSha = ''; }
  cv.carX = robot ? robot.x.toFixed(3) : ''; cv.carY = robot ? robot.y.toFixed(3) : '';
}

// ------------------------------------------------------------------ connection
const url = (() => { try { return endpointURL(Q.get('ws') || defaultEndpoint(), location.protocol === 'https:'); } catch { return defaultEndpoint(); } })();
const adapter = new LiveAdapter(url);
adapter.addEventListener('connection', e => { K.connection = e.detail.state === 'connected' ? 'connected' : e.detail.state === 'connecting' ? 'connecting' : 'disconnected'; render(); });
adapter.addEventListener('snapshot', e => {
  K.snapshot = e.detail; K.receivedAt = performance.now();
  syncBasemap(e.detail); render();
});
adapter.addEventListener('cloud', e => { scene.setCloud(e.detail); render(); });
adapter.addEventListener('error', e => { K.lastError = e.detail; });
adapter.connect();
setInterval(render, 500);                 // freshness / seen-ago text even when no snapshot arrives
if (Q.get('view') === 'first') setView('first'); else render();
