// 10-26 搜索热度图（桥的 belief 消息，协议见 ui/geomem/vehicle_interface_protocol.md「搜索热度图」）：纯函数，无 three.js，node 可测。
// 格子约定同 nav_msgs/OccupancyGrid：cells 行优先，第 0 行第 0 列的左下角在 origin [x, y, yaw]（map 系，米/弧度），
// 行沿 +y、列沿 +x（再按 yaw 转）。界面坐标同 scene.js 的 world()：协议 (x, y, z) → three (x, z, -y)。
export const BELIEF_PHASES = new Set(['preview', 'search', 'done', 'none']);
export const BELIEF_MAX_CELLS = 200000;
export const DONE_HOLD_MS = 3000, FADE_MS = 1500, STALE_DONE_MS = 8000;
export const BELIEF_Y = 0.03;   // 地面上 3 cm，压在底图地面点之上

const finite = v => typeof v === 'number' && Number.isFinite(v);
const pose3 = v => Array.isArray(v) && v.length === 3 && v.every(finite) ? v.slice() : null;

function b64bytes(text) {
  const bin = atob(text), out = new Int8Array(bin.length);
  for (let i = 0; i < bin.length; i++) out[i] = (bin.charCodeAt(i) << 24) >> 24;   // int8：0xFF → -1
  return out;
}

// 校验并解出桥的 belief 消息；不合规抛错（transport 记为「数据未被采用」）。
export function decodeBelief(m) {
  if (!m || typeof m !== 'object' || m.type !== 'belief' || !BELIEF_PHASES.has(m.phase)) throw Error('热度图类型无效');
  const {width: w, height: h} = m;
  if (!Number.isSafeInteger(w) || !Number.isSafeInteger(h) || w < 0 || h < 0 || w * h > BELIEF_MAX_CELLS) throw Error('热度图尺寸无效');
  const origin = pose3(m.origin);
  if (!finite(m.resolution) || m.resolution <= 0 || !origin) throw Error('热度图原点或分辨率无效');
  if (typeof m.cells !== 'string') throw Error('热度图格数据无效');
  let values;
  try { values = b64bytes(m.cells); } catch { throw Error('热度图格数据无效'); }
  if (values.length !== w * h) throw Error('热度图格数与尺寸不符');
  return {phase: m.phase, task: Number.isSafeInteger(m.task) ? m.task : null, seq: Number.isSafeInteger(m.seq) ? m.seq : null,
    frame: m.frame === 'odom' ? 'odom' : 'map', resolution: m.resolution, width: w, height: h, origin, values,
    next: pose3(m.next), seen: Array.isArray(m.seen) ? m.seen.map(pose3).filter(Boolean).slice(0, 200) : [],
    massLeft: finite(m.massLeft) ? m.massLeft : null, result: typeof m.result === 'string' ? m.result : null,
    reason: typeof m.reason === 'string' ? m.reason : null, ageMs: finite(m.ageMs) ? m.ageMs : 0};
}

// 格内点 (列 u, 行 v)，u, v 以格为单位、从 origin 角起算 → map 系 [x, y]
export function gridPoint(b, u, v) {
  const [ox, oy, yaw] = b.origin, c = Math.cos(yaw), s = Math.sin(yaw), lx = u * b.resolution, ly = v * b.resolution;
  return [ox + c * lx - s * ly, oy + s * lx + c * ly];
}
export const cellCenter = (b, row, col) => gridPoint(b, col + 0.5, row + 0.5);
// 四角，依次对应纹理 uv (0,0) (1,0) (1,1) (0,1)；DataTexture 第 0 行在 v=0，即 origin 那一边
export const beliefCorners = b => [gridPoint(b, 0, 0), gridPoint(b, b.width, 0), gridPoint(b, b.width, b.height), gridPoint(b, 0, b.height)];
// map 系 → three 世界坐标（同 scene.js world()）
export const toScene = ([x, y], z = BELIEF_Y) => [x, z, -y];

// 每格 RGBA（行优先，与 DataTexture 同序）：越可能越红越不透明，看过的（低值）淡黄近透明，范围外全透明；预演整体更淡。
export function beliefRGBA(b) {
  const n = b.width * b.height, out = new Uint8Array(n * 4), soft = b.phase === 'preview' ? 0.6 : 1;
  for (let i = 0; i < n; i++) {
    const v = b.values[i];
    if (v < 0) continue;
    const t = Math.sqrt(Math.min(100, v) / 100);
    let r, g, bl;
    if (t < 0.5) { const k = t / 0.5; r = 255; g = 238 - 78 * k; bl = 170 - 120 * k; }
    else { const k = (t - 0.5) / 0.5; r = 255 - 35 * k; g = 160 - 125 * k; bl = 50 - 15 * k; }
    out[i * 4] = Math.round(r); out[i * 4 + 1] = Math.round(g); out[i * 4 + 2] = Math.round(bl);
    out[i * 4 + 3] = Math.round((40 + 175 * t) * soft);
  }
  return out;
}

const RESULT_ZH = {found: '已找到', not_found: '已搜完，未找到', unable_to_confirm: '搜索结束，无法确认', cancelled: '搜索已停止', failed: '搜索中断'};
export function beliefCaption(b) {
  if (!b || b.phase === 'none') return '';
  if (b.phase === 'preview') return '若不在原处，将这样找';
  if (b.phase === 'search') return '搜索中 · 越红越可能';
  return RESULT_ZH[b.result] || '搜索结束';
}

// 收到 done 后多久完全隐藏（ms）；旧的 done（新页面接上时桥补发的）直接隐藏
export function doneHideDelay(b) { return b.ageMs > STALE_DONE_MS ? 0 : Math.max(0, DONE_HOLD_MS - b.ageMs); }
