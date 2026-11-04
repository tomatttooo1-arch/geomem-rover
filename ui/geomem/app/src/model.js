// GeoMem 协议第二版（docs/PROTOCOL_V2_DESIGN.md §2）的前端数据模型与校验。
export const PROTOCOL_VERSION = 2;
export const STATES = {idle:'待命', ambiguous:'请选择目标', searching:'正在搜索', planning:'规划路线', navigating:'正在前往', confirming:'现场确认', succeeded:'已确认找到', failed:'任务失败', paused:'已暂停', cancelled:'已停止'};
export const TERMINAL = new Set(['succeeded','failed','cancelled']);
export const activeTask = task => !!task && !TERMINAL.has(task.state) && task.state !== 'idle';
// 终态细分（10-16，docs/design/SELECT_SEMANTICS_20261016.md §2）：协议 state 仍是 failed（旧页面不认识的 state 会整帧拒收），
// 桥另给可选 outcome；只用于显示，校验、TERMINAL、activeTask 仍按协议 state。缺省、表外值一律当没有，从不拒帧。
export const OUTCOMES = {unable_to_confirm:'无法确认', needs_manual_approach:'需人工前往'};
export const taskDisplayState = t => !t ? 'idle' : (t.state === 'failed' && typeof t.outcome === 'string' && Object.hasOwn(OUTCOMES, t.outcome)) ? t.outcome : t.state;
export const stateLabel = s => Object.hasOwn(STATES, s) ? STATES[s] : Object.hasOwn(OUTCOMES, s) ? OUTCOMES[s] : '未知';
// needs_manual_approach 的记忆位置（米）；格式不对当没有。
export const manualTargetPosition = t => (taskDisplayState(t) === 'needs_manual_approach' && Array.isArray(t.targetPosition) && t.targetPosition.length === 3 && t.targetPosition.every(n => typeof n === 'number' && Number.isFinite(n))) ? t.targetPosition : null;
export const SOURCES = new Set(['live','replay','sim','inconsistent','unknown']);
// 回放细分（§2.1 sourceDetail，10-15 联调第 2 棒补）：可选字段，宽松读取——缺省、null、非字符串、表外值一律当 null，从不整帧拒收。
export const SOURCE_DETAILS = new Set(['recording','synthetic']);
export const sourceDetailOf = m => (m && typeof m === 'object' && SOURCE_DETAILS.has(m.sourceDetail)) ? m.sourceDetail : null;
// 来源横幅 [文字, 级别]；live 不显示横幅（null）。只有 recording 才写「实车真实录制」。
export function sourceBanner(source, detail) {
  if (source === 'replay') return detail === 'recording' ? ['回放：实车真实录制，非实时','info']
    : detail === 'synthetic' ? ['回放：合成测试数据（不是实车录制），非实时','info'] : ['回放数据，非实时','info'];
  return ({sim:['仿真数据：不是实车画面','info'],inconsistent:['数据来源不一致（实时与回放/仿真混杂），已禁止发起任务','error'],unknown:['数据来源未标注，请核对桥接服务','warn']})[source] || null;
}
// 设备详情「数据类别」一栏。
export function sourceZh(source, detail) {
  if (source === 'replay') return detail === 'recording' ? '回放（实车真实录制）' : detail === 'synthetic' ? '回放（合成测试数据）' : '回放（来源未细分）';
  return ({live:'实车实时',sim:'仿真',inconsistent:'来源不一致',unknown:'来源未标注'})[source] || '来源未标注';
}
export const POSES = new Set(['map','odometry','none']);
export const MAX_OBJECTS = 1000, MAX_PATH = 5000, MAX_CLOUD_POINTS = 60000;
export const MAX_TEXT_MESSAGE = 4_000_000, MAX_BINARY_MESSAGE = 16 * 1024 * 1024, MAX_CLOUD_HEADER = 65536;
export const FRESH_MS = 2000;

// COCO 英文类名 → 中文（与设计 §3.5 一致；表外类名原样显示英文）。
// COCO 80 类全表，名字与 perception/fpga_replay/coco_zh.py CLASS_ZH 主名一致（D-044，2026-10-21 补齐原缺的 17 类）。
export const CLASS_ZH = {person:'人',bicycle:'自行车',car:'汽车',motorcycle:'摩托车',airplane:'飞机',bus:'公交车',train:'火车',truck:'卡车',boat:'船','traffic light':'红绿灯','fire hydrant':'消防栓',
  'stop sign':'停车标志','parking meter':'停车计时器',bench:'长椅',bird:'鸟',cat:'猫',dog:'狗',horse:'马',sheep:'羊',cow:'牛',elephant:'大象',bear:'熊',zebra:'斑马',
  giraffe:'长颈鹿',backpack:'书包',umbrella:'雨伞',handbag:'手提包',tie:'领带',suitcase:'行李箱',frisbee:'飞盘',skis:'滑雪板',snowboard:'单板滑雪板','sports ball':'球',kite:'风筝',
  'baseball bat':'球棒','baseball glove':'棒球手套',skateboard:'滑板',surfboard:'冲浪板','tennis racket':'网球拍',bottle:'瓶子','wine glass':'酒杯',cup:'杯子',fork:'叉子',
  knife:'刀',spoon:'勺子',bowl:'碗',banana:'香蕉',apple:'苹果',sandwich:'三明治',orange:'橙子',broccoli:'西兰花',carrot:'胡萝卜','hot dog':'热狗',pizza:'披萨',donut:'甜甜圈',
  cake:'蛋糕',chair:'椅子',couch:'沙发','potted plant':'盆栽',bed:'床','dining table':'桌子',toilet:'马桶',tv:'电视',laptop:'笔记本电脑',mouse:'鼠标',remote:'遥控器',
  keyboard:'键盘','cell phone':'手机',microwave:'微波炉',oven:'烤箱',toaster:'烤面包机',sink:'水槽',refrigerator:'冰箱',book:'书',clock:'时钟',vase:'花瓶',scissors:'剪刀',
  'teddy bear':'玩具熊','hair drier':'吹风机',toothbrush:'牙刷',
  'medicine bottle':'药瓶','medicine box':'药盒',key:'钥匙',glasses:'眼镜',
  'waste paper':'废纸','drink can':'饮料罐','plastic packaging':'塑料包装'};
// 10-22（owner）：物体只显示类名，不显示颜色（车上颜色估计常错，同一个书包会被说成灰色/黑色）；color 字段仍随数据收下，不进任何文案。
export const OBJECT_STATE_ZH = {current_confirmed:'当前确认',historical_memory:'历史记忆',temporarily_occluded:'暂时被遮挡',position_invalid:'上次位置已确认不在'};
export const POSE_ZH = {map:'已定位',odometry:'仅里程计定位 · 里程推算',none:'未定位'};
// 里程计档（odom 坐标系，或仿真档没有地图位姿时桥退到里程计位姿）允许寻物并提示（10-15 联调第 3 棒，owner 默认）。
export const ODOM_BADGE = '仅里程计定位 · 里程推算';
export const ODOM_FIND_HINT = '仅里程计定位（里程推算，未经地图定位），位置可能有偏差；仍可发起寻物。';
export const classZh = cls => CLASS_ZH[cls] || cls || '未知物品';
// name 可空：回落到类名中文，再回落到英文类名（§2.3；10-22 起不带颜色）。
export const displayName = o => (o && typeof o.name === 'string' && o.name) ? o.name : classZh(o?.className);

// 命令拒绝与结果原因（桥/hub/runner 原词）→ 中文；桥已给中文 reason 时优先用桥的。
export const REASON_ZH = {
  not_controller:'另一台电脑持有控制权，本界面只读', estop_latched:'急停锁存中，请先在车上解除急停', bad_args:'请求格式不对', duplicate:'重复的请求，已忽略',
  scope_changed:'记忆已重置，请重新选择', no_such_candidate:'所选物品已不在候选中，请重新选择', not_supported:'小车不支持这个操作', not_ambiguous:'当前不在选择目标阶段',
  task_mismatch:'任务已变化，请按当前任务重新操作', stale_task:'任务已变化（不是当前任务），请按当前任务重新操作', no_ack:'车上没有回执，结果未知，请核对小车状态',
  source_inconsistent:'车上数据来源不一致，暂不接受找物或选择', bad_request:'请求格式不对', bad_op:'车上不认识这个命令', not_enabled:'车上没有启用这个功能', replay:'回放模式下不能操作小车',
  stale_scope:'记忆已重置，请重新选择', scope_mismatch:'记忆已重置，请重新选择', query_rejected:'小车拒绝了这次查找', selection_rejected:'小车拒绝了这次选择', link_down:'桥接服务与小车的连接已断开，请求未发送',
  candidate_selection_required:'记忆里有多个符合的物品，请选择要找哪一个', search_candidate_selection_required:'搜索中看到了新的候选，请确认是不是要找的那个',
  target_position_invalid:'原位置已空（物品被移走），正在按搜索路线寻找', not_seen_at_last_location:'原位置没看到物品，正在按搜索路线寻找', no_observation_pose:'原处附近无处可看，开始按搜索路线寻找', search_single_candidate:'搜索中看到唯一一个同类物品，判定为目标',
  search_exhausted:'搜索路线走完仍未找到', observation_positions_exhausted:'几个观察位置都没能确认目标', visit_budget_exhausted:'几个观察位置都没能确认目标',
  navigation_failed:'导航没能到达观察点', no_observation_pose_planned:'地图上没有能看到目标的停车位置（目标可能在地图范围外），没法开过去确认',
  navigation_server_unavailable:'车上没开导航，不会自己开过去；需要人工开到目标附近', navigation_busy_previous_goal_active:'上一个导航目标还没结束，这次导航没有发出',
  navigation_input_unhealthy:'画面或定位不够新，这次导航没有发出', navigation_timeout:'导航超时', task_timeout:'任务超过时间上限', stream_lost:'相机画面中断', no_recent_frame:'没有最新画面',
  pose_invalid:'小车位置不可靠，已停止', localization_invalid:'小车位置不可靠，已停止', user_cancelled:'已按要求停止', timeout:'等待回执超时'};
// 已知码用本地中文表；未知码显示通用「请求被拒」＋桥给的原因文本（或原码），不编造。
export function reasonText(reason, code) {
  const r = typeof reason === 'string' ? reason.trim().slice(0, 300) : '';
  if (code && REASON_ZH[code]) return r && /[\u4e00-\u9fff]/.test(r) ? r : REASON_ZH[code];
  if (r) return /[\u4e00-\u9fff]/.test(r) ? r : `请求被拒：${r}`;
  return code ? `请求被拒（${String(code).slice(0, 60)}）` : '';
}

export function matchObjects(objects, query) {
  // 10-22：颜色不是匹配依据（名字里已没有颜色），查询里的颜色词先去掉，「灰色书包」照样找到书包。
  const q = query.trim().toLowerCase().replace(/^(帮我)?(寻找|查找|找一下|找)\s*/, '').replace(/[红蓝绿黄白黑灰紫橙棕粉]色的?/g, '').trim();
  if (!q) return [];
  return objects.filter(o => [o.name, o.className, classZh(o.className), displayName(o), ...(o.aliases || [])].some(s => typeof s === 'string' && s.toLowerCase().includes(q)));
}
// 记忆里出现次数最多的前 n 个类名（替代写死的建议按钮）。
export function topClasses(objects, n = 3) {
  const count = new Map();
  for (const o of objects || []) count.set(o.className, (count.get(o.className) || 0) + 1);
  return [...count.entries()].sort((a, b) => b[1] - a[1] || String(a[0]).localeCompare(String(b[0]))).slice(0, n).map(([c]) => c);
}
export function endpointURL(value, secure = false) {
  let u;
  try { u = new URL(value); } catch { throw Error('地址格式不对，请输入 ws:// 或 wss:// 开头的完整地址。'); }
  if (!['ws:','wss:'].includes(u.protocol) || u.username || u.password || u.hash) throw Error('请输入不含账号密码或片段的 ws:// 或 wss:// 地址。');
  if (secure && u.protocol !== 'wss:') throw Error('HTTPS 页面必须使用 wss:// 安全连接。');
  return u.href;
}
export function defaultEndpoint(loc = location) {
  return `${loc.protocol === 'https:' ? 'wss:' : 'ws:'}//${loc.host}/ws`;
}

const finite = n => typeof n === 'number' && Number.isFinite(n);
const nonneg = n => finite(n) && n >= 0;
const vector = v => Array.isArray(v) && v.length === 3 && v.every(finite);
const str = (s, max = 200) => typeof s === 'string' && s.length <= max;
const nstr = (s, max = 200) => s === null || str(s, max);
const opt = (o, k, check) => !Object.hasOwn(o, k) || o[k] === null || check(o[k]);  // 规格允许为空的字段：缺省或 null
const isObj = o => !!o && typeof o === 'object' && !Array.isArray(o);
const fail = m => { throw Error(m); };

export function validateHello(h) {
  if (!isObj(h) || h.type !== 'hello') fail('握手消息格式错误');
  if (h.version !== PROTOCOL_VERSION) fail(`桥接服务协议版本不符（收到 ${String(h.version).slice(0, 10)}，需要 ${PROTOCOL_VERSION}）`);
  if (!isObj(h.bridge) || !str(h.bridge.id) || !nstr(h.bridge.hubBootId ?? null) || !['up','stale','down'].includes(h.bridge.link)) fail('握手消息里的桥接状态格式错误');
  if (!SOURCES.has(h.source)) fail('握手消息里的数据来源无法识别');
  if (!isObj(h.capabilities)) fail('握手消息缺少能力表');
  return h;
}
// 不认识的能力一律当 false（§2.1）。
export function normalizeCapabilities(c = {}) {
  const yes = v => v === true;
  return {findText: yes(c.findText), findById: c.findById === true ? true : c.findById === 'emulated' ? 'emulated' : false, select: yes(c.select), cancel: yes(c.cancel),
    pause: yes(c.pause), resume: yes(c.resume), estop: yes(c.estop), teleop: yes(c.teleop), mappingStop: yes(c.mappingStop), mapReset: yes(c.mapReset), goto: yes(c.goto), release: yes(c.release), control: ['controller','observer','none'].includes(c.control) ? c.control : 'none'};
}

function validObject(o) {
  return isObj(o) && str(o.id) && o.id.length > 0 && opt(o, 'name', s => str(s)) && str(o.className) && o.className.length > 0
    && opt(o, 'color', s => str(s, 40)) && opt(o, 'aliases', a => Array.isArray(a) && a.length <= 50 && a.every(s => str(s)))
    && opt(o, 'location', s => str(s)) && vector(o.position) && opt(o, 'confidence', n => finite(n) && n >= 0 && n <= 1)
    && opt(o, 'lastSeen', finite) && opt(o, 'state', s => str(s, 64));
}
function validCandidate(c) {
  return isObj(c) && str(c.id) && c.id.length > 0 && opt(c, 'name', s => str(s)) && str(c.className) && opt(c, 'color', s => str(s, 40))
    && vector(c.position) && opt(c, 'state', s => str(s, 64));
}
export function validateSnapshot(s) {
  if (!isObj(s) || s.type !== 'snapshot') fail('快照格式错误');
  if (s.version !== PROTOCOL_VERSION) fail(`快照协议版本不符（需要 ${PROTOCOL_VERSION}）`);
  if (!Number.isSafeInteger(s.seq) || s.seq < 0) fail('快照序号无效');
  if (!(s.serverTime === null || finite(s.serverTime))) fail('快照时间轴数值无效');
  if (!SOURCES.has(s.source)) fail('快照数据来源无法识别');
  if (!isObj(s.link) || !['up','stale','down'].includes(s.link.state) || !opt(s.link, 'hubAgeMs', nonneg)) fail('链路状态格式错误');
  const m = s.map;
  if (!isObj(m) || !opt(m, 'id', x => str(x)) || !opt(m, 'name', x => str(x)) || !['map','odom'].includes(m.frame) || m.units !== 'm'
    || !opt(m, 'cloudRev', x => str(x, 100)) || typeof m.placeholder !== 'boolean') fail('地图信息格式错误（坐标系须为 map 或 odom，单位为米）');
  if (s.robot !== null && (!isObj(s.robot) || !vector(s.robot.position) || !finite(s.robot.yaw) || !nonneg(s.robot.speed) || !POSES.has(s.robot.pose)
    || !opt(s.robot, 'poseAgeMs', nonneg) || !str(s.robot.status, 100))) fail('小车位姿格式错误');
  if (s.memory !== null && (!isObj(s.memory) || !opt(s.memory, 'scope', x => str(x)) || !opt(s.memory, 'revision', n => Number.isSafeInteger(n) && n >= 0) || !opt(s.memory, 'ageMs', nonneg))) fail('记忆信息格式错误');
  if (!Array.isArray(s.objects) || s.objects.length > MAX_OBJECTS) fail(`物体记忆格式错误或超过 ${MAX_OBJECTS} 条上限`);
  if (!s.objects.every(validObject)) fail('物体记忆格式错误');
  if (new Set(s.objects.map(o => o.id)).size !== s.objects.length) fail('物体实例 ID 重复');
  if (!Array.isArray(s.path) || s.path.length > MAX_PATH || !s.path.every(vector)) fail('路径格式错误');
  if (Object.hasOwn(s, 'points')) fail('快照不应再带点云（第二版点云单独发送）');
  const t = s.task;
  if (t !== null) {
    if (!isObj(t) || !Object.hasOwn(STATES, t.state) || !nstr(t.id ?? null) || !nstr(t.targetId ?? null) || !nstr(t.reasonCode ?? null, 200)
      || !str(t.message, 1000) || !opt(t, 'ageMs', nonneg) || !Array.isArray(t.candidates) || t.candidates.length > MAX_OBJECTS || !t.candidates.every(validCandidate)) fail('任务格式错误');
    if (new Set(t.candidates.map(c => c.id)).size !== t.candidates.length) fail('候选物品 ID 重复');
  }
  return s;
}

// 二进制点云帧：u32 头长 L | L 字节 UTF-8 JSON 头 | 补 0 到 4 字节对齐 | N×3 float32（小端）| N×3 uint8（§2.4）。
export function decodeCloud(buffer) {
  if (!(buffer instanceof ArrayBuffer)) fail('点云数据类型错误');
  if (buffer.byteLength < 4 || buffer.byteLength > MAX_BINARY_MESSAGE) fail('点云数据长度不对');
  const dv = new DataView(buffer), L = dv.getUint32(0, true);
  if (L < 2 || L > MAX_CLOUD_HEADER || 4 + L > buffer.byteLength) fail('点云头长度不对');
  let h;
  try { h = JSON.parse(new TextDecoder('utf-8', {fatal: true}).decode(new Uint8Array(buffer, 4, L))); } catch { fail('点云头无法解析'); }
  if (!isObj(h) || h.type !== 'cloud' || h.version !== PROTOCOL_VERSION) fail('点云头类型或版本不符');
  if (!Number.isSafeInteger(h.count) || h.count < 0 || h.count > MAX_CLOUD_POINTS) fail(`点云点数无效或超过 ${MAX_CLOUD_POINTS} 点上限`);
  if (h.encoding !== 'f32xyz+u8rgb' || !['map','odom'].includes(h.frame) || !nstr(h.rev ?? null, 100) || (h.rev == null && h.count !== 0) || !nstr(h.mapId ?? null) || !['grid','export'].includes(h.source)) fail('点云头字段无效');
  const off = Math.ceil((4 + L) / 4) * 4, n = h.count;
  if (buffer.byteLength !== off + 15 * n) fail('点云数据长度与头不符');
  for (let i = 4 + L; i < off; i++) if (dv.getUint8(i) !== 0) fail('点云对齐填充不为 0');
  const positions = new Float32Array(buffer, off, 3 * n);   // 平台为小端（浏览器全部如此）；不拷贝
  for (let i = 0; i < positions.length; i++) if (!Number.isFinite(positions[i])) fail('点云坐标含非有限数');
  const colors = new Uint8Array(buffer, off + 12 * n, 3 * n);
  return {header: h, positions, colors};
}

// 新鲜度：只看本机收帧时刻（单调钟）与桥报的链路状态，与任何墙钟无关（§2.2）。
export function snapshotFresh(snapshot, receivedAt, now = performance.now()) {
  return !!snapshot && receivedAt > 0 && now - receivedAt < FRESH_MS && snapshot.link?.state === 'up';
}
// 「多久前见到」= serverTime − lastSeen，两者都在 runner 的时间轴上；任一缺失就是未知。
export const SEEN_PLAUSIBLE_MAX_S = 7 * 86400;
export function secondsSince(snapshot, lastSeen) {
  if (!snapshot || !finite(snapshot.serverTime) || !finite(lastSeen)) return null;
  // 两数不同轴时（10-11 旧录制 last_seen 是 0、1…而 serverTime 是墙钟；或运行中对时跳变）给「见到时间未知」，
  // 不显示「20737 天前」或把未来时刻压成「0 秒前」（10-15 联调定）。
  const age = snapshot.serverTime - lastSeen;
  if (age < -2 || age > SEEN_PLAUSIBLE_MAX_S) return null;
  return Math.max(0, age);
}
export function relativeText(seconds) {
  if (seconds === null) return '见到时间未知';
  const s = Math.floor(seconds);
  return s < 60 ? `${s} 秒前见到` : s < 3600 ? `${Math.floor(s / 60)} 分钟前见到` : s < 86400 ? `${Math.floor(s / 3600)} 小时前见到` : `${Math.floor(s / 86400)} 天前见到`;
}
// 发起寻物的前提；返回空串＝可以，否则返回中文原因。
export function startBlocker({snapshot, connected, fresh, caps}) {
  if (!connected) return '连接尚未就绪，请先连接实车或切回演示。';
  if (!snapshot) return '正在等待有效的地图和位姿数据。';
  if (!fresh) return '数据已过期，请检查连接或等待数据恢复。';
  if (snapshot.source === 'inconsistent') return '数据来源不一致，已禁止发送命令。';
  if (snapshot.source === 'replay') return '正在显示回放数据，不能向小车发送命令。';
  if (!caps?.findText) return '当前连接不支持发起寻物。';
  if (caps.control !== 'controller') return '另一台电脑持有控制权，本界面只读。';
  const pose = snapshot.robot?.pose;
  if (!snapshot.robot && snapshot.map.frame !== 'odom') return '没有收到小车位姿，暂时不能发起寻物。';
  if (pose === 'none') return '小车尚没有完成定位，暂时不能发起寻物。';
  if (activeTask(snapshot.task)) return '已有任务正在执行，请先完成或停止当前任务。';
  return '';
}
export const odomTier = snapshot => !!snapshot && (snapshot.map.frame === 'odom' || snapshot.robot?.pose === 'odometry');

// 非安全上下文（局域网 http）下没有 crypto.randomUUID，用 getRandomValues 拼 v4 UUID（§4 N6）。
export function newId() {
  if (globalThis.crypto?.randomUUID) { try { return crypto.randomUUID(); } catch {} }
  const b = crypto.getRandomValues(new Uint8Array(16));
  b[6] = (b[6] & 0x0f) | 0x40; b[8] = (b[8] & 0x3f) | 0x80;
  const h = [...b].map(x => x.toString(16).padStart(2, '0')).join('');
  return `${h.slice(0, 8)}-${h.slice(8, 12)}-${h.slice(12, 16)}-${h.slice(16, 20)}-${h.slice(20)}`;
}
export const esc = s => String(s ?? '').replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
