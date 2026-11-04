export const STATES = {idle:'待命', searching:'检索记忆', ambiguous:'请选择目标', planning:'规划路线', navigating:'正在前往', confirming:'现场确认', succeeded:'已确认找到', failed:'任务失败', paused:'已暂停', cancelled:'已停止'};
export const TERMINAL = new Set(['succeeded','failed','cancelled']);
export const activeTask = task => task && !TERMINAL.has(task.state) && task.state !== 'idle';
export function matchObjects(objects, query) {
  // 10-22：颜色不是匹配依据（名字里已没有颜色），查询里的颜色词先去掉，「灰色书包」照样找到书包。
  const q = query.trim().toLowerCase().replace(/^(帮我)?(寻找|查找|找一下|找)\s*/, '').replace(/[红蓝绿黄白黑灰紫橙棕粉]色的?/g, '').trim();
  if (!q) return [];
  return objects.filter(o => [o.name,o.className,...(o.aliases || [])].some(s => s.toLowerCase().includes(q)));
}
export function endpointURL(value, secure = false) {
  const u = new URL(value);
  if (!['ws:','wss:'].includes(u.protocol) || u.username || u.password || u.hash) throw Error('请输入不含账号密码或片段的 ws:// 或 wss:// 地址。');
  if (secure && u.protocol !== 'wss:') throw Error('HTTPS 页面必须使用 wss:// 安全连接。');
  return u.href;
}
const finite = n => typeof n === 'number' && Number.isFinite(n);
const vector = v => Array.isArray(v) && v.length === 3 && v.every(finite);
const str = (s, max = 200) => typeof s === 'string' && s.length <= max;
const date = d => typeof d === 'string' && Number.isFinite(Date.parse(d));
export function validateSnapshot(s) {
  if (!s || s.type !== 'snapshot' || s.version !== 1 || !Number.isSafeInteger(s.seq) || s.seq < 0 || !date(s.timestamp)) throw Error('无效的快照版本或时间戳');
  if (!s.map || !str(s.map.id) || !str(s.map.name) || s.map.frame !== 'map' || s.map.units !== 'm') throw Error('地图必须使用 map 坐标系与米单位');
  if (!Array.isArray(s.objects) || s.objects.length > 1000 || !s.objects.every(o => str(o.id) && o.id.length && str(o.name) && str(o.className) && str(o.location) && vector(o.position) && finite(o.confidence) && o.confidence >= 0 && o.confidence <= 1 && (o.lastSeen === null || date(o.lastSeen)) && Array.isArray(o.aliases) && o.aliases.every(a => str(a)))) throw Error('物体记忆格式错误');
  if (new Set(s.objects.map(o => o.id)).size !== s.objects.length) throw Error('物体实例 ID 重复');
  if (s.robot !== null && (!s.robot || !vector(s.robot.position) || !finite(s.robot.yaw) || !finite(s.robot.speed) || s.robot.speed < 0 || !str(s.robot.status) || typeof s.robot.localized !== 'boolean')) throw Error('小车位姿格式错误');
  if (!Array.isArray(s.path) || s.path.length > 5000 || !s.path.every(vector)) throw Error('路径格式错误');
  if (!Array.isArray(s.points) || s.points.length > 60000 || !s.points.every(p => Array.isArray(p) && p.length === 6 && p.every(finite) && p.slice(3).every(c => c >= 0 && c <= 1))) throw Error('点云格式错误或超过点数上限');
  if (s.task !== null && (!s.task || !str(s.task.id) || !str(s.task.targetId) || !Object.hasOwn(STATES,s.task.state) || !str(s.task.message,1000) || !date(s.task.updatedAt))) throw Error('任务格式错误');
  return s;
}
export function canStart(snapshot, connected, fresh) { return !!(connected && fresh && snapshot?.robot?.localized && !activeTask(snapshot.task)); }
export function snapshotFresh(snapshot, receivedAt, now = Date.now()) {
  return !!snapshot && now - receivedAt < 5000 && now - Date.parse(snapshot.timestamp) < 5000 && Date.parse(snapshot.timestamp) - now < 2000;
}
export const esc = s => String(s ?? '').replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
