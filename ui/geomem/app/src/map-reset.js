// 10-26「重置地图」（owner 定；与 operator/src/map-reset.js 同一份，改动两边同步）：不停栈清空三维建图和物体记忆。协议见 ui/geomem/vehicle_interface_protocol.md「重置地图」。
// 纯模块（无 DOM、无 three.js），node --test 可测；remote.js 只接线：按钮 → 二次确认 → command('map_reset', {confirm:true})
// → 进度 → result done 时清空页面上的三维/点云/轨迹/记忆卡片/热力图。
export const MAP_RESET_ACTION = 'map_reset';
export const MAP_RESET_CONFIRM = '将清空地图和记忆，车会停下';
export const MAP_RESET_WAIT_MS = 40000;     // 桥 35 s（车上 runner 等 mapping_ctl 30 s）之后再给一点余量
export const MAP_RESET_DONE = '已重置，可以重新建图';
export const MAP_RESET_LOCALIZATION = '定位模式（使用已存地图）下不能重置地图；要重新建图请用建图模式起栈';

// 可用性：只看车栈 snapshot.mapping（hub 有建图状态才有；定位模式没有）与能力表。返回 '' 表示可用，否则中文原因。
// 连接/控制权/急停等通用门由 remote.js 的 blocker() 先判；这里只管建图状态。
export function mapResetBlocker(snapshot, caps = {}) {
  if (!caps.mapReset) return '车栈未提供此功能';
  const m = snapshot?.mapping;
  if (!m || typeof m.state !== 'string' || !m.state) return MAP_RESET_LOCALIZATION;
  if (m.state === 'saving' || (m.state === 'exported' && m.map_loaded == null)) return '地图正在保存，请等保存完成后再重置';
  if (m.state === 'resetting') return '地图正在重置';
  if (!['mapping', 'exported', 'failed'].includes(m.state)) return `建图状态 ${m.state}，暂不能重置`;
  return '';
}

// 结果 → 页面文案与是否清空页面（done 才清；rejected/unknown 不清，以车上为准）
export function mapResetOutcome(result) {
  const status = result?.status, reason = typeof result?.reason === 'string' ? result.reason : '';
  if (status === 'done') return {ok: true, clear: true, text: MAP_RESET_DONE};
  if (status === 'rejected') return {ok: false, clear: false, text: '重置地图没有完成' + (reason ? '：' + reason : '')};
  return {ok: false, clear: false, text: '重置地图结果未知，请核对小车状态' + (reason ? '：' + reason : '')};
}

// 小控制器：一次只跑一个重置；phase idle | confirm | pending | done | failed。
// send(action, payload) → Promise<id>（remote.js 的 request：拒绝时抛错）；clear() 清空页面；onChange(state) 刷新界面。
export class MapResetController {
  constructor({send, clear = () => {}, onChange = () => {}, now = () => Date.now()} = {}) {
    this.send = send; this.clear = clear; this.onChange = onChange; this.now = now;
    this.phase = 'idle'; this.id = null; this.text = ''; this.startedAt = 0;
  }
  get pending() { return this.phase === 'pending'; }
  _set(phase, text) { this.phase = phase; this.text = text; this.onChange(this); }
  // 按钮：blocker 非空则只给原因，不弹确认
  ask(blocker = '') {
    if (this.pending) return false;
    if (blocker) { this._set('failed', blocker); return false; }
    this._set('confirm', MAP_RESET_CONFIRM); return true;
  }
  cancel() { if (this.phase === 'confirm') this._set('idle', ''); }
  // 确认框里点「重置」
  async confirm(blocker = '') {
    if (this.pending) return null;
    if (blocker) { this._set('failed', blocker); return null; }
    this.id = null; this.startedAt = this.now();
    this._set('pending', '正在重置：停车、清空记忆和地图…');
    try {
      const id = await this.send(MAP_RESET_ACTION, {confirm: true});
      if (this.phase === 'pending') this.id = id;
      if (this.phase === 'pending' && this._early) { const r = this._early; this._early = null; if (r.id === id) this.onResult(r); }
      return id;
    } catch (e) {
      if (this.phase === 'pending') this._set('failed', '重置地图请求没有被接受：' + (e?.message || e));
      return null;
    }
  }
  // transport 的 result 事件（只认 action map_reset；id 未回来前先暂存）
  onResult(d) {
    if (!d || d.action !== MAP_RESET_ACTION || !this.pending) return false;
    if (this.id === null) { this._early = d; return false; }
    if (d.id !== this.id) return false;
    const o = mapResetOutcome(d);
    if (o.clear) this.clear();
    this._set(o.ok ? 'done' : 'failed', o.text);
    return true;
  }
  // 断线：结果未知
  onDisconnect() { if (this.pending) this._set('failed', '连接断开，重置地图结果未知，请核对小车状态'); }
  buttonLabel() { return this.pending ? '正在重置…' : '重置地图'; }
}
