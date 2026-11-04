// 浏览器 ⇄ 桥：协议第二版（docs/PROTOCOL_V2_DESIGN.md §2）。
// 事件：connection {state, attempt?, delayMs?}、hello {hello, caps}、snapshot、cloud {header, positions, colors}、result {id, action, status, reason, reasonCode}、error（中文字符串）。
import { PROTOCOL_VERSION, validateHello, normalizeCapabilities, validateSnapshot, decodeCloud, newId, reasonText, MAX_TEXT_MESSAGE, MAX_BINARY_MESSAGE } from './model.js';

const BACKOFF_MS = [1000, 2000, 4000, 5000];
const ACK_TIMEOUT_MS = 7000;      // 桥自己的未回执判定是截止＋0.5 s（find/select 3.5 s、cancel 5.5 s），本地兜底要晚于它（10-15 联调定）
const RESULT_TIMEOUT_MS = 9000;   // 桥给 find/select 3 s、cancel 5 s 截止，加 hub 2 s 观察窗与余量

export class LiveAdapter extends EventTarget {
  constructor(url) { super(); this.url = url; this.pending = new Map(); this.lastSeq = -1; this.ws = null; this.closed = false; this.attempt = 0; this.ready = false; this.caps = normalizeCapabilities({}); }
  emit(type, detail) { this.dispatchEvent(new CustomEvent(type, {detail})); }
  connect() { this.closed = false; this.open(); }
  open() {
    if (this.closed) return;
    this.ready = false; this.lastSeq = -1;                  // 新连接序号归零（§2.3）
    this.emit('connection', {state: this.attempt ? 'reconnecting' : 'connecting', attempt: this.attempt});
    let ws;
    try { ws = new WebSocket(this.url); } catch { this.emit('error', '桥接服务地址无效，无法建立连接。'); this.scheduleReconnect(); return; }
    ws.binaryType = 'arraybuffer';
    this.ws = ws;
    this.openTimer = setTimeout(() => { if (ws.readyState !== WebSocket.OPEN) { this.emit('error', '连接超时，请检查桥接服务是否在运行。'); try { ws.close(); } catch {} } }, 8000);
    ws.onopen = () => { clearTimeout(this.openTimer); this.helloTimer = setTimeout(() => { if (!this.ready && this.ws === ws) { this.emit('error', '已连上，但桥接服务没有发送握手信息，正在重连。'); try { ws.close(); } catch {} } }, 5000); };
    ws.onmessage = event => { if (this.ws === ws) this.receive(event.data); };
    ws.onerror = () => {};   // 失败原因统一在 onclose 里处理，避免重复提示
    ws.onclose = () => {
      clearTimeout(this.openTimer); clearTimeout(this.helloTimer);
      if (this.ws !== ws) return;
      this.ws = null; const wasReady = this.ready; this.ready = false;
      this.failPending('连接中断：结果未知，请核对小车状态；未自动重发。');
      if (this.closed) return;
      if (wasReady) this.attempt = 0;
      this.emit('connection', {state: 'disconnected'});
      this.scheduleReconnect();
    };
  }
  scheduleReconnect() {
    if (this.closed) return;
    const delayMs = BACKOFF_MS[Math.min(this.attempt, BACKOFF_MS.length - 1)];
    this.attempt++;
    this.emit('connection', {state: 'waiting', attempt: this.attempt, delayMs});
    clearTimeout(this.retryTimer);
    this.retryTimer = setTimeout(() => this.open(), delayMs);
  }
  receive(raw) {
    try {
      if (raw instanceof ArrayBuffer) {
        if (!this.ready) throw Error('握手前收到点云');
        if (raw.byteLength > MAX_BINARY_MESSAGE) throw Error('点云消息超过 16 MB 上限');
        this.emit('cloud', decodeCloud(raw));
        return;
      }
      if (typeof raw !== 'string') throw Error('消息类型无法识别');
      if (raw.length > MAX_TEXT_MESSAGE) throw Error('消息超过 4 MB 上限');
      let data;
      try { data = JSON.parse(raw); } catch { throw Error('收到无法解析的数据'); }
      if (!data || typeof data !== 'object') throw Error('消息格式错误');
      if (data.type === 'hello') {
        validateHello(data);
        this.caps = normalizeCapabilities(data.capabilities);
        const first = !this.ready; this.ready = true; this.attempt = 0; clearTimeout(this.helloTimer);
        this.emit('hello', {hello: data, caps: this.caps});
        if (first) this.emit('connection', {state: 'connected'});
      } else if (data.type === 'snapshot') {
        if (!this.ready) throw Error('握手前收到快照');
        validateSnapshot(data);
        if (data.seq <= this.lastSeq) return;   // 旧帧/重复帧丢弃
        this.lastSeq = data.seq; this.emit('snapshot', data);
      } else if (data.type === 'ack') {
        if (data.version !== PROTOCOL_VERSION || typeof data.id !== 'string' || typeof data.accepted !== 'boolean') throw Error('无效的指令回执');
        const p = this.pending.get(data.id); if (!p || p.acked) return;
        clearTimeout(p.timer);
        if (data.accepted) {
          p.acked = true; p.note = typeof data.note === 'string' ? data.note.slice(0, 200) : ''; p.resolveAck(data);
          if (p.action === 'teleop') { this.pending.delete(data.id); return; }
          const wait = p.action === 'mapping_stop' ? 190000 : p.action === 'map_reset' ? 40000 : ['estop','release'].includes(p.action) ? 35000 : RESULT_TIMEOUT_MS;
          p.timer = setTimeout(() => this.settle(data.id, {status: 'unknown', reason: '没有收到执行结果，结果未知，请核对小车状态；未自动重发。', reasonCode: 'timeout'}), wait);
        } else if (data.status === 'expired' || data.reasonCode === 'no_ack' || data.code === 'no_ack') {
          // 桥转发了但小车没回执：命令可能已到车上，不是「被拒」。只给一次「结果未知」（经 result 事件），
          // 桥随后补发的 result.unknown 因 pending 已删而被丢弃（10-15 联调定）。
          this.settle(data.id, {status: 'unknown', reason: '小车没有回应这条命令：结果未知（命令可能已到车上），请核对小车状态；未自动重发。', reasonCode: 'no_ack'});
        } else {
          this.pending.delete(data.id);
          const code = typeof data.code === 'string' ? data.code : typeof data.reasonCode === 'string' ? data.reasonCode : null;
          const note = typeof data.note === 'string' && data.note ? `（${data.note.slice(0, 200)}）` : '';
          const err = Error((reasonText(data.reason, code) || '请求被拒') + note); err.code = code; p.rejectAck(err);
        }
      } else if (data.type === 'result') {
        if (data.version !== PROTOCOL_VERSION || typeof data.id !== 'string' || !['accepted','rejected','done','unknown'].includes(data.status)) throw Error('无效的执行结果');
        this.settle(data.id, {status: data.status, reason: typeof data.reason === 'string' ? data.reason : '', note: typeof data.note === 'string' ? data.note.slice(0, 200) : '', reasonCode: typeof data.reasonCode === 'string' ? data.reasonCode : typeof data.code === 'string' ? data.code : null});
      }
      // 其余类型静默忽略（向前兼容）。
    } catch (e) { this.emit('error', `数据未被采用：${e.message}`); }
  }
  settle(id, result) {
    const p = this.pending.get(id); if (!p) return;
    clearTimeout(p.timer); this.pending.delete(id);
    if (!p.acked) { p.acked = true; p.resolveAck({accepted: true, status: 'forwarded'}); }
    this.emit('result', {id, action: p.action, ...result, note: [p.note, result.note].filter(Boolean).join('；')});
  }
  // 返回 {id, acked}：acked 在桥接受后兑现，拒绝/超时/断线时拒绝；执行结果走 'result' 事件。浏览器从不自动重发。
  command(action, payload = {}) {
    if (!this.ready || this.ws?.readyState !== WebSocket.OPEN) return {id: null, acked: Promise.reject(Error('连接已断开，请求未发送'))};
    const id = newId();
    let resolveAck, rejectAck;
    const acked = new Promise((res, rej) => { resolveAck = res; rejectAck = rej; });
    const p = {action, resolveAck, rejectAck, acked: false};
    p.timer = setTimeout(() => { this.pending.delete(id); rejectAck(Error('回执超时：执行结果未知，请核对小车状态；未自动重发。')); }, action === 'estop' ? 35000 : ACK_TIMEOUT_MS);
    this.pending.set(id, p);
    try { this.ws.send(JSON.stringify({type: 'command', version: PROTOCOL_VERSION, id, action, payload})); }
    catch { clearTimeout(p.timer); this.pending.delete(id); rejectAck(Error('请求发送失败，未自动重发。')); }
    return {id, acked};
  }
  failPending(message) {
    for (const [id, p] of this.pending) {
      clearTimeout(p.timer);
      if (!p.acked) p.rejectAck(Error(message));
      else this.emit('result', {id, action: p.action, status: 'unknown', reason: message, reasonCode: 'link_down'});
    }
    this.pending.clear();
  }
  close() {
    this.closed = true; clearTimeout(this.retryTimer); clearTimeout(this.openTimer); clearTimeout(this.helloTimer);
    this.failPending('连接已关闭：结果未知，请核对小车状态；未自动重发。');
    const ws = this.ws; this.ws = null; this.ready = false;
    if (ws) { ws.onclose = null; try { ws.close(); } catch {} }
    this.emit('connection', {state: 'disconnected'});
  }
}
