// 联调试验台公共件：起/停 Python 子进程（按 pid）、等日志行、hub 夹具控制口、最小 WS 客户端（只带 Host/Origin/升级头）。
// 只在 run_tb.sh 起的隔离网络命名空间里用（GEOMEM_TB_NETNS=1）。
import { spawn } from 'node:child_process';
import fs from 'node:fs';
import WebSocket from 'ws';
export { launch, snap, wait, clearToast } from '../lib.mjs';
import { wait } from '../lib.mjs';

if (!process.env.GEOMEM_TB_NETNS) { console.error('拒绝运行：须经 ui/geomem/bridge/testbed/run_tb.sh 进隔离网络命名空间'); process.exit(2); }
export const WT = process.env.GEOMEM_WT;
// GEOMEM_PY：T3 在车的网络命名空间（容器里）跑时只有容器自己的 python3，两个都指向它。
export const PY313 = process.env.GEOMEM_PY || '${FPGA_HOME}/miniconda3/bin/python3.13';
export const PY38 = process.env.GEOMEM_PY || '/usr/bin/python3.8';
export const HUB_PORT = 47431, BRIDGE_PORT = 8788, CTL_PORT = 19291;
export const APP = `http://127.0.0.1:${BRIDGE_PORT}`;
export const REC_DIR = '${FPGA_SRC}/runs/integ-car-chain-v2-20261011-results/board/nano_formal_1314/runs';

const children = new Set();
export function startPy(py, args, log) {
  const out = fs.openSync(log, 'a');
  const proc = spawn(py, args, { cwd: WT, env: { ...process.env, PYTHONPATH: WT, PYTHONUNBUFFERED: '1' }, stdio: ['ignore', out, out] });
  children.add(proc); proc.on('exit', () => children.delete(proc));
  proc.log = log; proc.exited = new Promise(r => proc.on('exit', (code, sig) => r({ code, sig })));
  return proc;
}
export async function waitLog(proc, re, ms = 15000) {
  const t0 = Date.now();
  while (Date.now() - t0 < ms) { const txt = fs.readFileSync(proc.log, 'utf8'); const m = txt.match(re); if (m) return m; if (proc.exitCode !== null) break; await wait(100); }
  throw Error(`等不到日志 ${re} in ${proc.log}`);
}
export async function stopPid(proc, sig = 'SIGTERM') { if (proc.exitCode === null && proc.signalCode === null) { process.kill(proc.pid, sig); } return Promise.race([proc.exited, wait(8000).then(() => { try { process.kill(proc.pid, 'SIGKILL'); } catch {} return { killed: true }; })]); }
export async function stopAll() { for (const p of [...children]) await stopPid(p); }
process.on('exit', () => { for (const p of children) { try { process.kill(p.pid, 'SIGKILL'); } catch {} } });

export function tokenFile(dir) { const f = `${dir}/token.txt`; fs.writeFileSync(f, 'geomem-integ-testbed-token-0123456789\n', { mode: 0o600 }); return f; }
// GEOMEM_HUB（host:port）：T3 连真车 hub（地址取自本趟 car.json），T1/T2 不设、用命名空间里的 127.0.0.1:47431。
export const HUB_ADDR = process.env.GEOMEM_HUB || `127.0.0.1:${HUB_PORT}`;
export function startBridge(py, tok, dir, tag, extra = []) {
  return startPy(py, [`${WT}/ui/geomem/bridge/geomem_bridge.py`, '--hub', HUB_ADDR, '--token-file', tok, '--listen', `127.0.0.1:${BRIDGE_PORT}`, '--stats-out', `${dir}/bridge_stats_${tag}.json`, ...extra], `${dir}/bridge_${tag}.log`);
}
export async function waitHttp(url, ms = 15000) { const t0 = Date.now(); while (Date.now() - t0 < ms) { try { const r = await fetch(url); if (r.ok) return true; } catch {} await wait(150); } throw Error(`等不到 ${url}`); }
export const ctl = async q => { const r = await fetch(`http://127.0.0.1:${CTL_PORT}/ctl?${q}`); const j = await r.json(); if (j.error) throw Error(j.error); return j; };
export function startFixture(py, tok, dir, extra = []) {
  return startPy(py, [`${WT}/ui/geomem/bridge/testbed/t2_hub_fixture.py`, '--token-file', tok, '--port', String(HUB_PORT), '--ctl-port', String(CTL_PORT), ...extra], `${dir}/fixture.log`);
}
export const readJson = f => JSON.parse(fs.readFileSync(f, 'utf8'));

// 最小 WS 客户端：收所有文本帧（二进制帧只记长度与头），可发命令并等 ack/result。
export class RawClient {
  constructor(url = `ws://127.0.0.1:${BRIDGE_PORT}/ws`) { this.url = url; this.msgs = []; this.waiters = []; }
  open() { return new Promise((res, rej) => { const ws = new WebSocket(this.url, { headers: { Origin: `http://127.0.0.1:${BRIDGE_PORT}` } }); this.ws = ws;
    ws.on('message', (data, isBinary) => { let m; if (isBinary) { const b = Buffer.from(data); const L = b.readUInt32LE(0); m = { type: 'cloud', header: JSON.parse(b.subarray(4, 4 + L).toString('utf8')), bytes: b.length }; } else m = JSON.parse(data.toString('utf8')); m._t = Date.now(); this.msgs.push(m); for (const w of [...this.waiters]) if (w.pred(m)) { this.waiters.splice(this.waiters.indexOf(w), 1); w.res(m); } });
    ws.on('open', () => res(this)); ws.on('error', rej); }); }
  next(pred, ms = 8000) { const hit = this.msgs.find(pred); if (hit) return Promise.resolve(hit); return new Promise((res, rej) => { const w = { pred, res }; this.waiters.push(w); setTimeout(() => { const i = this.waiters.indexOf(w); if (i >= 0) { this.waiters.splice(i, 1); rej(Error('等不到消息')); } }, ms); }); }
  last(type) { for (let i = this.msgs.length - 1; i >= 0; i--) if (this.msgs[i].type === type) return this.msgs[i]; return null; }
  async command(action, payload, ms = 8000) {
    const id = `tb-${Date.now()}-${Math.random().toString(16).slice(2, 8)}`; const t0 = Date.now();
    this.ws.send(JSON.stringify({ type: 'command', version: 2, id, action, payload }));
    const ack = await this.next(m => m.type === 'ack' && m.id === id, ms);
    let result = null; if (ack.accepted || ack.reasonCode === 'no_ack') result = await this.next(m => m.type === 'result' && m.id === id, ms).catch(() => null);
    return { id, ack, result, ackMs: ack._t - t0, resultMs: result ? result._t - t0 : null };
  }
  close() { try { this.ws.close(); } catch {} }
}
// 等页面连接标签满足条件（SwiftShader 下首帧/着色器编译会卡主线程数秒，不能用固定等待）。
export async function waitPill(page, re = /^实车已连接$/, ms = 30000) {
  const t0 = Date.now(); let last = '';
  while (Date.now() - t0 < ms) { last = await page.locator('#connection-pill').innerText(); if (re.test(last)) return { ms: Date.now() - t0, pill: last }; await wait(200); }
  throw Error(`连接标签等不到 ${re}，最后是「${last}」`);
}
export function startReplay(py, rec, tok, dir, tag) { return startPy(py, [`${WT}/ui/geomem/bridge/testbed/t1_replay.py`, rec, tok, '--port', String(HUB_PORT)], `${dir}/replay_${tag}.log`); }
// 页内仪器（init script）：记下页面的 WebSocket 实例（供注入合成帧）、按出现顺序记提示文字、可选整体偏移 Date。
export const hooks = (arg = {}) => ({ fn: a => {
  if (a.dateOffset) { const D = Date; class FD extends D { constructor(...x) { if (x.length === 0) super(D.now() + a.dateOffset); else super(...x); } static now() { return D.now() + a.dateOffset; } } FD.parse = D.parse; FD.UTC = D.UTC; globalThis.Date = FD; }
  const W = window.WebSocket; window.__ws = [];
  window.WebSocket = class extends W { constructor(...x) { super(...x); window.__ws.push(this); } };
  // 每次 toast() 调用都整体替换 #toast 的文字节点（一条 childList 记录），同文字重复弹也能数到。
  window.__toasts = [];
  document.addEventListener('DOMContentLoaded', () => new MutationObserver(recs => { for (const r of recs) for (const n of r.addedNodes) if (n.textContent.trim()) window.__toasts.push({ t: performance.now(), text: n.textContent.trim() }); }).observe(document.getElementById('toast'), { childList: true }));
}, arg });
export const toasts = page => page.evaluate(() => window.__toasts.map(x => x.text));
export const toastMark = page => page.evaluate(() => window.__toasts.length);
export const toastsSince = (page, n) => page.evaluate(n => window.__toasts.slice(n).map(x => x.text), n);
// 在页面当前 WebSocket 上派发一条合成消息（字符串或 base64 二进制）。
export const inject = (page, data, binary = false) => page.evaluate(([d, b]) => { const ws = window.__ws.at(-1); const payload = b ? Uint8Array.from(atob(d), c => c.charCodeAt(0)).buffer : d; ws.dispatchEvent(new MessageEvent('message', { data: payload })); }, [data, binary]);
export const cloudInfo = page => page.evaluate(() => ({ points: +document.getElementById('map-canvas').dataset.cloudPoints, rev: document.getElementById('map-canvas').dataset.cloudRev }));
export const out = d => { fs.mkdirSync(d, { recursive: true }); return d; };
export function verdict(name, controlHolds, pass, extra = '') { const v = !controlHolds ? 'TESTBED-INVALID' : pass ? 'PASS' : 'FAIL'; console.log(`VERDICT ${name} ${v} ${extra}`); return v; }
