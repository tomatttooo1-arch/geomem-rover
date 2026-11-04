// 展台页（10-17）试验台公共件：与 tb.mjs 同一套起停/控制口约定，但浏览器用 puppeteer-core（宿主没有 playwright-core，
// tb.mjs 顶层 import lib.mjs → playwright，故不 import 它）。只在 run_tb.sh 起的隔离网络命名空间里用（GEOMEM_TB_NETNS=1）。
import { spawn } from 'node:child_process';
import fs from 'node:fs';
import puppeteer from 'puppeteer-core';

if (!process.env.GEOMEM_TB_NETNS) { console.error('拒绝运行：须经 ui/geomem/bridge/testbed/run_tb.sh 进隔离网络命名空间'); process.exit(2); }
export const WT = process.env.GEOMEM_WT;
export const PY313 = process.env.GEOMEM_PY || '${FPGA_HOME}/miniconda3/bin/python3.13';
export const PY38 = process.env.GEOMEM_PY || '/usr/bin/python3.8';
export const HUB_PORT = 47431, BRIDGE_PORT = 8788, CTL_PORT = 19291;
export const APP = `http://127.0.0.1:${BRIDGE_PORT}`;
export const CHROME = process.env.GEOMEM_CHROME || '${FPGA_HOME}/chrome-headless-shell/linux-147.0.7727.50/chrome-headless-shell-linux64/chrome-headless-shell';
export const REC_DIR = '${FPGA_SRC}/runs/integ-car-chain-v2-20261011-results/board/nano_formal_1314/runs';
// 样例库（map3d-look-20261017/src/20261015_190638.db，控制器 10-17 给全值）与其底图包、记忆、建图轨迹
export const SAMPLE_SHA = 'fae1c3bbe5cac6352ca8fdeaccfddee5439d5831ee858899d59b4a90d52575e6';
export const LOOK = '${FPGA_SRC}/runs/map3d-look-20261017';
export const wait = ms => new Promise(r => setTimeout(r, ms));

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
  while (Date.now() - t0 < ms) { const m = fs.readFileSync(proc.log, 'utf8').match(re); if (m) return m; if (proc.exitCode !== null) break; await wait(100); }
  throw Error(`等不到日志 ${re} in ${proc.log}`);
}
export async function stopPid(proc, sig = 'SIGTERM') { if (proc.exitCode === null && proc.signalCode === null) process.kill(proc.pid, sig); return Promise.race([proc.exited, wait(8000).then(() => { try { process.kill(proc.pid, 'SIGKILL'); } catch {} return { killed: true }; })]); }
export async function stopAll() { for (const p of [...children]) await stopPid(p); }
process.on('exit', () => { for (const p of children) { try { process.kill(p.pid, 'SIGKILL'); } catch {} } });
export function tokenFile(dir) { const f = `${dir}/token.txt`; fs.writeFileSync(f, 'geomem-integ-testbed-token-0123456789\n', { mode: 0o600 }); return f; }
export function startBridge(py, tok, dir, tag, extra = []) {
  return startPy(py, [`${WT}/ui/geomem/bridge/geomem_bridge.py`, '--hub', `127.0.0.1:${HUB_PORT}`, '--token-file', tok, '--listen', `127.0.0.1:${BRIDGE_PORT}`, '--stats-out', `${dir}/bridge_stats_${tag}.json`, ...extra], `${dir}/bridge_${tag}.log`);
}
export function startFixture(py, tok, dir, extra = []) {
  return startPy(py, [`${WT}/ui/geomem/bridge/testbed/t2_hub_fixture.py`, '--token-file', tok, '--port', String(HUB_PORT), '--ctl-port', String(CTL_PORT), ...extra], `${dir}/fixture.log`);
}
export function startReplay(py, rec, tok, dir, tag) { return startPy(py, [`${WT}/ui/geomem/bridge/testbed/t1_replay.py`, rec, tok, '--port', String(HUB_PORT)], `${dir}/replay_${tag}.log`); }
export async function waitHttp(url, ms = 20000) { const t0 = Date.now(); while (Date.now() - t0 < ms) { try { const r = await fetch(url); if (r.ok) return true; } catch {} await wait(150); } throw Error(`等不到 ${url}`); }
export const ctl = async q => { const r = await fetch(`http://127.0.0.1:${CTL_PORT}/ctl?${q}`); const j = await r.json(); if (j.error) throw Error(j.error); return j; };
export const out = d => { fs.mkdirSync(d, { recursive: true }); return d; };

export async function launch({ gpu = false } = {}) {
  const args = ['--no-sandbox', ...(gpu ? ['--use-angle=vulkan', '--enable-features=Vulkan', '--ignore-gpu-blocklist'] : ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'])];
  const browser = await puppeteer.launch({ executablePath: CHROME, headless: 'shell', args, protocolTimeout: 600000 });
  const page = await browser.newPage(); await page.setViewport({ width: 1920, height: 1080 });
  page.errors = []; page.consoleErrors = [];
  page.on('pageerror', e => { page.errors.push(e.message); console.log('pageerror:', e.message); });
  page.on('console', m => { if (m.type() === 'error') { page.consoleErrors.push(m.text()); console.log('console.error:', m.text()); } });
  return { browser, page };
}
export const renderer = page => page.evaluate(() => { const gl = document.createElement('canvas').getContext('webgl2'); const d = gl?.getExtension('WEBGL_debug_renderer_info'); return d ? gl.getParameter(d.UNMASKED_RENDERER_WEBGL) : null; });
// 页面状态（DOM 文字＋画布 data-*＋相机），所有判据都从这里读
export const kstate = page => page.evaluate(() => {
  const t = id => document.getElementById(id)?.textContent?.trim() ?? null, el = id => document.getElementById(id);
  const row = id => ({ k: el(id).querySelector('.k').textContent, v: el(id).querySelector('.v').textContent, cls: el(id).querySelector('.v').className, hidden: el(id).hidden });
  const K = window.__kiosk, s = K?.scene, cam = s?.camera, tg = s?.controls?.target;
  return { view: document.body.dataset.view, statusK: t('status-k'), statusV: t('status-v'), statusS: t('status-s'),
    banner: el('banner').hidden ? '' : t('banner'), basemapNote: el('basemap-note').hidden ? '' : t('basemap-note'),
    fpgaTitle: t('fpga-title'), geo: row('fpga-geo'), roi: row('fpga-roi'), ar: row('fpga-ar'), hb: row('fpga-hb'),
    fpVisible: getComputedStyle(el('fp-cam')).display !== 'none', fpText: t('fp-cam-text'), fpgaVisible: getComputedStyle(el('fpga')).display !== 'none',
    camsVisible: getComputedStyle(el('cams')).display !== 'none', ds: { ...el('kiosk-canvas').dataset },
    camera: cam ? [cam.position.x, cam.position.y, cam.position.z, cam.fov] : null, target: tg ? [tg.x, tg.y, tg.z] : null,
    car: s?.car?.visible ? [s.car.position.x, -s.car.position.z, s.car.rotation.y] : null, camMode: s?.camMode,
    trail: s?.trail?.length ?? 0, pins: s?.pins?.children?.length ?? 0, beams: s ? s.pins.children.filter(p => p.children.some(c => c.userData.beam)).length : 0,
    labels: s ? s.pins.children.filter(p => p.children.some(c => c.isSprite)).length : 0,
    pinCls: s ? s.pins.children.reduce((a, p) => (a[p.userData.className] = (a[p.userData.className] || 0) + 1, a), {}) : {},
    pinIds: s ? s.pins.children.map(p => p.userData.id) : [], basemapStatus: K?.basemap?.status, connection: K?.connection, bench: K?.bench,
    seq: K?.snapshot?.seq ?? null };
});
export async function until(page, pred, ms = 30000, what = '') {
  const t0 = Date.now(); let last = null;
  while (Date.now() - t0 < ms) { last = await kstate(page); if (pred(last)) return { ms: Date.now() - t0, st: last }; await wait(200); }
  throw Error(`等不到 ${what}：${JSON.stringify(last).slice(0, 600)}`);
}
export function verdict(name, controlHolds, pass, extra = '') { const v = !controlHolds ? 'TESTBED-INVALID' : pass ? 'PASS' : 'FAIL'; console.log(`VERDICT ${name} ${v} ${extra}`); return v; }
