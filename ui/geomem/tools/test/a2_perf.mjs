// A2：4090 无头 Chrome，6 万点云发一次＋快照 5 Hz×100 个物体，跑 60 s：帧率中位 ≥ 50 fps；云到达后稳态主线程 >50 ms 长任务 ≤ 1 次；云解码到上屏 ≤ 30 ms。
// 阳性对照：同台 v1 每秒 5 帧 6 万点复现 ≤ 20 fps（验收约 12 fps）。
import { launch, wait, ctl } from './lib.mjs';
import { execSync } from 'node:child_process';
const [appV2, appV1, shots, mode = 'gpu', durS = '60'] = process.argv.slice(2);
const DUR = Number(durS) * 1000;
const instrument = () => {
  window.__h = []; window.__lt = [];
  const W = window.WebSocket;
  window.WebSocket = class extends W {
    set onmessage(fn) { super.onmessage = ev => { const bin = ev.data instanceof ArrayBuffer; const revBefore = document.getElementById('map-canvas')?.dataset.cloudRev; const t0 = performance.now(); fn(ev); const t1 = performance.now();
      const applied = document.getElementById('map-canvas')?.dataset.cloudRev !== revBefore; const rec = { bin, ms: t1 - t0, applied, len: bin ? ev.data.byteLength : ev.data.length };
      if (applied) requestAnimationFrame(() => { rec.toFrameMs = performance.now() - t1; });  // 场景自己的 rAF 先注册，这个回调在新云那一帧 render 之后运行
      window.__h.push(rec); }; }
    get onmessage() { return super.onmessage; }
  };
  try { new PerformanceObserver(l => { for (const e of l.getEntries()) window.__lt.push({ t: e.startTime, d: e.duration }); }).observe({ type: 'longtask', buffered: true }); } catch {}
};
async function orbitAndCount(page, ms) {
  const box = await page.locator('#map-canvas').boundingBox();
  const fpsP = page.evaluate(ms => new Promise(res => { const per = []; let n = 0, sec = performance.now(); const t0 = sec; const f = t => { n++; if (t - sec >= 1000) { per.push(n * 1000 / (t - sec)); n = 0; sec = t; } if (t - t0 < ms) requestAnimationFrame(f); else res(per); }; requestAnimationFrame(f); }), ms);
  await page.mouse.move(box.x + box.width / 2, box.y + box.height / 2); await page.mouse.down();
  const steps = Math.floor(ms / 50);
  for (let i = 0; i < steps; i++) { await page.mouse.move(box.x + box.width / 2 + 200 * Math.sin(i / 10), box.y + box.height / 2 + 30 * Math.cos(i / 10)); await wait(40); }
  await page.mouse.up();
  const per = await fpsP; const s = [...per].sort((a, b) => a - b);
  return { seconds: per.length, medianFps: +s[Math.floor(s.length / 2)].toFixed(1), p10Fps: +s[Math.floor(s.length * .1)].toFixed(1), minFps: +s[0].toFixed(1) };
}
const results = {};
// ---- v2
{
  const c = ctl('http://127.0.0.1:19191');
  await c('op=reset&tier=map&source=live&link=up&op=resume_stream&rate=5&objects=100&points=60000');
  const { browser, page } = await launch({ gpu: mode === 'gpu', init: { fn: instrument } });
  await page.goto(`${appV2}/?source=live&ws=ws://127.0.0.1:19190/ws`, { waitUntil: 'networkidle' }); await wait(3000);
  const renderer = await page.evaluate(() => { const g = document.createElement('canvas').getContext('webgl2'); const d = g.getExtension('WEBGL_debug_renderer_info'); return g.getParameter(d.UNMASKED_RENDERER_WEBGL); });
  // 再发 5 次新云（新 rev），采解码与上屏
  for (let i = 0; i < 5; i++) { await c('points=60000'); await wait(1500); }
  const cloudRecs = await page.evaluate(() => window.__h.filter(r => r.bin || r.applied));
  const steadyStart = await page.evaluate(() => performance.now());
  try { results.nvidia = execSync('nvidia-smi --query-gpu=index,utilization.gpu,memory.used --format=csv,noheader').toString().trim().split('\n'); } catch {}
  const fps = await orbitAndCount(page, DUR);
  const lt = await page.evaluate(t => window.__lt.filter(x => x.t >= t), steadyStart);
  const snaps = await page.evaluate(t => { const a = window.__h.filter(r => !r.bin); const s = a.map(r => r.ms).sort((x, y) => x - y); return { n: a.length, medMs: +s[Math.floor(s.length / 2)].toFixed(2), p99Ms: +s[Math.floor(s.length * .99)].toFixed(2), bytes: a.at(-1)?.len }; });
  const decode = cloudRecs.filter(r => r.bin).map(r => +r.ms.toFixed(1)), apply = cloudRecs.filter(r => r.applied).map(r => ({ applyMs: +r.ms.toFixed(1), toFrameMs: +(r.toFrameMs ?? NaN).toFixed(1) }));
  const n = Math.min(decode.length, apply.length);
  const d2s = Array.from({ length: n }, (_, i) => +(decode[decode.length - n + i] + apply[apply.length - n + i].applyMs + apply[apply.length - n + i].toFrameMs).toFixed(1));
  await page.screenshot({ path: `${shots}/a2_v2_60k_${mode}.png` });
  results.v2 = { renderer, cloudPoints: await page.evaluate(() => +document.getElementById('map-canvas').dataset.cloudPoints), objects: await page.locator('#metric-objects').innerText(), fps, longTasksOver50ms: lt.filter(x => x.d > 50).length, longTasksAll: lt.map(x => Math.round(x.d)), cloudDecodeMs: decode, cloudApply: apply, decodeToScreenMs: d2s, snapshotHandler: snaps, pageerrors: page.errors.length };
  results.v2.pass = fps.medianFps >= 50 && results.v2.longTasksOver50ms <= 1 && d2s.length > 0 && Math.max(...d2s) <= 30;
  await browser.close();
  await c('objects=6&points=3000');
}
// ---- v1 阳性对照
{
  const c = ctl('http://127.0.0.1:19091');
  await c('op=reset&points=60000&rate=5&op=resume_stream');
  const { browser, page } = await launch({ gpu: mode === 'gpu' });
  await page.goto(`${appV1}/`, { waitUntil: 'networkidle' }); await wait(1000);
  await page.click('#settings-button'); await wait(400); await page.check('input[name="source"][value="live"]'); await page.fill('#endpoint', 'ws://127.0.0.1:19090/m8');
  await page.click('#connection-form button[type=submit]'); await wait(4000);
  const fps = await orbitAndCount(page, Math.min(DUR, 20000));
  await page.screenshot({ path: `${shots}/a2_v1control_60k_${mode}.png` });
  results.v1control = { fps, controlHolds: fps.medianFps <= 20 };
  await browser.close();
  await c('points=3000&rate=2');
}
console.log(JSON.stringify(results, null, 1));
console.log('A2', !results.v1control.controlHolds ? 'TESTBED-INVALID' : results.v2.pass ? 'PASS' : 'FAIL');
