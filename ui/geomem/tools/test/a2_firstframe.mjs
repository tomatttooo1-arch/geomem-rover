// 拆分「连接后第一份点云」的上屏延迟：同样首连，6 万点 vs 0 点，看首个快照处理＋首帧耗时差多少是点云造成的。
import { launch, wait, ctl } from './lib.mjs';
const [app, mode = 'gpu'] = process.argv.slice(2); const c = ctl('http://127.0.0.1:19191');
const instrument = () => { window.__h = []; const W = window.WebSocket; window.WebSocket = class extends W { set onmessage(fn) { super.onmessage = ev => { const t0 = performance.now(); fn(ev); const t1 = performance.now(); const rec = { bin: ev.data instanceof ArrayBuffer, ms: t1 - t0, type: ev.data instanceof ArrayBuffer ? 'cloud' : JSON.parse(ev.data).type }; requestAnimationFrame(() => { rec.toFrameMs = performance.now() - t1; }); window.__h.push(rec); }; } get onmessage() { return super.onmessage; } }; };
for (const pts of [60000, 0, 60000, 0]) {
  await c(`op=reset&tier=map&rate=5&objects=100&points=${pts}`);
  const { browser, page } = await launch({ gpu: mode === 'gpu', init: { fn: instrument } });
  await page.goto(`${app}/?source=live&ws=ws://127.0.0.1:19190/ws`, { waitUntil: 'networkidle' }); await wait(2500);
  const h = await page.evaluate(() => window.__h.slice(0, 4).map(r => ({ type: r.type, ms: +r.ms.toFixed(1), toFrameMs: +(r.toFrameMs ?? NaN).toFixed(1) })));
  console.log(pts, JSON.stringify(h));
  await browser.close();
}
await c('objects=6&points=3000');
