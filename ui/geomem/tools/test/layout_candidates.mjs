// 候选列表不被裁（清单第 6 条；A5 的可见性部分在模拟桥上预跑）：1920×1080 与 390×844 下两个候选的 DOM 矩形都在可视滚动区内。
import { launch, wait, ctl } from './lib.mjs';
const [app, shots] = process.argv.slice(2); const c = ctl('http://127.0.0.1:19191');
let all = true;
for (const vp of [{ width: 1920, height: 1080 }, { width: 390, height: 844 }]) {
  await c('op=reset&tier=map&points=3000&objects=6&rate=5&source=live&link=up');
  const { browser, page } = await launch({ viewport: vp });
  await page.goto(`${app}/?source=live&ws=ws://127.0.0.1:19190/ws`, { waitUntil: 'networkidle' }); await wait(2500);
  await page.fill('#search-input', '杯子'); await page.click('#find-button'); await wait(200); await page.click('[data-find]'); await wait(1500);
  await page.locator('#task-candidates').scrollIntoViewIfNeeded();
  await wait(400);
  const r = await page.evaluate(() => {
    const vis = (el) => { const r = el.getBoundingClientRect(); let ok = r.top >= 0 && r.bottom <= innerHeight && r.left >= 0 && r.right <= innerWidth; for (let p = el.parentElement; p; p = p.parentElement) { const cs = getComputedStyle(p); if (/(auto|scroll|hidden)/.test(cs.overflowY + cs.overflowX)) { const q = p.getBoundingClientRect(); if (r.top < q.top - 1 || r.bottom > q.bottom + 1) ok = false; } } return { ok, top: Math.round(r.top), bottom: Math.round(r.bottom) }; };
    return [...document.querySelectorAll('[data-candidate]')].map(b => ({ id: b.dataset.candidate, ...vis(b) }));
  });
  await page.screenshot({ path: `${shots}/layout_candidates_${vp.width}.png` });
  const ok = r.length === 2 && r.every(x => x.ok); all &&= ok;
  console.log(JSON.stringify({ viewport: vp, candidates: r, ok }));
  await page.click('#stop-task').catch(() => {}); await wait(500);
  await browser.close();
}
console.log('LAYOUT', all ? 'PASS' : 'FAIL');
