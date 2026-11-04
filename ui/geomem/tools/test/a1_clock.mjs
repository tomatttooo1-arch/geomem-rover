// A1：浏览器 Date 整体偏 ±3600 s 各跑 60 s；v2 连接标签全程「实车已连接」、无「数据已过期」；「N 秒前见到」与 serverTime − lastSeen 差 ≤ 1 s。
// 阳性对照：同样偏移下 v1 界面（3426dde）＋ v1 模拟桥 6 s 内显示「数据已过期」。
import { launch, wait, ctl } from './lib.mjs';
const [appV2, appV1, shots, durS = '60'] = process.argv.slice(2);
const DUR = Number(durS) * 1000;
const shiftDate = off => { const D = Date; class FD extends D { constructor(...a) { if (a.length === 0) super(D.now() + off); else super(...a); } static now() { return D.now() + off; } } FD.parse = D.parse; FD.UTC = D.UTC; globalThis.Date = FD; };
const c2 = ctl('http://127.0.0.1:19191');
let allPass = true;
for (const off of [3600e3, -3600e3]) {
  await c2('op=reset&tier=map&points=3000&source=live&link=up&objects=6&op=resume_stream&rate=5');
  const { browser, page } = await launch({ init: { fn: shiftDate, arg: off } });
  await page.goto(`${appV2}/?source=live&ws=ws://127.0.0.1:19190/ws`, { waitUntil: 'networkidle' }); await wait(2000);
  const dateSkew = await page.evaluate(() => (Date.now() - performance.timeOrigin - performance.now()) / 1000);
  const labels = new Map(); let maxDiff = 0, samples = 0, stale = 0, diffSamples = 0;
  const t0 = Date.now();
  while (Date.now() - t0 < DUR) {
    const [pill, seen, st] = await Promise.all([
      page.locator('#connection-pill').innerText(),
      page.evaluate(() => [...document.querySelectorAll('[data-seen-id]')].map(e => [e.dataset.seenId, e.querySelector('.seen')?.dataset.seconds])),
      c2('op=state')]);
    samples++; labels.set(pill, (labels.get(pill) || 0) + 1); if (/过期/.test(pill)) stale++;
    for (const [id, sec] of seen) { if (sec === '' || sec === undefined) continue; const o = st.objects.find(o => o.id === id); const truth = st.serverTime - o.lastSeen; maxDiff = Math.max(maxDiff, Math.abs(Number(sec) - truth)); diffSamples++; }
    await wait(500);
  }
  await page.screenshot({ path: `${shots}/a1_v2_offset_${off > 0 ? 'plus' : 'minus'}3600.png` });
  const pass = stale === 0 && [...labels.keys()].every(l => l === '实车已连接') && maxDiff <= 1 && diffSamples > 0;
  allPass &&= pass;
  console.log(JSON.stringify({ arm: 'v2', offsetS: off / 1000, measuredDateSkewS: +dateSkew.toFixed(1), samples, labels: Object.fromEntries(labels), staleSamples: stale, seenCompared: diffSamples, maxSeenDiffS: +maxDiff.toFixed(3), pass, pageerrors: page.errors.length }));
  await browser.close();
}
// 阳性对照：v1
const c1 = ctl('http://127.0.0.1:19091');
let ctrlOk = true;
for (const off of [3600e3, -3600e3]) {
  await c1('op=reset&points=3000&rate=2&op=resume_stream');
  const { browser, page } = await launch({ init: { fn: shiftDate, arg: off } });
  await page.goto(`${appV1}/`, { waitUntil: 'networkidle' }); await wait(1000);
  await page.click('#settings-button'); await wait(400); await page.check('input[name="source"][value="live"]'); await page.fill('#endpoint', 'ws://127.0.0.1:19090/m8');
  const t0 = Date.now(); await page.click('#connection-form button[type=submit]');
  let firstStale = null, seq = [];
  while (Date.now() - t0 < 6000) { const pill = await page.locator('#connection-pill').innerText(); seq.push(pill); if (/过期/.test(pill) && firstStale === null) firstStale = Date.now() - t0; await wait(250); }
  await page.screenshot({ path: `${shots}/a1_v1control_offset_${off > 0 ? 'plus' : 'minus'}3600.png` });
  const ok = firstStale !== null; ctrlOk &&= ok;
  console.log(JSON.stringify({ arm: 'v1-control', offsetS: off / 1000, firstStaleMs: firstStale, labels: [...new Set(seq)], controlHolds: ok }));
  await browser.close();
}
console.log('A1', !ctrlOk ? 'TESTBED-INVALID' : allPass ? 'PASS' : 'FAIL');
