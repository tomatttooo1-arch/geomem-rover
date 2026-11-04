// A3 校验（真桥）：(a) 上游坏 MEMORY 经真桥后不以非法帧到达页面；(b) 页内注入非法帧被拒。预注册 STAGE1_INTEG_PREREG.md。
import { startFixture, startBridge, waitLog, waitHttp, stopAll, tokenFile, RawClient, ctl, launch, snap, wait, waitPill, hooks, toasts, inject, readJson, verdict, out as mk, APP, PY313 } from './tb.mjs';
const [dir] = process.argv.slice(2); mk(dir);
const tok = tokenFile(dir);
const fx = startFixture(PY313, tok, dir); await waitLog(fx, /fixture up/);
startBridge(PY313, tok, dir, 'a3'); await waitHttp(`${APP}/`);
const c = await new RawClient().open();
const { browser, page } = await launch({ init: hooks() });
await page.goto(`${APP}/?source=live`, { waitUntil: 'networkidle' }); await waitPill(page); await wait(1500);
const objsText = () => page.locator('#metric-objects').innerText();
const revOf = s => s.memory.revision;
// 等到原始客户端看到 revision 满足条件的快照
const waitSnap = async (pred, ms = 8000) => { const t0 = Date.now(); while (Date.now() - t0 < ms) { const s = c.last('snapshot'); if (s && pred(s)) return s; await wait(100); } return null; };
const EXPECT = { nan_pos: 6, dup_id: 6, over_1000: 1000, inf_score: 6, nulls: 6, weird_state: 6 };
const rowsA = []; let aPass = true, aCtl = true;
for (const [kind, n] of Object.entries(EXPECT)) {
  // 坏帧保持 2.5 s（仪器修正：首跑坏帧只留 0.5 s，页面采样追不上）
  await ctl(`op=bad&kind=${kind}&hold=2.5`);
  const bad = await waitSnap(s => revOf(s) > 1000, 4000);
  await wait(1000); const pageBad = await objsText();
  const o1 = bad?.objects.find(o => o.id === 'object-0001');
  let extra = true;
  if (kind === 'inf_score') extra = o1 && o1.confidence === null && o1.lastSeen === null;
  if (kind === 'nulls') extra = o1 && o1.name === null && o1.state === null;
  if (kind === 'weird_state') extra = o1 && o1.state === null;
  const good = await waitSnap(s => revOf(s) < 1000 && s._t > bad?._t);
  await wait(600); const pageGood = await objsText();
  const row = { kind, rawObjects: bad?.objects.length ?? null, expect: n, extra, pageBad, pageGood, nextGoodObjects: good?.objects.length ?? null };
  row.pass = !!bad && bad.objects.length === n && extra && pageBad === `${n} 件物品`;
  row.controlHolds = !!good && good.objects.length === 6 && pageGood === '6 件物品';
  rowsA.push(row); aPass &&= row.pass; aCtl &&= row.controlHolds;
  if (kind === 'over_1000') await page.screenshot({ path: `${dir}/a3_over1000.png` });
}
const rejToastsA = (await toasts(page)).filter(t => /数据未被采用/.test(t));
for (const r of rowsA) console.log(JSON.stringify(r));
// (b) 页内注入
const base = c.last('snapshot');
const valid = () => JSON.parse(JSON.stringify(c.last('snapshot')));
const bads = [
  ['version 1 快照', () => JSON.stringify({ ...valid(), version: 1, seq: 1e9, map: { ...valid().map, name: 'INJ-V1' } })],
  ['文本 NaN', () => JSON.stringify({ ...valid(), seq: 1e9, map: { ...valid().map, name: 'INJ-NAN' } }).replace(/"position":\[[^\]]*\]/, '"position":[NaN,1,0]')],
  ['坐标系 base_link', () => JSON.stringify({ ...valid(), seq: 1e9, map: { ...valid().map, frame: 'base_link', name: 'INJ-FRAME' } })],
  ['云长度少 1 字节', () => { const h = Buffer.from(JSON.stringify({ type: 'cloud', version: 2, mapId: null, rev: 'global:999', frame: 'map', count: 2, source: 'grid', encoding: 'f32xyz+u8rgb' })); const pad = (4 - (4 + h.length) % 4) % 4; const b = Buffer.alloc(4 + h.length + pad + 2 * 15 - 1); b.writeUInt32LE(h.length, 0); h.copy(b, 4); return b.toString('base64'); }, true],
];
const rowsB = []; let bPass = true, bCtl = true;
for (const [name, make, bin] of bads) {
  const n0 = (await toasts(page)).length; const before = await snap(page);
  await inject(page, make(), !!bin); await wait(300);
  const tt = (await toasts(page)).slice(n0); const after = await snap(page);
  const tNext = Date.now(); await wait(700); const next = await snap(page); const fresh = await page.locator('#metric-fresh').innerText();
  const row = { name, toasts: tt, mapBefore: before.map, mapAfter: after.map, objsAfter: after.objs, freshAfterNext: fresh, cloud: await page.evaluate(() => document.getElementById('map-canvas').dataset.cloudRev) };
  row.pass = tt.filter(t => /数据未被采用/.test(t)).length === 1 && after.map === before.map && after.objs === before.objs && !/INJ-/.test(after.map) && row.cloud !== 'global:999';
  row.controlHolds = /^[01] 秒前$/.test(fresh) && next.objs === '6 件物品';
  rowsB.push(row); bPass &&= row.pass; bCtl &&= row.controlHolds;
}
for (const r of rowsB) console.log(JSON.stringify(r));
await page.screenshot({ path: `${dir}/a3_after_inject.png` });
const pe = page.errors.length; await browser.close(); c.close(); await stopAll();
const st = readJson(`${dir}/bridge_stats_a3.json`); console.log(JSON.stringify({ rejToastsDuringUpstreamBad: rejToastsA.length, bridgeCounts: st.counts, pageerrors: pe }));
verdict('A3', aCtl && bCtl, aPass && bPass && rejToastsA.length === 0 && pe === 0);
