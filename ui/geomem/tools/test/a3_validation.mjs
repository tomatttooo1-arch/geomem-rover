// A3：非法帧全部被拒、画面不变、中文提示；每个非法帧后紧跟合法帧必须被采用；可空字段为 null 的合法帧被采用。
import { launch, snap, wait, ctl, clearToast } from './lib.mjs';
const [app, shots] = process.argv.slice(2); const c = ctl('http://127.0.0.1:19191');
await c('op=reset&tier=map&points=3000&source=live&link=up&objects=6');
const { browser, page } = await launch();
await page.goto(`${app}/?source=live&ws=ws://127.0.0.1:19190/ws`, { waitUntil: 'networkidle' }); await wait(2500);
await c('op=pause_stream');
const cloud = () => page.evaluate(() => +document.getElementById('map-canvas').dataset.cloudPoints);
// 按事件等待（10-15 联调第 3 棒 c）：原来是固定 500/700/600 ms。第 2、3 棒各有一趟首行 version1 失败：「之前」采样时页面还没画出
// 第一帧（地图名仍是「等待空间地图」，SwiftShader 首帧卡主线程 3–5 s），之后那帧合法快照才到，「未变」比较就错了；
// 拒收本身正常（中文提示、下一帧采用）。改为等页面状态满足条件或超时，判定条件一字不改。seqback 没有可等的事件，仍固定 700 ms。
async function until(fn, ms = 6000) { const t0 = Date.now(); while (Date.now() - t0 < ms) { const v = await fn(); if (v) return v; await wait(50); } return null; }
const mapNow = async () => (await snap(page)).map;
const validAdopted = async (isCloud, prevMap, prevCloud) => isCloud ? until(async () => { const n = await cloud(); return n !== prevCloud && n > 700; }) : until(async () => { const m = await mapNow(); return m !== prevMap && /VALID-/.test(m); });
const ascii = s => /[A-Za-z]{4,}/.test(s.replace(/BAD-[\w-]+|VALID-\d+|NULLFIELDS-OK|map|odom|base_link|ws|MB/g, ''));
let pass = true; const rows = [];
const kinds = ['version1','nantext','infinity','badframe','dupid','toomanyobjects','pointsinsnapshot','seqback','notjson','cloud_badheader','cloud_short','cloud_nonfinite','cloud_toomany'];
for (const kind of kinds) {
  const isCloud = kind.startsWith('cloud_');
  { const m0 = await mapNow(), c0 = await cloud(); await c('inject=' + (isCloud ? 'cloud_valid' : 'valid')); await validAdopted(isCloud, m0, c0); await wait(150); }
  const before = await snap(page), cBefore = await cloud();
  await clearToast(page);
  await c('inject=' + kind); if (kind === 'seqback') await wait(700); else { await until(async () => (await snap(page)).toast); await wait(150); }
  const after = await snap(page), cAfter = await cloud();
  const unchanged = after.map === before.map && after.objs === before.objs && cAfter === cBefore;
  const toastOk = kind === 'seqback' ? after.toast === '' : !!after.toast && !ascii(after.toast);  // 旧序号帧按 §2.3 静默丢弃（同 v1），不弹提示
  await c('inject=' + (isCloud ? 'cloud_valid' : 'valid')); await validAdopted(isCloud, after.map, cAfter);
  const next = await snap(page), cNext = await cloud();
  const adopted = isCloud ? cNext !== cAfter && cNext > 700 : next.map !== after.map && /VALID-/.test(next.map);
  const ok = unchanged && toastOk && adopted; pass &&= ok;
  rows.push({ kind, rejected_unchanged: unchanged, toast: after.toast, chinese: toastOk, next_valid_adopted: adopted, ...(isCloud ? { cloudBefore: cBefore, cloudAfter: cAfter, cloudNext: cNext } : { mapBefore: before.map, mapNext: next.map }) });
}
await clearToast(page);
await c('inject=nullfields'); await until(async () => /NULLFIELDS-OK/.test(await mapNow()));
const nf = await snap(page); const cards = await page.locator('#objects-grid').innerText();
const nullOk = /NULLFIELDS-OK/.test(nf.map) && !nf.toast && /见到时间未知/.test(cards);
rows.push({ kind: 'nullfields(合法)', adopted: nullOk, map: nf.map, toast: nf.toast, firstCard: cards.split('\n').slice(0, 3).join(' / ') });
pass &&= nullOk;
await page.screenshot({ path: `${shots}/a3_nullfields.png` });
for (const r of rows) console.log(JSON.stringify(r));
console.log('A3', pass ? 'PASS' : 'FAIL', 'pageerrors=', page.errors.length);
await c('op=resume_stream');
await browser.close();
