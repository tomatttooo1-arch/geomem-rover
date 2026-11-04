// 展台页 10-18 判据 K7a–K7e（docs/STAGE2_KIOSK_ODOM_PREREG.md，跑前写定）：里程档画底图、运行中加载、id 错一位与 sim 不画。
//   TB_TOOLS=… GEOMEM_BASEMAP=<1 cm 参照包目录> sh ui/geomem/bridge/testbed/run_tb.sh kiosk_t3.mjs <输出目录> gpu|swr
import fs from 'node:fs';
import path from 'node:path';
import { startFixture, startBridge, stopPid, waitLog, waitHttp, stopAll, tokenFile, ctl, launch, kstate, renderer, until, verdict, wait, APP, PY313, SAMPLE_SHA, LOOK, out } from './kiosk_tb.mjs';

const [dir, arm = 'gpu'] = process.argv.slice(2); out(dir);
const GPU = arm === 'gpu';
const PKG = process.env.GEOMEM_BASEMAP;
const MAN = JSON.parse(fs.readFileSync(`${PKG}/basemap.json`, 'utf8'));
const CELLS = MAN.files.cells.count;
const POSES_F = `${LOOK}/export/cloud_poses.txt`;
const WRONG = SAMPLE_SHA.slice(0, -1) + (SAMPLE_SHA.at(-1) === '6' ? '7' : '6');
const WATCH = out(`${dir}/watch`);
const NAME = `basemap-${SAMPLE_SHA.slice(0, 12)}`;
const R = { arm, cells: CELLS, checks: {}, verdicts: {}, notes: [] };
const save = () => fs.writeFileSync(`${dir}/results_t3_${arm}.json`, JSON.stringify(R, null, 1));
function judge(name, control, checks, extra = '') { R.checks[name] = { control, ...checks, extra }; const pass = Object.values(checks).every(v => v === true); R.verdicts[name] = verdict(name, control, pass, extra); save(); }
const NOTE_ODOM = '底图：20261015_190638.db · 按建图起点对齐，未做定位校正 · 底图校验依据：操作员配置';
const NOTE_LOC = '底图：20261015_190638.db · 底图校验依据：操作员配置';
const snapBm = page => page.evaluate(() => ({ bm: window.__kiosk?.snapshot?.map?.basemap ?? null, source: window.__kiosk?.snapshot?.source ?? null, mapId: window.__kiosk?.snapshot?.map?.id ?? null }));

if (fs.existsSync(`${WATCH}/${NAME}`)) throw Error('监视目录里已有包：换一个输出目录');
const tok = tokenFile(dir);
let fx = startFixture(PY313, tok, dir, ['--tier', 'odom']); await waitLog(fx, /fixture up/);
const scene = async () => {
  await ctl(`op=objects&file=${LOOK}/memory_final.json&age=120`);
  await ctl(`op=walk&file=${POSES_F}&upto=16&dt=0.05`);
  await ctl('op=kv&geo=12&rows=8&hb=1&roi_age=200');
};
await scene();
let br = startBridge(PY313, tok, dir, `${arm}_t3ok`, ['--map-id', SAMPLE_SHA, '--basemap-watch', WATCH]); await waitHttp(`${APP}/kiosk.html`);
const { browser, page } = await launch({ gpu: GPU });
page.setDefaultNavigationTimeout(180000);
R.renderer = await renderer(page);
const armOk = GPU ? /NVIDIA/.test(R.renderer || '') : /SwiftShader/.test(R.renderer || '');
console.log('renderer', R.renderer, 'armOk', armOk);

try {
  // ================================================================ K7d 运行中加载
  await page.goto(`${APP}/kiosk.html`);
  const w0 = await until(page, s => s.connection === 'connected' && s.basemapStatus === 'mismatch', 60000, '连上且 mismatch').catch(e => ({ err: String(e) }));
  const pre = await kstate(page);
  const preNoteOk = pre.basemapNote === '未加载底图（还没有底图包，建图结束后生成）' && +pre.ds.basemapCells === 0;
  const marker = Math.random().toString(36).slice(2);
  await page.evaluate(m => { window.__k7marker = m; }, marker);
  const nav0 = await page.evaluate(() => performance.getEntriesByType('navigation').length);
  // 对照：放包前 3 s 一直没画
  let drewEarly = false; for (let i = 0; i < 15; i++) { if (+(await kstate(page)).ds.basemapCells > 0) drewEarly = true; await wait(200); }
  // make_basemap 的写法：临时名里写全，再一次改名
  const tmp = `${WATCH}/${NAME}.tmp-k7`;
  fs.cpSync(PKG, tmp, { recursive: true });
  await wait(2500);                                   // 至少一次轮询看见临时名
  const tmpIgnored = +(await kstate(page)).ds.basemapCells === 0;
  const t0 = Date.now(); fs.renameSync(tmp, `${WATCH}/${NAME}`);
  const r = await until(page, s => +s.ds.basemapCells === CELLS && s.ds.basemapSha === SAMPLE_SHA, 20000, '运行中加载').catch(e => ({ err: String(e) }));
  const loadMs = Date.now() - t0;
  const marker2 = await page.evaluate(() => window.__k7marker), nav1 = await page.evaluate(() => performance.getEntriesByType('navigation').length);
  judge('K7d', armOk && !w0.err && !drewEarly && preNoteOk, { loaded: !r.err && loadMs <= 20000, noReload: marker2 === marker && nav1 === nav0, tmpIgnored },
    `loadMs=${loadMs} preNote=${pre.basemapNote} nav=${nav0}->${nav1}`);

  // ================================================================ K7a 里程档＋id 对 → 画、提示
  await wait(1500);
  const sa = await kstate(page), ba = await snapBm(page); R.k7aState = sa; R.k7aBm = ba;
  await page.screenshot({ path: `${dir}/k7a_odom_${arm}.png` });
  judge('K7a', armOk && ba.bm?.tier === 'odom', {
    status: sa.basemapStatus === 'ok' && ba.bm?.basis === 'mapping_start' && ba.mapId === null,
    drawn: +sa.ds.basemapCells === CELLS && sa.ds.basemapSha === SAMPLE_SHA,
    note: sa.basemapNote === NOTE_ODOM,
    banner: /仅里程计定位/.test(sa.banner),
  }, `note=${sa.basemapNote} banner=${sa.banner}`);

  // ================================================================ K7e 定位档回归
  await ctl('op=tier&tier=loc');
  const re = await until(page, s => s.basemapNote === NOTE_LOC, 10000, '定位档说明').catch(e => ({ err: String(e) }));
  const se = await kstate(page), be = await snapBm(page);
  await ctl('op=tier&tier=odom');
  const re2 = await until(page, s => s.basemapNote === NOTE_ODOM, 10000, '回里程档说明').catch(e => ({ err: String(e) }));
  const se2 = await kstate(page);
  judge('K7e', armOk, { loc: !re.err && be.bm?.basis === 'localized' && be.mapId === SAMPLE_SHA && +se.ds.basemapCells === CELLS,
    backToOdom: !re2.err && +se2.ds.basemapCells === CELLS }, `loc=${se.basemapNote} odom=${se2.basemapNote}`);

  // ================================================================ K7b 阳性对照：id 错一位
  await stopPid(br);
  br = startBridge(PY313, tok, dir, `${arm}_t3wrong`, ['--map-id', WRONG, '--basemap-watch', WATCH]); await waitHttp(`${APP}/kiosk.html`);
  await page.goto(`${APP}/kiosk.html`);
  const rb = await until(page, s => s.connection === 'connected' && s.basemapStatus === 'mismatch', 60000, 'mismatch').catch(e => ({ err: String(e) }));
  await wait(3000);
  const sb = await kstate(page), bb = await snapBm(page);
  await page.screenshot({ path: `${dir}/k7b_wrongid_${arm}.png` });
  judge('K7b', armOk && R.verdicts.K7a === 'PASS' && (bb.bm?.loaded || []).includes(SAMPLE_SHA), {
    notDrawn: !rb.err && sb.basemapStatus === 'mismatch' && +sb.ds.basemapCells === 0 && sb.basemapNote === '未加载底图（当前地图与底图包不符）',
  }, `wrong=${WRONG} note=${sb.basemapNote} loaded=${JSON.stringify(bb.bm?.loaded)}`);

  // ================================================================ K7c sim → 不画
  await stopPid(br); await stopPid(fx);
  fx = startFixture(PY313, tok, dir, ['--data-source', 'sim', '--tier', 'odom']); await waitLog(fx, /fixture up.*data_source=sim/);
  await scene();
  br = startBridge(PY313, tok, dir, `${arm}_t3sim`, ['--map-id', SAMPLE_SHA, '--basemap-watch', WATCH]); await waitHttp(`${APP}/kiosk.html`);
  await page.goto(`${APP}/kiosk.html`);
  const rc = await until(page, s => s.connection === 'connected' && s.basemapStatus === 'not_localized', 60000, 'sim').catch(e => ({ err: String(e) }));
  await wait(3000);
  const sc = await kstate(page), bc = await snapBm(page);
  await page.screenshot({ path: `${dir}/k7c_sim_${arm}.png` });
  judge('K7c', armOk && bc.source === 'sim' && (bc.bm?.loaded || []).includes(SAMPLE_SHA), {
    notDrawn: !rc.err && bc.bm?.tier === 'sim' && +sc.ds.basemapCells === 0 && sc.basemapNote === '未加载底图（仿真数据）',
  }, `note=${sc.basemapNote} banner=${sc.banner} source=${bc.source}`);

  R.pageErrors = page.errors; R.consoleErrors = page.consoleErrors;
  R.verdicts.global = page.errors.length === 0 ? 'OK' : 'PAGEERROR'; save();
} catch (e) {
  R.fatal = String(e && e.stack || e); save(); console.log('FATAL', R.fatal); process.exitCode = 1;
} finally {
  await browser.close(); await stopAll();
}
console.log('VERDICTS', JSON.stringify(R.verdicts));
