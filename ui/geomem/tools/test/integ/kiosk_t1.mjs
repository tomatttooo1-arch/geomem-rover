// 展台页判据 K6（docs/STAGE1_KIOSK_PREREG.md）：回放 d1247_full_1 → 真桥（python3.8，带底图包、不给 --map-id）→ 展台页（SwiftShader）。
//   TB_TOOLS=… GEOMEM_BASEMAP=<底图包目录> sh ui/geomem/bridge/testbed/run_tb.sh kiosk_t1.mjs <输出目录>
import fs from 'node:fs';
import { startReplay, startBridge, waitLog, waitHttp, stopAll, tokenFile, launch, kstate, renderer, until, verdict, wait, APP, PY38, REC_DIR, out } from './kiosk_tb.mjs';

const [dir] = process.argv.slice(2); out(dir);
const PKG = process.env.GEOMEM_BASEMAP;
const R = { samples: [], verdicts: {} };
const save = () => fs.writeFileSync(`${dir}/results_t1.json`, JSON.stringify(R, null, 1));
const tok = tokenFile(dir);
const rp = startReplay(PY38, `${REC_DIR}/d1247_full_1/pc.fpgarec`, tok, dir, 'd1247'); await waitLog(rp, /replay up/);
startBridge(PY38, tok, dir, 't1', ['--basemap', PKG]); await waitHttp(`${APP}/kiosk.html`);
const { browser, page } = await launch({ gpu: false });
page.setDefaultNavigationTimeout(180000);
try {
  R.renderer = await renderer(page);
  await page.goto(`${APP}/kiosk.html`);
  await until(page, s => s.connection === 'connected' && s.seq !== null, 60000, '连上回放');
  const t0 = Date.now();
  while (Date.now() - t0 < 30000) {
    const s = await kstate(page), nObj = await page.evaluate(() => window.__kiosk.snapshot?.objects?.length ?? 0);
    R.samples.push({ t: Date.now() - t0, banner: s.banner, note: s.basemapNote, cells: s.ds.basemapCells, carX: s.ds.carX, carY: s.ds.carY, status: s.basemapStatus, conn: s.connection, nObj, seq: s.seq });
    await wait(500);
  }
  await page.screenshot({ path: `${dir}/kiosk_t1_replay.png` });
  const S = R.samples, cars = new Set(S.filter(x => x.carX !== '').map(x => `${x.carX},${x.carY}`));
  const globalOk = page.errors.length === 0 && S.every(x => x.conn === 'connected') && S.every(x => x.nObj > 0);
  const checks = {
    banner100: S.length > 0 && S.every(x => x.banner === '回放：实车真实录制，非实时'),
    noBasemap100: S.every(x => x.note.startsWith('未加载底图（') && x.cells === '0'),
    carMoves: cars.size >= 2,
    noPageError: page.errors.length === 0,
  };
  R.checks = checks; R.distinctCar = cars.size; R.notesSeen = [...new Set(S.map(x => x.note))]; R.statusSeen = [...new Set(S.map(x => x.status))]; R.pageErrors = page.errors;
  R.verdicts.K6 = verdict('K6', globalOk, Object.values(checks).every(Boolean), `samples=${S.length} distinctCar=${cars.size} notes=${JSON.stringify(R.notesSeen)}`);
  save();
} catch (e) { R.fatal = String(e && e.stack || e); save(); console.log('FATAL', R.fatal); process.exitCode = 1; }
finally { await browser.close(); await stopAll(); }
