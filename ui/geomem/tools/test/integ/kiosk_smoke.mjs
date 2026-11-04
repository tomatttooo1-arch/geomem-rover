// 冒烟（不算判据）：夹具＋真桥（带底图包）＋展台页，教室场景截一张图、打印页面状态。
import fs from 'node:fs';
import { startFixture, startBridge, waitLog, waitHttp, stopAll, tokenFile, ctl, launch, kstate, renderer, wait, APP, PY313, SAMPLE_SHA, LOOK, out } from './kiosk_tb.mjs';
const [dir, gpuArg = 'gpu'] = process.argv.slice(2); out(dir);
const PKG = process.env.GEOMEM_BASEMAP;
const tok = tokenFile(dir);
const fx = startFixture(PY313, tok, dir); await waitLog(fx, /fixture up/);
await ctl(`op=objects&file=${LOOK}/memory_final.json&age=120`);
await ctl(`op=walk&file=${LOOK}/export/cloud_poses.txt&upto=0&dt=0.05`);   // 先停在轨迹起点，页面连上后再走（否则页面累积轨迹只有 1 点）
await ctl('op=kv&geo=12&rows=8&hb=1&roi_age=200');
await ctl('op=task&state=navigating&reason=reobserve&target=object-0007');
const br = startBridge(PY313, tok, dir, 'smoke', ['--map-id', SAMPLE_SHA, '--basemap', PKG]); await waitHttp(`${APP}/kiosk.html`);
const { browser, page } = await launch({ gpu: gpuArg === 'gpu' });
console.log('renderer', await renderer(page));
await page.goto(`${APP}/kiosk.html`);
const t0 = Date.now();
while (Date.now() - t0 < 60000) { const s = await kstate(page); if (+s.ds.basemapCells > 0) break; await wait(300); }
await ctl(`op=walk&file=${LOOK}/export/cloud_poses.txt&upto=16&dt=0.2`);
await wait(16 * 200 + 2500);
console.log(JSON.stringify(await kstate(page), null, 1));
await page.screenshot({ path: `${dir}/smoke_${gpuArg}.png` });
await browser.close(); await stopAll();
