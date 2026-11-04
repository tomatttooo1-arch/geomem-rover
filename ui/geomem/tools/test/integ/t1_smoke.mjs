// 冒烟（不算判据）：回放服务器＋真桥＋WS 客户端＋真前端，看 T1 形状。
import fs from 'node:fs';
import { startReplay, startBridge, waitLog, waitHttp, stopAll, tokenFile, RawClient, launch, snap, wait, waitPill, APP, PY38, REC_DIR } from './tb.mjs';
const [out, rec = 'd1247_full_1'] = process.argv.slice(2); fs.mkdirSync(out, { recursive: true });
const tok = tokenFile(out);
const rp = startReplay(PY38, `${REC_DIR}/${rec}/pc.fpgarec`, tok, out, 'smoke'); await waitLog(rp, /replay up/);
const br = startBridge(PY38, tok, out, 't1smoke'); await waitHttp(`${APP}/`);
const c = await new RawClient().open(); await wait(4000);
const sn = c.last('snapshot');
console.log('hello', JSON.stringify(c.last('hello')));
console.log('clouds', JSON.stringify(c.msgs.filter(m => m.type === 'cloud').map(x => x.header)));
console.log('snapshot', JSON.stringify({ ...sn, objects: sn.objects.slice(0, 2), nObjects: sn.objects.length }));
console.log('find', JSON.stringify(await c.command('find', { text: '杯子' })));
const { browser, page } = await launch();
await page.goto(`${APP}/?source=live`, { waitUntil: 'networkidle' }); console.log(JSON.stringify(await waitPill(page, /回放/)));
await wait(2000); console.log('page', JSON.stringify(await snap(page)), await page.locator('#map-watermark').innerText());
await page.screenshot({ path: `${out}/t1_smoke.png` });
{ const pg = await snap(page); const checks = { helloDetail: c.last('hello').sourceDetail === 'recording', snapDetail: c.last('snapshot').sourceDetail === 'recording', banner: pg.banner === '回放：实车真实录制，非实时' };
  console.log('detail_checks', JSON.stringify({ checks, pass: Object.values(checks).every(Boolean) })); if (!Object.values(checks).every(Boolean)) process.exitCode = 1; }
await browser.close(); c.close(); await stopAll();
