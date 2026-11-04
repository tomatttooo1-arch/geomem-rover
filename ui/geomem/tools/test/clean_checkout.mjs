// 干净检出（git archive HEAD）后：Node 演示服务起页，无 pageerror、三维视图起来（map-fallback 隐藏、canvas 有 WebGL2、云已画）、旧 SW 不再注册。
import { launch, wait } from './lib.mjs';
const [base, shot] = process.argv.slice(2);
const { browser, page } = await launch();
const failed = []; page.on('requestfailed', r => failed.push(r.url())); page.on('response', r => { if (r.status() >= 400) failed.push(r.status() + ' ' + r.url()); });
await page.goto(base + '/', { waitUntil: 'networkidle' }); await wait(2500);
const r = await page.evaluate(async () => ({ fallbackHidden: document.getElementById('map-fallback').hidden, cloudPoints: +document.getElementById('map-canvas').dataset.cloudPoints, webgl2: !!document.createElement('canvas').getContext('webgl2'), pill: document.getElementById('connection-pill').textContent.trim(), swRegs: (await navigator.serviceWorker.getRegistrations()).length }));
await page.screenshot({ path: shot });
console.log(JSON.stringify({ ...r, failedRequests: failed, pageerrors: page.errors }));
console.log('CLEAN', r.fallbackHidden && r.cloudPoints > 0 && !failed.length && !page.errors.length && r.swRegs === 0 ? 'PASS' : 'FAIL');
await browser.close();
