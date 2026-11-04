// sw.js 开发期处理：浏览器先装上 v1 的 SW（geomem-m8-v9 缓存），同一地址换成新版后再打开一次：旧 SW 被注销、缓存被删；
// 之后服务停掉再刷新，不再显示旧演示页（对照：v1 下服务停掉刷新仍显示缓存页，验收时已复现）。
import { launch, wait } from './lib.mjs';
import { spawn } from 'node:child_process';
const [v1App, v2App, port] = process.argv.slice(2);
const start = dir => spawn('node', [dir + '/server.mjs'], { env: { ...process.env, PORT: port, HOST: '127.0.0.1' }, stdio: 'ignore' });
const url = `http://127.0.0.1:${port}/`;
const { browser, page } = await launch();
let srv = start(v1App); await wait(800);
await page.goto(url, { waitUntil: 'networkidle' }); await page.evaluate(() => navigator.serviceWorker.ready); await page.reload({ waitUntil: 'networkidle' }); await wait(500);
const v1 = await page.evaluate(async () => ({ controlled: !!navigator.serviceWorker.controller, caches: await caches.keys() }));
srv.kill('SIGTERM'); await wait(800);
let v1Down; try { await page.reload({ waitUntil: 'load', timeout: 8000 }); v1Down = 'served: ' + await page.title(); } catch (e) { v1Down = 'error: ' + e.message.split('\n')[0]; }
srv = start(v2App); await wait(800);
await page.reload({ waitUntil: 'networkidle' }); await wait(2000);
await page.reload({ waitUntil: 'networkidle' }); await wait(1500);
const v2 = await page.evaluate(async () => ({ controlled: !!navigator.serviceWorker.controller, regs: (await navigator.serviceWorker.getRegistrations()).length, caches: await caches.keys(), hasSuggestionsId: !!document.getElementById('suggestions') }));
srv.kill('SIGTERM'); await wait(800);
let v2Down; try { await page.reload({ waitUntil: 'load', timeout: 8000 }); v2Down = 'served: ' + await page.title() + ' / body=' + (await page.locator('body').innerText()).slice(0, 60); } catch (e) { v2Down = 'error: ' + e.message.split('\n')[0]; }
console.log(JSON.stringify({ v1Installed: v1, v1ServerDownReload: v1Down, afterV2Load: v2, v2ServerDownReload: v2Down }));
console.log('SW', v1.controlled && /served/.test(v1Down) && v2.regs === 0 && v2.caches.length === 0 && v2.hasSuggestionsId && !/GeoMem/.test(v2Down) ? 'PASS' : 'FAIL');
await browser.close();
