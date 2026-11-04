// stock 冒烟：python -m host.hub.hub_server --data-source replay --feed synthetic ＋真桥＋页面，记形状。
// 10-15 第 2 棒小修 a 起断言：sourceDetail=synthetic（hello 与快照），横幅为合成测试数据、不写「实车真实录制」；不符则退出码 1。
import { startPy, startBridge, waitLog, waitHttp, stopAll, tokenFile, RawClient, launch, snap, wait, waitPill, hooks, cloudInfo, readJson, out as mk, APP, PY313, HUB_PORT } from './tb.mjs';
const [dir] = process.argv.slice(2); mk(dir);
const tok = tokenFile(dir);
const hub = startPy(PY313, ['-m', 'host.hub.hub_server', '--data-source', 'replay', '--token-file', tok, '--feed', 'synthetic', '--bind', '127.0.0.1', '--port', String(HUB_PORT)], `${dir}/stock_hub.log`);
await waitLog(hub, /hub listening/);
startBridge(PY313, tok, dir, 'stock'); await waitHttp(`${APP}/`);
const c = await new RawClient().open();
const { browser, page } = await launch({ init: hooks() });
await page.goto(`${APP}/?source=live`, { waitUntil: 'networkidle' }); await waitPill(page, /回放/); await wait(3000);
const sn = c.last('snapshot'); const s = await snap(page);
await page.screenshot({ path: `${dir}/stock_synthetic.png` });
console.log(JSON.stringify({ hello: c.last('hello').capabilities, source: sn.source, robot: sn.robot, map: sn.map, nObjects: sn.objects.length, objects: sn.objects.slice(0, 2).map(o => [o.id, o.name, o.state]), memory: sn.memory, serverTime: sn.serverTime, task: sn.task.state }));
console.log(JSON.stringify({ page: { pill: s.pill, banner: s.banner, pose: s.pose, frame: s.frame, objs: s.objs, mapStatus: s.mapStatus, hint: s.hint }, cloud: await cloudInfo(page), pageerrors: page.errors.length }));
const BANNER = '回放：合成测试数据（不是实车录制），非实时';
const checks = { helloDetail: c.last('hello').sourceDetail === 'synthetic', snapDetail: sn.sourceDetail === 'synthetic', banner: s.banner === BANNER, noRecordingWord: !String(s.banner).includes('实车真实录制'), noPageErrors: page.errors.length === 0 };
console.log(JSON.stringify({ checks, pass: Object.values(checks).every(Boolean) }));
if (!Object.values(checks).every(Boolean)) process.exitCode = 1;
await browser.close(); c.close(); await stopAll();
console.log(JSON.stringify({ bridgeCounts: readJson(`${dir}/bridge_stats_stock.json`).counts }));
