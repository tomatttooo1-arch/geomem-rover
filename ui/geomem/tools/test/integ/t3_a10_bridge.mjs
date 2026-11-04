// T3 A10「桥」臂客户端（第 2 棒）：真桥＋一个 1920×1080 无头页面连着本趟 hub，保持 <秒数> 后按 pid 全停。
// 在车的网络命名空间里的试验台容器中跑（bridge/testbed/t3_tb.sh a10）。参数：<输出目录> <口令文件> <秒数>。
import { startBridge, waitHttp, stopAll, RawClient, launch, wait, waitPill, readJson, out as mk, APP, PY313 } from './tb.mjs';
const [dir, tok, secs = '140'] = process.argv.slice(2); mk(dir);
startBridge(PY313, tok, dir, 'a10'); await waitHttp(`${APP}/`, 30000);
const { browser, page } = await launch();
await page.goto(`${APP}/?source=live`, { waitUntil: 'networkidle' });
const pill = await waitPill(page, /仿真数据/, 60000);
// 一次性核链路（读 hello 后即关，不常驻第二个 WS 会话）
const c = await new RawClient().open(); const h = await c.next(m => m.type === 'hello', 5000).catch(() => null); c.close();
console.log(new Date().toISOString(), 'A10 bridge up', JSON.stringify({ pill, link: h?.bridge?.link, control: h?.capabilities?.control }));
await wait(Number(secs) * 1000);
console.log(new Date().toISOString(), 'A10 bridge done', JSON.stringify({ pageerrors: page.errors.length }));
await browser.close(); await stopAll();
try { console.log(JSON.stringify({ bridgeCounts: readJson(`${dir}/bridge_stats_a10.json`).counts })); } catch {}
process.exit(0);
