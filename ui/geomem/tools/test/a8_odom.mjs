// A8（前端侧）：odom 档（pose_source=reference、无定位位姿、替身 /map）：坐标系显示 odom、徽标「里程推算」、替身图不画、滚动栅格障碍画出；
// 小车位姿缺失（H2 未做）时小车不显示且有说明。阳性对照：同夹具换成有占据格的真图（map 档），被画出。
// 注：替身判定（全图无占据格）是桥的事，这里只验「桥给了 placeholder/odom 标志时」前端的显示。
import { launch, snap, wait, ctl } from './lib.mjs';
const [app, shots] = process.argv.slice(2); const c = ctl('http://127.0.0.1:19191');
await c('op=reset&tier=odom&source=live&link=up&op=resume_stream&caps=' + encodeURIComponent(JSON.stringify({ pause: false, control: 'controller' })));
const { browser, page } = await launch();
await page.goto(`${app}/?source=live&ws=ws://127.0.0.1:19190/ws`, { waitUntil: 'networkidle' }); await wait(3000);
const cloud = () => page.evaluate(() => ({ points: +document.getElementById('map-canvas').dataset.cloudPoints, rev: document.getElementById('map-canvas').dataset.cloudRev }));
const rows = [];
let s = await snap(page), cl = await cloud();
await page.fill('#search-input', '杯子'); await page.click('#find-button'); await wait(300);
const findEnabled = !(await page.locator('[data-find]').isDisabled()); const hint = (await snap(page)).hint;
const r1 = { case: 'odom 档，无小车位姿（H2 未做）', frame: s.frame, pose: s.pose, mapStatus: s.mapStatus, cloud: cl, robotNote: s.robotNote, position: await page.locator('#metric-position').innerText(), findEnabled, hint };
r1.pass = /^odom/.test(s.frame) && /里程推算/.test(s.pose) && cl.points === 1600 && cl.rev.startsWith('rolling:') && /里程/.test(s.robotNote) && findEnabled && /里程推算/.test(hint);
rows.push(r1); await page.screenshot({ path: `${shots}/a8_odom_norobot.png` });
await c('tier=odomrobot'); await wait(1500);
s = await snap(page); cl = await cloud();
const r2 = { case: 'odom 档，有里程位姿（H2 做了之后）', frame: s.frame, pose: s.pose, cloud: cl, robotNote: s.robotNote, position: await page.locator('#metric-position').innerText() };
r2.pass = /^odom/.test(s.frame) && /里程推算/.test(s.pose) && cl.rev.startsWith('rolling:') && cl.points === 1600 && /odom/.test(r2.position);
rows.push(r2); await page.screenshot({ path: `${shots}/a8_odom_robot.png` });
// 阳性对照：真图
await c('tier=map&points=3000'); await wait(1500);
s = await snap(page); cl = await cloud();
const r3 = { case: '阳性对照：map 档真图', frame: s.frame, pose: s.pose, cloud: cl, mapStatus: s.mapStatus }; r3.controlHolds = cl.points === 3000 && cl.rev.startsWith('global:') && /已定位/.test(s.pose);
rows.push(r3); await page.screenshot({ path: `${shots}/a8_map_control.png` });
for (const r of rows) console.log(JSON.stringify(r));
console.log('A8', !r3.controlHolds ? 'TESTBED-INVALID' : r1.pass && r2.pass ? 'PASS' : 'FAIL', 'pageerrors=', page.errors.length);
await browser.close();
