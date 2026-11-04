// A8 odom 档（真桥）＋定位丢失；对照：定位档＋有占据格真图。预注册 STAGE1_INTEG_PREREG.md。
import { startFixture, startBridge, waitLog, waitHttp, stopAll, tokenFile, RawClient, ctl, launch, snap, wait, waitPill, hooks, cloudInfo, readJson, verdict, out as mk, APP, PY313 } from './tb.mjs';
const [dir] = process.argv.slice(2); mk(dir);
const tok = tokenFile(dir);
const fx = startFixture(PY313, tok, dir, ['--tier', 'odom']); await waitLog(fx, /fixture up/);
startBridge(PY313, tok, dir, 'a8'); await waitHttp(`${APP}/`);
const c = await new RawClient().open();
const { browser, page } = await launch({ init: hooks() });
await page.goto(`${APP}/?source=live`, { waitUntil: 'networkidle' }); await waitPill(page); await wait(2500);
const until = async (pred, ms = 12000) => { const t0 = Date.now(); while (Date.now() - t0 < ms) { if (await pred()) return true; await wait(250); } return false; };
await until(async () => (await cloudInfo(page)).rev.startsWith('rolling:'));
await page.fill('#search-input', '杯子'); await page.click('#find-button'); await wait(300);
let s = await snap(page), cl = await cloudInfo(page), sn = c.last('snapshot');
const odom = { frame: s.frame, pose: s.pose, mapStatus: s.mapStatus, placeholder: sn.map.placeholder, cloudRevSnap: sn.map.cloudRev, cloud: cl, robot: sn.robot, robotNote: s.robotNote,
  findEnabled: await page.evaluate(() => [...document.querySelectorAll('[data-find]')].every(b => !b.disabled)), hint: s.hint };
await page.screenshot({ path: `${dir}/a8_odom.png` });
const odomPass = /^odom/.test(odom.frame) && /里程推算/.test(odom.pose) && /替身地图/.test(odom.mapStatus) && odom.placeholder === true && cl.rev.startsWith('rolling:') && cl.points > 0
  && odom.robot === null && /里程/.test(odom.robotNote) && odom.findEnabled && /里程推算/.test(odom.hint);
// 对照：定位档＋真图
await ctl('op=tier&tier=loc'); await wait(5200); await ctl('op=map&kind=real');
await until(async () => (await cloudInfo(page)).rev.startsWith('global:'), 15000); await wait(800);
s = await snap(page); cl = await cloudInfo(page); sn = c.last('snapshot');
const loc = { frame: s.frame, pose: s.pose, mapStatus: s.mapStatus, placeholder: sn.map.placeholder, cloud: cl, robotPose: sn.robot?.pose };
await page.screenshot({ path: `${dir}/a8_loc_control.png` });
const ctlHolds = cl.rev.startsWith('global:') && cl.points > 0 && /^已定位/.test(loc.pose) && loc.placeholder === false;
// 定位丢失
await ctl('op=tier&tier=lost'); await wait(1500);
await page.fill('#search-input', '杯子'); await page.click('#find-button'); await wait(300);
s = await snap(page); sn = c.last('snapshot');
const lost = { pose: s.pose, level: await page.locator('#pose-badge').getAttribute('data-level'), robotPose: sn.robot?.pose, hint: s.hint,
  findDisabled: await page.evaluate(() => [...document.querySelectorAll('[data-find]')].every(b => b.disabled)) };
await page.screenshot({ path: `${dir}/a8_lost.png` });
const lostPass = /^未定位/.test(lost.pose) && lost.level === 'error' && /尚没有完成定位/.test(lost.hint) && lost.findDisabled;
const pe = page.errors.length; await browser.close(); c.close(); await stopAll();
console.log(JSON.stringify({ odom })); console.log(JSON.stringify({ loc })); console.log(JSON.stringify({ lost }));
console.log(JSON.stringify({ bridgeCounts: readJson(`${dir}/bridge_stats_a8.json`).counts, pageerrors: pe }));
verdict('A8', ctlHolds, odomPass && lostPass && pe === 0);
