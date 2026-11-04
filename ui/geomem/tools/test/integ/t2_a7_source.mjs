// A7 来源不一致（T2）＋托管检查＋T1-A7 的对照（live 帧时横幅/水印隐藏）。预注册 STAGE1_INTEG_PREREG.md。
import { startFixture, startBridge, waitLog, waitHttp, stopAll, tokenFile, RawClient, ctl, launch, snap, wait, waitPill, hooks, cloudInfo, readJson, verdict, out as mk, APP, PY313 } from './tb.mjs';
const [dir] = process.argv.slice(2); mk(dir);
const tok = tokenFile(dir);
const fx = startFixture(PY313, tok, dir); await waitLog(fx, /fixture up/);
startBridge(PY313, tok, dir, 'a7'); await waitHttp(`${APP}/`);
const c = await new RawClient().open();
const { browser, page } = await launch({ init: hooks() });
const failed = []; page.on('requestfailed', r => failed.push(r.url())); page.on('response', r => { if (r.status() >= 400) failed.push(`${r.status()} ${r.url()}`); });
await page.goto(`${APP}/?source=live`, { waitUntil: 'networkidle' }); await waitPill(page); await wait(1500);
const markVis = async () => (await page.locator('#map-watermark').isVisible()) ? (await page.locator('#map-watermark').innerText()).trim() : '';
// 托管＋对照：live
const live = { ...(await snap(page)), mark: await markVis(), cloud: await cloudInfo(page), failed: [...failed] };
await page.screenshot({ path: `${dir}/a7_live_control.png` });
const hostingOk = live.failed.length === 0 && live.cloud.points > 0;
const ctlLive = live.banner === '' && live.mark === '' && c.last('snapshot').source === 'live';
// inconsistent：先喂 ambiguous 看候选，再喂 navigating 看停止
await ctl('op=source&sensor=replay'); await ctl('op=task&state=ambiguous&reason=candidate_selection_required&cands=cup'); await wait(2000);
await page.fill('#search-input', '杯子'); await page.click('#find-button'); await wait(300);
const amb = { ...(await snap(page)), mark: await markVis(), level: await page.locator('#source-banner').getAttribute('data-level'),
  findDis: await page.evaluate(() => [...document.querySelectorAll('[data-find]')].map(b => b.disabled)),
  candDis: await page.evaluate(() => [...document.querySelectorAll('[data-candidate]')].map(b => b.disabled)), src: c.last('snapshot').source };
await page.screenshot({ path: `${dir}/a7_inconsistent_ambiguous.png` });
const rf = await c.command('find', { text: '杯子' });
await ctl('op=task&state=navigating&reason=reobserve&target=object-0002'); await wait(1500);
const nav = { ...(await snap(page)), mark: await markVis() };
await ctl('op=clear'); await page.click('#stop-task'); await wait(3000);
const st = await ctl('op=state'); const after = await snap(page);
await page.screenshot({ path: `${dir}/a7_inconsistent_stopped.png` });
const pe = page.errors.length; await browser.close(); c.close(); await stopAll();
const bc = readJson(`${dir}/bridge_stats_a7.json`).counts;
console.log(JSON.stringify({ live: { banner: live.banner, mark: live.mark, pill: live.pill, cloud: live.cloud, failed: live.failed } }));
console.log(JSON.stringify({ inconsistentAmbiguous: { src: amb.src, banner: amb.banner, level: amb.level, mark: amb.mark, findDis: amb.findDis, candDis: amb.candDis, stopDisabled: amb.stopDisabled }, rawFind: rf.ack.reasonCode }));
console.log(JSON.stringify({ inconsistentNavigating: { banner: nav.banner, stopDisabled: nav.stopDisabled, task: nav.task }, afterStop: { published: st.published, task: after.task, toast: after.toast }, bridgeCounts: bc, pageerrors: pe }));
const pass = amb.src === 'inconsistent' && /来源不一致/.test(amb.banner) && amb.level === 'error' && amb.mark === '来源不一致' && amb.findDis.length > 0 && amb.findDis.every(Boolean)
  && amb.candDis.length === 2 && amb.candDis.every(Boolean) && rf.ack.reasonCode === 'source_inconsistent' && nav.stopDisabled === false
  && st.published.some(p => p[0] === '/fpga/cancel') && (bc['result:done'] || 0) >= 1 && pe === 0;
verdict('A7-T2', ctlLive, pass); verdict('托管', true, hostingOk);
