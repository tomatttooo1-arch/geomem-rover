// A6 能力隐藏（真桥）：observer 只读；对照①让出控制权 → controller；②页内注入 pause:true 的 hello → 暂停按钮进 DOM。
import { startFixture, startBridge, waitLog, waitHttp, stopAll, tokenFile, RawClient, ctl, launch, snap, wait, waitPill, hooks, inject, readJson, verdict, out as mk, APP, PY313 } from './tb.mjs';
const [dir] = process.argv.slice(2); mk(dir);
const tok = tokenFile(dir);
const fx = startFixture(PY313, tok, dir); await waitLog(fx, /fixture up/);
const sq = await ctl('op=squat');                    // 占控制权的 FPGAC 客户端先连
startBridge(PY313, tok, dir, 'a6'); await waitHttp(`${APP}/`);
const c = await new RawClient().open();
await ctl('op=task&state=ambiguous&reason=candidate_selection_required&cands=cup');
const { browser, page } = await launch({ init: hooks() });
await page.goto(`${APP}/?source=live`, { waitUntil: 'networkidle' }); await waitPill(page); await wait(1500);
await page.fill('#search-input', '杯子'); await page.click('#find-button'); await wait(300);
const state = async () => ({ ...(await snap(page)), control: c.last('hello')?.capabilities.control,
  findDis: await page.evaluate(() => [...document.querySelectorAll('[data-find]')].map(b => b.disabled)),
  candDis: await page.evaluate(() => [...document.querySelectorAll('[data-candidate]')].map(b => b.disabled)) });
const s1 = await state();
await page.screenshot({ path: `${dir}/a6_observer.png` });
await page.evaluate(() => document.querySelector('[data-candidate]')?.click()); await wait(1500);
const pass = s1.control === 'observer' && s1.findDis.length > 0 && s1.findDis.every(Boolean) && s1.candDis.length === 2 && s1.candDis.every(Boolean)
  && s1.stopDisabled && /另一台电脑持有控制权/.test(s1.controlNote) && !s1.pauseInDom;
// 对照①：占位客户端让出控制权
const y = await ctl('op=yield'); const ty = Date.now(); let s2 = null;
while (Date.now() - ty < 3000) { s2 = await state(); if (s2.control === 'controller' && s2.candDis.length && s2.candDis.every(b => !b) && !s2.stopDisabled) break; await wait(150); }
const yieldMs = Date.now() - ty;
await page.screenshot({ path: `${dir}/a6_controller.png` });
const ctl1 = s2.control === 'controller' && s2.candDis.every(b => !b) && !s2.stopDisabled && yieldMs <= 3000;
// 对照②：注入 pause:true 的 hello
const h = { ...c.last('hello'), capabilities: { ...c.last('hello').capabilities, pause: true } }; delete h._t;
await inject(page, JSON.stringify(h)); await ctl('op=task&state=navigating&reason=reobserve&target=object-0001'); await wait(1500);
const pauseIn = (await snap(page)).pauseInDom;
await page.screenshot({ path: `${dir}/a6_pause_injected.png` });
const pe = page.errors.length; await browser.close(); c.close(); await stopAll();
const st = readJson(`${dir}/bridge_stats_a6.json`).counts;
console.log(JSON.stringify({ observer: { control: s1.control, findDis: s1.findDis, candDis: s1.candDis, stopDisabled: s1.stopDisabled, controlNote: s1.controlNote, pauseInDom: s1.pauseInDom }, squatController: sq.controller }));
console.log(JSON.stringify({ yield: y, yieldMs, controller: { control: s2.control, candDis: s2.candDis, stopDisabled: s2.stopDisabled }, pauseInjectedInDom: pauseIn, bridgeBrowserCmd: st.browser_cmd || 0, pageerrors: pe }));
verdict('A6', ctl1 && pauseIn, pass && (st.browser_cmd || 0) === 0 && pe === 0);
