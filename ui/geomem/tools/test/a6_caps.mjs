// A6：capabilities.pause=false 时 DOM 里没有暂停按钮；control=observer 时找物/选择/停止全部置灰并有说明。阳性对照：pause 置真，按钮出现。
import { launch, snap, wait, ctl } from './lib.mjs';
const [app, shots] = process.argv.slice(2); const c = ctl('http://127.0.0.1:19191');
const caps = o => c('caps=' + encodeURIComponent(JSON.stringify(o)));
await c('op=reset&tier=map&points=3000&source=live&link=up&noack=0&reject=0&op=resume_stream');
await caps({ pause: false, resume: false, control: 'controller' });
const { browser, page } = await launch();
await page.goto(`${app}/?source=live&ws=ws://127.0.0.1:19190/ws`, { waitUntil: 'networkidle' }); await wait(2500);
const rows = []; let pass = true;
// 1. pause=false，有进行中的任务
await c('op=active'); await wait(1200);
let s = await snap(page);
const r1 = { case: 'pause=false, 任务进行中', pauseInDom: s.pauseInDom, stopVisible: !s.controlsHidden, task: s.task }; r1.pass = !s.pauseInDom && !s.controlsHidden; rows.push(r1);
await page.screenshot({ path: `${shots}/a6_pause_false.png` });
// 2. observer：先进入 ambiguous，再切 observer
await page.click('#stop-task'); await wait(800); await c('op=reset'); await wait(600);
await page.fill('#search-input', '杯子'); await page.click('#find-button'); await wait(200); await page.click('[data-find]'); await wait(1200);
await caps({ control: 'observer' }); await wait(800);
await page.fill('#search-input', '书'); await page.click('#find-button'); await wait(300);
s = await snap(page);
const dis = await page.evaluate(() => ({ find: [...document.querySelectorAll('[data-find]')].map(b => b.disabled), select: [...document.querySelectorAll('[data-candidate]')].map(b => b.disabled), stop: document.getElementById('stop-task').disabled }));
await page.click('[data-object="object-0003"]'); await wait(500);
const startDisabled = await page.locator('#start-task').isDisabled(); const startHint = await page.locator('#start-hint').innerText();
await page.screenshot({ path: `${shots}/a6_observer_dialog.png` });
await page.keyboard.press('Escape'); await wait(400);
await page.screenshot({ path: `${shots}/a6_observer.png` });
const r2 = { case: 'observer', task: s.task, findDisabled: dis.find, selectDisabled: dis.select, stopDisabled: dis.stop, startTaskDisabled: startDisabled, startHint, controlNote: s.controlNote, queryHint: s.hint };
r2.pass = dis.find.length > 0 && dis.find.every(Boolean) && dis.select.length === 2 && dis.select.every(Boolean) && dis.stop && startDisabled && /控制权/.test(s.controlNote); rows.push(r2);
// 观察者点击被禁用的按钮不会发命令
const cmdBefore = (await c('op=state')).cmdTotal; await page.evaluate(() => document.querySelector('[data-candidate]')?.click()); await wait(500);
rows.push({ case: 'observer 强点候选', cmdsSent: (await c('op=state')).cmdTotal - cmdBefore });
// 3. 阳性对照：pause=true
await caps({ control: 'controller', pause: true, resume: true }); await wait(500);
await page.click('#stop-task'); await wait(800); await c('op=active'); await wait(1200);
s = await snap(page);
const r3 = { case: '阳性对照 pause=true', pauseInDom: s.pauseInDom, pauseText: await page.locator('#pause-task').innerText().catch(() => null) }; r3.controlHolds = s.pauseInDom; rows.push(r3);
await page.screenshot({ path: `${shots}/a6_pause_true_control.png` });
for (const r of rows) console.log(JSON.stringify(r));
pass = rows.filter(r => 'pass' in r).every(r => r.pass) && rows[2].cmdsSent === 0;
console.log('A6', !r3.controlHolds ? 'TESTBED-INVALID' : pass ? 'PASS' : 'FAIL', 'pageerrors=', page.errors.length);
await caps({ pause: false, resume: false, control: 'controller' }); await c('op=reset');
await browser.close();
