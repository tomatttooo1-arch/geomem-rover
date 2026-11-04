// A9：中途换 runtime_boot_id：选中项清掉、详情框关闭；带旧 scope 的命令被桥拒并提示「记忆已重置」。阳性对照：scope 不变时同一命令被转发并受理。
// 按控制器补充：scope 随 select 发送（find 为按文字找，不带 scope）；所以「旧 scope 的命令」用 select 验。
import { launch, snap, wait, ctl, clearToast } from './lib.mjs';
const [app, shots] = process.argv.slice(2); const c = ctl('http://127.0.0.1:19191');
await c('op=reset&tier=map&points=3000&source=live&link=up&noack=0&reject=0&op=resume_stream&caps=' + encodeURIComponent(JSON.stringify({ pause: false, control: 'controller' })));
const { browser, page } = await launch();
await page.goto(`${app}/?source=live&ws=ws://127.0.0.1:19190/ws`, { waitUntil: 'networkidle' }); await wait(2500);
const rows = [];
// 1. 选中物体、打开详情，再换 scope
await page.click('[data-object="object-0003"]'); await wait(600);
const before = { detailOpen: (await snap(page)).detailOpen, selectedLabels: await page.locator('.object-label.selected').count() };
const scopeOld = (await c('op=state')).scope;
await clearToast(page); await c('op=scope'); await wait(1000);
const s1 = await snap(page);
const after = { detailOpen: s1.detailOpen, selectedLabels: await page.locator('.object-label.selected').count(), toast: s1.toast };
const r1 = { case: 'scope 变化清选择、关详情', scopeOld, scopeNew: (await c('op=state')).scope, before, after };
r1.pass = before.detailOpen && before.selectedLabels === 1 && !after.detailOpen && after.selectedLabels === 0 && /记忆已重置/.test(after.toast); rows.push(r1);
await page.screenshot({ path: `${shots}/a9_after_scope_change.png` });
// 2. 进入 ambiguous；冻结快照流后桥换 scope（前端还持旧 scope），点候选 → 桥拒 scope_changed
await page.fill('#search-input', '杯子'); await page.click('#find-button'); await wait(200); await page.click('[data-find]'); await wait(1200);
await c('op=pause_stream'); await c('op=scope'); await wait(300); await clearToast(page);
const n0 = (await c('op=state')).cmdTotal;
await page.click('[data-candidate="object-0002"]'); await wait(800);
const s2 = await snap(page); const st2 = await c('op=state');
const r2 = { case: '旧 scope 的 select', forwardedToBridge: st2.cmdTotal - n0, toast: s2.toast, bridgeTask: st2.task?.state };
r2.pass = r2.forwardedToBridge === 1 && /记忆已重置/.test(s2.toast) && st2.task?.state === 'ambiguous'; rows.push(r2);
await page.screenshot({ path: `${shots}/a9_stale_scope_rejected.png` });
// 3. 阳性对照：快照恢复（前端拿到新 scope），scope 不变，同一 select 被转发并受理
await c('op=resume_stream'); await wait(1200); await clearToast(page);
const n1 = (await c('op=state')).cmdTotal;
await page.click('[data-candidate="object-0002"]'); await wait(1200);
const s3 = await snap(page); const st3 = await c('op=state');
const r3 = { case: '阳性对照：scope 不变时 select', forwardedToBridge: st3.cmdTotal - n1, toast: s3.toast, task: s3.task, bridgeTarget: st3.task?.targetId };
r3.controlHolds = r3.forwardedToBridge === 1 && st3.task?.targetId === 'object-0002' && !/记忆已重置/.test(s3.toast); rows.push(r3);
await page.click('#stop-task'); await wait(600);
// 4. 任务记录降级为纯文本
await page.click('[data-page="history"]'); await wait(700);
const hist = await page.evaluate(() => ({ text: document.getElementById('history-list').innerText.split('\n').slice(0, 4).join(' / '), clickable: document.querySelectorAll('#history-list button, #history-list a').length }));
rows.push({ case: '任务记录（纯文本，带作用域前 8 位）', ...hist });
for (const r of rows) console.log(JSON.stringify(r));
console.log('A9', !r3.controlHolds ? 'TESTBED-INVALID' : r1.pass && r2.pass && hist.clickable === 0 ? 'PASS' : 'FAIL', 'pageerrors=', page.errors.length);
await c('op=reset');
await browser.close();
