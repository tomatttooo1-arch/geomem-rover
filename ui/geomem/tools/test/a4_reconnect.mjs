// A4：桥 WS 连续断 3 次：每次 ≤ 6 s 回到已连接；断线时待回执的命令显示「结果未知」；重连后桥收到的浏览器命令数 = 0。
// 阳性对照：重连后手动点一次停止，桥收到 1 条。
import { launch, snap, wait, ctl, clearToast } from './lib.mjs';
const [app, shots] = process.argv.slice(2); const c = ctl('http://127.0.0.1:19191');
await c('op=reset&tier=map&points=3000&source=live&link=up&noack=0&reject=0&op=resume_stream');
const { browser, page } = await launch();
await page.goto(`${app}/?source=live&ws=ws://127.0.0.1:19190/ws`, { waitUntil: 'networkidle' }); await wait(2500);
let pass = true; const rows = [];
for (let i = 1; i <= 3; i++) {
  await c('op=reset&noack=1'); await wait(600); await clearToast(page);
  await page.fill('#search-input', '杯子'); await page.click('#find-button'); await wait(200);
  await page.click('[data-find]'); await wait(500);
  const before = await c('op=state'); const sentBeforeDrop = before.cmdThisConn.at(-1);
  const hintBefore = (await snap(page)).hint;
  await c('op=drop'); const t0 = Date.now();
  let toastSeen = '', reconnectMs = null, labels = new Set();
  while (Date.now() - t0 < 8000) { const s = await snap(page); labels.add(s.pill); if (/结果未知/.test(s.toast)) toastSeen ||= s.toast; if (s.pill === '实车已连接' && Date.now() - t0 > 300) { reconnectMs = Date.now() - t0; break; } await wait(100); }
  await c('noack=0'); await wait(3000);
  const after = await c('op=state');
  const r = { drop: i, sentBeforeDrop, hintBefore, pendingShownUnknown: toastSeen, reconnectMs, labelsDuringOutage: [...labels], connections: after.cmdThisConn.length, cmdsOnNewConn: after.cmdThisConn.at(-1) };
  r.pass = sentBeforeDrop === 1 && !!toastSeen && reconnectMs !== null && reconnectMs <= 6000 && after.cmdThisConn.at(-1) === 0; pass &&= r.pass; rows.push(r);
  if (i === 1) await page.screenshot({ path: `${shots}/a4_after_reconnect1.png` });
}
// 阳性对照
await c('op=active'); await wait(1200);
const s0 = await snap(page);
await page.click('#stop-task'); await wait(1200);
const ctrl = await c('op=state'); const s1 = await snap(page);
const ctrlOk = ctrl.cmdThisConn.at(-1) === 1;
rows.push({ control: 'manual stop after reconnect', stopEnabledBefore: !s0.stopDisabled, cmdsOnNewConn: ctrl.cmdThisConn.at(-1), taskAfter: s1.task, toast: s1.toast, controlHolds: ctrlOk });
for (const r of rows) console.log(JSON.stringify(r));
console.log('A4', !ctrlOk ? 'TESTBED-INVALID' : pass ? 'PASS' : 'FAIL', 'pageerrors=', page.errors.length);
await c('op=reset');
await browser.close();
