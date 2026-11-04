// T1（预注册 STAGE1_INTEG_PREREG.md）：真实录制回放＋真桥（python3.8）＋真前端。
// T1-A7 回放标注、T1-cmd 回放拒命令、T1-link 断 hub/换 boot。回放只作工程验证，不是比赛画面。
import { startReplay, startBridge, waitLog, waitHttp, stopPid, stopAll, tokenFile, RawClient, launch, snap, wait, waitPill, hooks, readJson, verdict, out as mk, APP, PY38, REC_DIR } from './tb.mjs';
const [dir, durS = '75'] = process.argv.slice(2); mk(dir);
const tok = tokenFile(dir);
const BANNER = '回放：实车真实录制，非实时', MARK = '回放 · 非实时';
let rp = startReplay(PY38, `${REC_DIR}/d1247_full_1/pc.fpgarec`, tok, dir, 'd1247'); const boot1 = (await waitLog(rp, /boot=(\S+)/))[1];
const br = startBridge(PY38, tok, dir, 't1'); await waitHttp(`${APP}/`);
const c = await new RawClient().open();
const { browser, page } = await launch({ init: hooks() });
await page.goto(`${APP}/?source=live`, { waitUntil: 'networkidle' }); await waitPill(page, /回放/);
const sample = async () => { const s = await snap(page); return { ...s, mark: (await page.locator('#map-watermark').isVisible()) ? (await page.locator('#map-watermark').innerText()).trim() : '',
  findDisabled: await page.evaluate(() => { const b = [...document.querySelectorAll('[data-find]')]; return b.length ? b.every(x => x.disabled) : null; }),
  seen: await page.evaluate(() => [...document.querySelectorAll('.seen')].map(e => e.textContent)) }; };
// 建议按钮只在搜索后出现：先搜一次「杯子」让 [data-find] 进 DOM
await page.fill('#search-input', '杯子'); await page.click('#find-button'); await wait(300);
// ---- T1-A7
const DUR = Number(durS) * 1000, rows = []; const t0 = Date.now();
while (Date.now() - t0 < DUR) { rows.push(await sample()); await wait(500); }
const ok = r => r.banner === BANNER && r.mark === MARK;
const okCount = rows.filter(ok).length, findAll = rows.every(r => r.findDisabled === true), hintOk = rows.every(r => /回放/.test(r.hint)), noPause = rows.every(r => !r.pauseInDom);
const seenAll = [...new Set(rows.flatMap(r => r.seen))], seenOk = !seenAll.some(t => /天前见到/.test(t)) && seenAll.includes('见到时间未知');
const objsOk = rows.every(r => /^[1-9]\d* 件物品$/.test(r.objs));
await page.screenshot({ path: `${dir}/t1_a7_d1247.png` });
console.log(JSON.stringify({ part: 'T1-A7 d1247', samples: rows.length, bannerAndMark: okCount, findDisabledAll: findAll, hintOk, noPause, seenTexts: seenAll.slice(0, 5), objs: [...new Set(rows.map(r => r.objs))], pills: [...new Set(rows.map(r => r.pill))], pageerrors: page.errors.length }));
const a7d = okCount === rows.length && findAll && hintOk && noPause && seenOk && objsOk;
// ---- T1-cmd：hub 在时，原始客户端 find/select/cancel
const last = c.last('snapshot');
const cmds = [await c.command('find', { text: '杯子' }), await c.command('select', { taskId: last.task.id, candidateId: 'object-0001', scope: last.memory.scope }), await c.command('cancel', { taskId: null })];
const cmdRows = cmds.map(x => ({ accepted: x.ack.accepted, code: x.ack.reasonCode, status: x.ack.status, result: x.result?.status ?? null, ackMs: x.ackMs }));
console.log(JSON.stringify({ part: 'T1-cmd', cmds: cmdRows }));
const cmdOk = cmds.every(x => x.ack.accepted === false) && !cmds.some(x => ['accepted', 'done'].includes(x.result?.status));
// ---- T1-link：按 pid 停回放服务器
const before = c.last('snapshot'); const linkBefore = before.link.state, bootBefore = c.last('hello').bridge.hubBootId;
const tStop = Date.now(); await stopPid(rp);
let tDown = null, sawStale = false; while (Date.now() - tStop < 9000) { const s = c.last('snapshot'); if (s.link.state === 'stale') sawStale = true; if (s.link.state === 'down') { tDown = Date.now() - tStop; break; } await wait(100); }
const nDown0 = c.msgs.filter(m => m.type === 'snapshot').length; await wait(1500); const downFrames = c.msgs.filter(m => m.type === 'snapshot').length - nDown0;
const pillDown = (await snap(page)).pill;
const fDown = await c.command('find', { text: '杯子' });
await page.screenshot({ path: `${dir}/t1_link_down.png` });
const nHello = c.msgs.filter(m => m.type === 'hello').length;
rp = startReplay(PY38, `${REC_DIR}/d1247_full_1/pc.fpgarec`, tok, dir, 'd1247b'); const boot2 = (await waitLog(rp, /boot=(\S+)/))[1]; const tUp = Date.now();
let helloNew = null; while (Date.now() - tUp < 12000) { helloNew = c.msgs.filter(m => m.type === 'hello').slice(nHello).find(h => h.bridge.hubBootId === boot2); if (helloNew) break; await wait(100); }
const helloMs = helloNew ? helloNew._t - tUp : null;
let pillBack = null; try { pillBack = await waitPill(page, /^回放 · 非实时$/, 12000); } catch {}
const link = { linkBefore, bootBefore, boot1, sawStale, downAfterMs: tDown, downFrames, pillDown, findWhileDown: fDown.ack.reasonCode, boot2, helloNewMs: helloMs, pillBack };
console.log(JSON.stringify({ part: 'T1-link', ...link }));
const linkCtl = linkBefore === 'up' && bootBefore === boot1;
const linkOk = sawStale && tDown !== null && tDown <= 7000 && downFrames >= 5 && pillDown !== MARK && fDown.ack.reasonCode === 'link_down' && helloMs !== null && helloMs <= 10000 && !!pillBack;
// ---- chain_1314 抽查 20 s
await stopPid(rp); rp = startReplay(PY38, `${REC_DIR}/chain_1314/pc.fpgarec`, tok, dir, 'chain1314'); await waitLog(rp, /replay up/);
await waitPill(page, /^回放 · 非实时$/, 15000); const rows2 = []; const t1 = Date.now();
while (Date.now() - t1 < 20000) { rows2.push(await sample()); await wait(500); }
const a7c = rows2.every(ok) && rows2.every(r => r.findDisabled === true) && rows2.every(r => /^[1-9]\d* 件物品$/.test(r.objs)) && !rows2.flatMap(r => r.seen).some(t => /天前见到/.test(t));
await page.screenshot({ path: `${dir}/t1_a7_chain1314.png` });
console.log(JSON.stringify({ part: 'T1-A7 chain_1314', samples: rows2.length, bannerAndMark: rows2.filter(ok).length, objs: [...new Set(rows2.map(r => r.objs))], seen: [...new Set(rows2.flatMap(r => r.seen))].slice(0, 4) }));
const pe = page.errors.length;
await browser.close(); c.close(); await stopAll();
const st = readJson(`${dir}/bridge_stats_t1.json`);
console.log(JSON.stringify({ bridgeCounts: st.counts }));
verdict('T1-A7', true /* 对照在 t2_a7_source */, a7d && a7c && pe === 0, '(对照见 t2_a7_source 第一段)');
verdict('T1-cmd', link.findWhileDown === 'link_down', cmdOk, `codes=${cmdRows.map(r => r.code).join(',')}`);
verdict('T1-link', linkCtl, linkOk);
