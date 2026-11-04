// 展台页判据 K2–K5 ＋外观截图（docs/STAGE1_KIOSK_PREREG.md，10-17 第 3 棒跑前写定）。
//   TB_TOOLS=… GEOMEM_BASEMAP=<底图包目录> sh ui/geomem/bridge/testbed/run_tb.sh kiosk_t2.mjs <输出目录> gpu|swr
import fs from 'node:fs';
import { startFixture, startBridge, stopPid, waitLog, waitHttp, stopAll, tokenFile, ctl, launch, kstate, renderer, until, verdict, wait, APP, PY313, SAMPLE_SHA, LOOK, out } from './kiosk_tb.mjs';

const [dir, arm = 'gpu'] = process.argv.slice(2); out(dir);
const GPU = arm === 'gpu';
const PKG = process.env.GEOMEM_BASEMAP;
const MAN = JSON.parse(fs.readFileSync(`${PKG}/basemap.json`, 'utf8'));
const MEM = JSON.parse(fs.readFileSync(`${LOOK}/memory_final.json`, 'utf8'));
const POSES_F = `${LOOK}/export/cloud_poses.txt`;
const POSES = fs.readFileSync(POSES_F, 'utf8').split('\n').filter(l => l.trim() && !l.startsWith('#')).map(l => l.trim().split(/\s+/).map(Number))
  .sort((a, b) => a[0] - b[0]).map(([t, x, y, z, qx, qy, qz, qw]) => [x, y, Math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))]);
const TARGET = 'object-0007', WRONG = SAMPLE_SHA.slice(0, -1) + (SAMPLE_SHA.at(-1) === '6' ? '7' : '6');
const R = { arm, checks: {}, verdicts: {}, bench: [], notes: [] };
const near = (a, b, tol) => a.length === b.length && a.every((v, i) => Math.abs(v - b[i]) <= tol);
const save = () => fs.writeFileSync(`${dir}/results_${arm}.json`, JSON.stringify(R, null, 1));
const sh = (cmd) => { try { return fs.readFileSync(cmd, 'utf8').trim(); } catch { return null; } };
function judge(name, control, checks, extra = '') { R.checks[name] = { control, ...checks }; const pass = Object.values(checks).every(v => v === true); R.verdicts[name] = verdict(name, control, pass, extra); save(); }

// 独立重算应画钉子（预注册 K2 规则：去 position_invalid；目标第一，其余 lastSeen 新→旧（夹具全部同龄）再按 id；同类 xy < 0.35 m 只留一个）
function expectedPins() {
  const objs = MEM.filter(o => o.state !== 'position_invalid').sort((a, b) => (b.id === TARGET) - (a.id === TARGET) || (a.id < b.id ? -1 : 1));
  const kept = [];
  for (const o of objs) if (o.id === TARGET || !kept.some(k => k.label === o.label && Math.hypot(k.position_m[0] - o.position_m[0], k.position_m[1] - o.position_m[1]) < 0.35)) kept.push(o);
  return kept;
}
// 截图区域像素统计（与某颜色的平均差、亮度标准差），在页内解码 PNG
async function regionStats(page, clip, ref = [5, 7, 11]) {
  const b64 = await page.screenshot({ clip, encoding: 'base64' });
  return page.evaluate(async (b64, ref) => {
    const img = await createImageBitmap(await (await fetch('data:image/png;base64,' + b64)).blob());
    const c = new OffscreenCanvas(img.width, img.height), g = c.getContext('2d'); g.drawImage(img, 0, 0);
    const d = g.getImageData(0, 0, img.width, img.height).data; let n = 0, diff = 0, s = 0, s2 = 0;
    for (let i = 0; i < d.length; i += 4) { const L = (d[i] + d[i + 1] + d[i + 2]) / 3; diff += (Math.abs(d[i] - ref[0]) + Math.abs(d[i + 1] - ref[1]) + Math.abs(d[i + 2] - ref[2])) / 3; s += L; s2 += L * L; n++; }
    return { meanDiff: diff / n, std: Math.sqrt(Math.max(0, s2 / n - (s / n) ** 2)), px: n };
  }, b64, ref);
}
const loadInfo = () => ({ loadavg: sh('/proc/loadavg') });

// ------------------------------------------------------------------ scene
const tok = tokenFile(dir);
const fx = startFixture(PY313, tok, dir); await waitLog(fx, /fixture up/);
await ctl(`op=objects&file=${LOOK}/memory_final.json&age=120`);
await ctl(`op=walk&file=${POSES_F}&upto=0&dt=0.05`);
await ctl('op=kv&geo=12&rows=8&hb=1&roi_age=200');
await ctl(`op=task&state=navigating&reason=reobserve&target=${TARGET}`);
let br = startBridge(PY313, tok, dir, `${arm}_ok`, ['--map-id', SAMPLE_SHA, '--basemap', PKG]); await waitHttp(`${APP}/kiosk.html`);
const { browser, page } = await launch({ gpu: GPU });
page.setDefaultNavigationTimeout(180000);   // 仪器时限：SwiftShader 下旧页一帧 5–9 s，卸载换页超过 puppeteer 默认 30 s（第 2 趟 swr FATAL）
R.renderer = await renderer(page);
const armOk = GPU ? /NVIDIA/.test(R.renderer || '') : /SwiftShader/.test(R.renderer || '');
console.log('renderer', R.renderer, 'armOk', armOk);
const globalOk = () => page.errors.length === 0;
const OV = { cam: [0.4, 5.4, 5.6], tgt: [3.0, 0, -0.35] };

try {
  // ================================================================ K2
  await page.goto(`${APP}/kiosk.html`);
  await until(page, s => s.connection === 'connected' && +s.ds.basemapCells > 0, 120000, '底图载入');
  await ctl(`op=walk&file=${POSES_F}&upto=16&dt=0.2`);
  const p16 = POSES[16];
  await until(page, s => s.car && near(s.car.slice(0, 2), p16.slice(0, 2), 0.005), 30000, '小车到第 16 点');
  await wait(2000);
  const s2 = await kstate(page); R.k2state = s2;
  await page.screenshot({ path: `${dir}/kiosk_${arm}.png` });
  const exp = expectedPins(), expIds = exp.map(o => o.id).sort(), invalidIds = new Set(MEM.filter(o => o.state === 'position_invalid').map(o => o.id));
  const tgt = MEM.find(o => o.id === TARGET), N = s2.pinCls.bottle ?? 0, NExp = exp.filter(o => o.label === 'bottle').length;
  const D = Math.hypot(tgt.position_m[0] - s2.car[0], tgt.position_m[1] - s2.car[1]).toFixed(1);
  const g2 = globalOk() && s2.connection === 'connected' && s2.seq !== null;
  judge('K2', armOk && g2, {
    basemap: s2.basemapStatus === 'ok' && +s2.ds.basemapCells === 259494 && MAN.files.cells.count === 259494 && s2.ds.basemapSha === SAMPLE_SHA,
    overview: near(s2.camera.slice(0, 3), OV.cam, 1e-3) && near(s2.target, OV.tgt, 1e-3) && s2.camera[3] === 42 && s2.camMode === 'overview',
    car: near(s2.car.slice(0, 2), p16.slice(0, 2), 0.005) && s2.trail >= 10,
    pins: JSON.stringify([...s2.pinIds].sort()) === JSON.stringify(expIds) && !s2.pinIds.some(i => invalidIds.has(i)) && s2.beams === 1 && s2.labels === 1 + Math.min(5, s2.pins - 1),
    status: s2.statusK === '寻物任务进行中' && s2.statusV === '正在寻找：水瓶' && s2.statusS === `记忆中有 ${N} 个水瓶 · 距小车 ${D} m · 正在前往` && N === NExp,
    fpga: s2.geo.v === '12 ms' && s2.roi.k === '本帧核对记忆物体' && s2.roi.v === '8 个' && s2.hb.v === '正常' && /\bok\b/.test(s2.hb.cls) && s2.ar.hidden === true,
    note: s2.basemapNote === '底图：20261015_190638.db · 底图校验依据：操作员配置' && s2.banner === '',
    fpHidden: s2.fpVisible === false,
  }, `N=${N} NExp=${NExp} pins=${s2.pins} expPins=${exp.length} D=${D} statusS=${s2.statusS}`);

  // ================================================================ K4 bench（有底图）
  R.benchEnv = { before: loadInfo() };
  const REPS = Number(process.env.KIOSK_BENCH_REPS || 3); R.benchReps = REPS;
  for (let i = 0; i < REPS; i++) { const b = await page.evaluate(() => window.__kiosk.scene.bench(40)); R.bench.push(b); console.log('bench', JSON.stringify(b)); save(); }
  R.benchEnv.after = loadInfo();

  // ================================================================ K5 实时层与按钮
  { // a 换点
    const before = await kstate(page), p30 = POSES[30];
    await ctl(`op=pose&x=${p30[0]}&y=${p30[1]}&yaw=${p30[2]}`);
    const r = await until(page, s => s.car && near(s.car.slice(0, 2), p30.slice(0, 2), 0.005), 5000, '小车到第 30 点').catch(e => ({ err: String(e) }));
    const st = r.st || await kstate(page);
    judge('K5a', near(before.car.slice(0, 2), p16.slice(0, 2), 0.005), { moved: !r.err, trailGrew: st.trail >= before.trail + 1 }, `trail ${before.trail}->${st.trail} ms=${r.ms}`);
  }
  { // b 心跳
    const before = await kstate(page);
    await ctl('op=kv&geo=12&rows=8&hb=0&roi_age=200');
    const r = await until(page, s => s.hb.v === '断开' && /\bbad\b/.test(s.hb.cls), 5000, '心跳断开').catch(e => ({ err: String(e) }));
    judge('K5b', before.hb.v === '正常' && /\bok\b/.test(before.hb.cls), { hbOff: !r.err }, `ms=${r.ms}`);
    await ctl('op=kv&geo=12&rows=8&hb=1&roi_age=200');
  }
  { // c ROI 年龄
    await ctl('op=kv&geo=12&rows=8&hb=1&roi_age=1500');
    const r = await until(page, s => s.roi.k === '最近一次核对物体' && s.roi.v === '—', 5000, 'ROI 变旧').catch(e => ({ err: String(e) }));
    await ctl('op=kv&geo=12&rows=8&hb=1&roi_age=200');
    const r2 = await until(page, s => s.roi.k === '本帧核对记忆物体' && s.roi.v === '8 个', 5000, 'ROI 恢复').catch(e => ({ err: String(e) }));
    judge('K5c', !r2.err, { stale: !r.err }, `ms=${r.ms} restore_ms=${r2.ms}`);
  }
  { // d 第一视角
    await page.click('#toggle button[data-view=first]');
    const r = await until(page, s => s.view === 'first' && s.fpVisible, 5000, '进第一视角').catch(e => ({ err: String(e) }));
    await wait(1500);
    const st = await kstate(page);
    await page.screenshot({ path: `${dir}/look_first_${arm}.png` });
    const mr = await page.evaluate(() => { const b = document.getElementById('fp-mini').getBoundingClientRect(); return { x: b.left + 4, y: b.top + 30, width: b.width - 8, height: b.height - 34 }; });
    const mini = await regionStats(page, mr), bg = await regionStats(page, { x: 0, y: 300, width: 200, height: 400 });
    R.k5dStats = { mini, bg, miniRect: mr };
    const fpgaVis = await page.evaluate(() => getComputedStyle(document.getElementById('fpga')).display !== 'none');
    await page.click('#toggle button[data-view=third]');
    const back = await until(page, s => s.view === 'third' && !s.fpVisible, 5000, '回第三视角').catch(e => ({ err: String(e) }));
    judge('K5d', bg.meanDiff <= 20 && bg.std <= 10 && !back.err, {
      first: !r.err && st.fpText === '车载画面未接入' && st.statusV === '车载画面未接入' && !fpgaVis && !st.camsVisible,
      miniDrawn: mini.meanDiff > 20 && mini.std > 10,
    }, `mini=${JSON.stringify(mini)} bg=${JSON.stringify(bg)}`);
  }
  const followOk = s => { if (!s.car) return false; const [x, y, yaw] = s.car, fx = Math.cos(yaw), fz = -Math.sin(yaw);
    const p = [x - 1.1 * fx, 0.85, -y - 1.1 * fz], t = [x + 1.5 * fx, 0.05, -y + 1.5 * fz];
    return s.camMode === 'follow' && s.camera[3] === 55 && near(s.camera.slice(0, 3), p, 0.01) && near(s.target, t, 0.01); };
  { // e 跟随
    const before = await kstate(page);
    await page.click('#cam-follow');
    const r = await until(page, followOk, 5000, '跟随').catch(e => ({ err: String(e) }));
    const p16 = POSES[16]; await ctl(`op=pose&x=${p16[0]}&y=${p16[1]}&yaw=${p16[2]}`);
    const r2 = await until(page, s => s.car && near(s.car.slice(0, 2), p16.slice(0, 2), 0.005) && followOk(s), 5000, '跟随到新车位').catch(e => ({ err: String(e) }));
    await page.screenshot({ path: `${dir}/look_follow_${arm}.png` });
    judge('K5e', before.camMode === 'overview', { follow: !r.err, tracks: !r2.err }, `ms=${r.ms}/${r2.ms}`);
  }
  { // f 回到全景
    const before = await kstate(page);
    await page.click('#cam-overview');
    const r = await until(page, s => s.camMode === 'overview' && near(s.camera.slice(0, 3), OV.cam, 1e-3) && near(s.target, OV.tgt, 1e-3) && s.camera[3] === 42, 5000, '回全景').catch(e => ({ err: String(e) }));
    judge('K5f', Math.hypot(...before.camera.slice(0, 3).map((v, i) => v - OV.cam[i])) > 0.5, { overview: !r.err }, `ms=${r.ms}`);
  }
  { // g 拖动
    const before = await kstate(page);
    await page.mouse.move(960, 600); await page.mouse.down(); await page.mouse.move(1160, 600, { steps: 12 }); await page.mouse.up();
    const r = await until(page, s => s.camMode === 'free' && Math.hypot(...s.camera.slice(0, 3).map((v, i) => v - OV.cam[i])) > 0.05, 5000, '拖动').catch(e => ({ err: String(e) }));
    judge('K5g', before.camMode === 'overview', { free: !r.err }, `ms=${r.ms}`);
    await page.click('#cam-overview');
  }

  // ================================================================ 外观截图（只 4090 臂）
  if (GPU) {
    for (const [tag, q] of [['default', ''], ['cams_above', 'cams=above'], ['labels0', 'labels=0'], ['labels30', 'labels=30'], ['invalid_show', 'invalid=show']]) {
      await page.goto(`${APP}/kiosk.html${q ? '?' + q : ''}`);
      await until(page, s => s.connection === 'connected' && +s.ds.basemapCells > 0 && s.car, 120000, `外观 ${tag}`);
      await wait(1500); const st = await kstate(page);
      R.notes.push({ look: tag, pins: st.pins, labels: st.labels, pinCls: st.pinCls, statusS: st.statusS });
      await page.screenshot({ path: `${dir}/look_${tag}.png` });
    }
    await page.goto(`${APP}/kiosk.html`); await until(page, s => s.connection === 'connected' && +s.ds.basemapCells > 0, 120000, '回默认');
  }

  // ================================================================ K3b 里程档
  // 10-18 owner 方案改为里程档＋id 对也画底图（basis mapping_start）：本条按 10-17 语义已失效，不再跑；新判据见 kiosk_t3.mjs / STAGE2_KIOSK_ODOM_PREREG.md
  {
    await ctl('op=tier&tier=odom');
    const r = await until(page, s => s.basemapStatus === 'not_localized' && +s.ds.basemapCells === 0 && s.basemapNote === '未加载底图（仅里程计定位）', 10000, '里程档').catch(e => ({ err: String(e) }));
    if (GPU) await page.screenshot({ path: `${dir}/look_odom_${arm}.png` });
    await ctl('op=tier&tier=loc');
    const r2 = await until(page, s => +s.ds.basemapCells === 259494, 30000, '恢复定位档').catch(e => ({ err: String(e) }));
    judge('K3b', !r2.err, { odom: !r.err }, `ms=${r.ms} restore_ms=${r2.ms} note=${(await kstate(page)).basemapNote}`);
  }

  // ================================================================ K3a 地图 id 错一位（＋K4 无底图 bench 对照）
  {
    await stopPid(br);
    br = startBridge(PY313, tok, dir, `${arm}_wrong`, ['--map-id', WRONG, '--basemap', PKG]); await waitHttp(`${APP}/kiosk.html`);
    await page.goto(`${APP}/kiosk.html`);
    const r = await until(page, s => s.connection === 'connected' && s.basemapStatus === 'mismatch' && +s.ds.cloudPoints > 0, 60000, 'mismatch').catch(e => ({ err: String(e) }));
    await wait(2000); const st = await kstate(page); R.k3aState = st;
    await page.screenshot({ path: `${dir}/neg_mapid_${arm}.png` });
    judge('K3a', globalOk(), { mismatch: !r.err && st.basemapStatus === 'mismatch' && +st.ds.basemapCells === 0 && st.basemapNote === '未加载底图（当前地图与底图包不符）' && +st.ds.cloudPoints > 0 }, `wrong=${WRONG} note=${st.basemapNote} cloud=${st.ds.cloudPoints}`);
    R.benchNoBasemap = await page.evaluate(() => window.__kiosk.scene.bench(40)); console.log('bench_nobasemap', JSON.stringify(R.benchNoBasemap));
  }
  // ================================================================ K4 判定
  {
    const med = a => a.slice().sort((x, y) => x - y)[Math.floor(a.length / 2)];
    const dm = med(R.bench.map(b => b.drawMs)), ds = med(R.bench.map(b => b.drawWithShadowMs));
    R.benchMedian = { drawMs: dm, drawWithShadowMs: ds };
    const control = R.benchNoBasemap.instances === 0 && R.benchNoBasemap.drawMs < dm;
    const checks = { instances: R.bench.every(b => b.instances === 259494) };
    if (GPU) checks.under16_7 = dm <= 16.7;
    judge('K4', control && armOk, checks, `drawMs med=${dm.toFixed(3)} shadow med=${ds.toFixed(3)} noBasemap=${R.benchNoBasemap.drawMs.toFixed(3)}`);
  }
  R.pageErrors = page.errors; R.consoleErrors = page.consoleErrors;
  R.verdicts.global = page.errors.length === 0 ? 'OK' : 'PAGEERROR'; save();
} catch (e) {
  R.fatal = String(e && e.stack || e); save(); console.log('FATAL', R.fatal); process.exitCode = 1;
} finally {
  await browser.close(); await stopAll();
}
console.log('VERDICTS', JSON.stringify(R.verdicts));
