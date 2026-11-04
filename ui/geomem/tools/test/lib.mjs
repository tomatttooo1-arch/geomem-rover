import { chromium } from 'playwright-core';
export const CHROME = '${FPGA_HOME}/dev1/tmp/chrome/chrome-headless-shell-linux64/chrome-headless-shell';
export const wait = ms => new Promise(r => setTimeout(r, ms));
export const ctl = base => async q => JSON.parse(await (await fetch(`${base}/ctl?${q}`)).text());
export async function launch({ gpu = false, viewport = { width: 1920, height: 1080 }, init } = {}) {
  const args = gpu ? ['--use-angle=vulkan', '--enable-features=Vulkan', '--ignore-gpu-blocklist'] : ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'];
  const browser = await chromium.launch({ executablePath: CHROME, args });
  const ctx = await browser.newContext({ viewport });
  if (init) await ctx.addInitScript(init.fn, init.arg);
  const page = await ctx.newPage();
  page.errors = [];
  page.on('pageerror', e => { page.errors.push(e.message); console.log('pageerror:', e.message); });
  page.on('console', m => { if (m.type() === 'error') console.log('console.error:', m.text()); });
  return { browser, ctx, page };
}
export const snap = page => page.evaluate(() => { const t = id => document.getElementById(id)?.textContent?.trim(); const el = id => document.getElementById(id); return {
  pill: t('connection-pill'), notice: t('source-notice-text'), map: t('map-title'), mapStatus: t('map-status'), frame: t('map-frame'), pose: el('pose-badge').hidden ? '' : t('pose-badge'),
  robotNote: el('robot-note').hidden ? '' : t('robot-note'), banner: el('source-banner').hidden ? '' : t('source-banner'), fresh: t('metric-fresh'), objs: t('metric-objects'), task: t('task-state'),
  msg: t('task-message'), toast: el('toast').hidden ? '' : t('toast'), hint: t('query-hint'), stopDisabled: el('stop-task').disabled, controlsHidden: document.querySelector('.task-controls').hidden,
  pauseInDom: !!el('pause-task'), controlNote: el('control-note').hidden ? '' : t('control-note'), detailOpen: el('detail-dialog').open }; });
export const clearToast = page => page.evaluate(() => { const t = document.getElementById('toast'); t.hidden = true; t.textContent = ''; });
