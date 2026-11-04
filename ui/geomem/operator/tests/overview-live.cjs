// Actual read-only workbench observation. No drive, navigation or map-stop commands.
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'C:/Users/lenovo/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
(async()=>{
  const browser=await chromium.launch({headless:true,executablePath:process.env.BROWSER_PATH||'C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe'});
  try{
    const page=await browser.newPage({viewport:{width:1440,height:900}}),commands=[],errors=[];
    page.on('pageerror',e=>errors.push(e.message));
    page.on('websocket',ws=>ws.on('framesent',f=>{try{const m=JSON.parse(String(f.payload));if(m.type==='command')commands.push(m.action);}catch{}}));
    await page.goto('http://127.0.0.1:8788/#overview');
    await page.waitForFunction(()=>window.__overviewMap?.mapping?.count>0,null,{timeout:45000});
    await page.waitForTimeout(1200);
    const out=path.resolve(process.env.GEOMEM_OBSERVATION_DIR||path.resolve(__dirname,'../artifacts/competition-live'));fs.mkdirSync(out,{recursive:true});
    const read=()=>page.evaluate(()=>({mode:window.__overviewMap.mode,meta:window.__overviewMap.mapping,visible:window.__overviewMap.visibleCount,status:document.querySelector('#map-status').textContent,viewport:[innerWidth,innerHeight],document:[document.documentElement.scrollWidth,document.documentElement.scrollHeight]}));
    const global=await read();assert.equal(global.mode,'global');assert.equal(global.visible,global.meta.count);assert.equal(global.meta.source,'rtabmap');assert.deepEqual(global.viewport,global.document);
    await page.screenshot({path:path.join(out,'overview-global-desktop.png'),fullPage:true});
    await page.locator('#local-view').click();await page.waitForTimeout(1200);
    const local=await read();assert.equal(local.mode,'local');assert(local.visible>0&&local.visible<=local.meta.count);
    await page.screenshot({path:path.join(out,'overview-local-desktop.png'),fullPage:true});
    await page.waitForFunction(meta=>window.__overviewMap.mapping?.revision>meta.revision||window.__overviewMap.mapping?.cloudAgeMs>meta.cloudAgeMs+1000,local.meta,{timeout:15000});
    if((await read()).meta.cached)assert((await read()).status.includes('缓存'));
    await page.locator('#layers-button').click();await page.locator('#layer-geometry').uncheck();assert.equal(await page.evaluate(()=>window.__overviewMap.geometryVisible),false);
    await page.locator('#layer-geometry').check();await page.locator('#layers-button').click();
    await page.setViewportSize({width:1280,height:720});await page.waitForTimeout(1000);
    const laptop=await read();assert.deepEqual(laptop.viewport,laptop.document);
    await page.screenshot({path:path.join(out,'overview-local-laptop.png'),fullPage:true});
    await page.locator('#global-view').click();await page.waitForTimeout(1200);
    await page.screenshot({path:path.join(out,'overview-global-laptop.png'),fullPage:true});
    const restored=await read();assert.equal(restored.mode,'global');assert.equal(restored.visible,restored.meta.count);
    await page.locator('#overview-memory').click();await page.waitForFunction(()=>!document.querySelector('#memory-section').hidden);
    assert.deepEqual(commands,[]);assert.deepEqual(errors,[]);
    const result={global,local,laptop,restored,sentCommands:commands,pageErrors:errors};fs.writeFileSync(path.join(out,'overview-live-observation.json'),JSON.stringify(result,null,2));
    console.log(JSON.stringify({global:global.visible,local:local.visible,laptop:laptop.visible,restored:restored.visible,source:global.meta.source,commands,pageErrors:errors},null,2));
    console.log('PASS: actual RTAB-Map in workbench; global/local switching, ongoing updates, layers, all objects, desktop/laptop no scroll; zero commands.');
  }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1});
