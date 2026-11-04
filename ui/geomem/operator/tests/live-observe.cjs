// Read-only observation of the actual bridge. Never sends a vehicle command.
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'C:/Users/lenovo/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
const assert=require('node:assert/strict'),path=require('node:path'),fs=require('node:fs');
(async()=>{
  const browser=await chromium.launch({headless:true,executablePath:process.env.BROWSER_PATH||'C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe'});
  try {
    const page=await browser.newPage({viewport:{width:1440,height:900}}),sent=[],errors=[];
    page.on('pageerror',e=>errors.push(e.message));
    page.on('websocket',socket=>socket.on('framesent',frame=>{try{const msg=JSON.parse(String(frame.payload));if(msg.type==='command')sent.push(msg.action);}catch{}}));
    await page.goto('http://127.0.0.1:8788/#remote');
    await page.waitForFunction(()=>window.__remoteLive?.snapshot?.source==='live'&&window.__remoteLive.snapshot.link?.state==='up',null,{timeout:45000});
    await page.waitForFunction(()=>document.querySelector('#first-person-image').naturalWidth>0&&!document.querySelector('#first-person-image').hidden,null,{timeout:20000});
    await page.waitForFunction(()=>window.__remoteLive?.mapping?.count>0,null,{timeout:45000});
    await page.locator('[data-remote-view=split]').click();
    await page.waitForTimeout(4000);
    const result=await page.evaluate(()=>{
      const s=window.__remoteLive.snapshot,img=document.querySelector('#first-person-image');
      return {source:s.source,link:s.link,pose:s.robot?.pose,objectCount:s.objects.length,task:s.task?.state,estop:s.estop,mapping:window.__remoteLive.mapping,camera:[img.naturalWidth,img.naturalHeight],cameraLabel:document.querySelector('#camera-source').textContent,mapCaption:document.querySelector('#remote-map-caption').textContent,viewport:[innerWidth,innerHeight],document:[document.documentElement.scrollWidth,document.documentElement.scrollHeight],armed:document.querySelector('#remote-arm').checked};
    });
    assert.equal(result.source,'live');assert.equal(result.link.state,'up');assert.equal(result.armed,false);assert.deepEqual(sent,[]);assert.deepEqual(errors,[]);
    assert.equal(result.mapping.source,'rtabmap');assert(result.mapping.count>0);assert(result.mapping.graphNodes>=1);assert.deepEqual(result.viewport,result.document);
    if(result.mapping.cached)assert(result.mapCaption.includes('缓存'));
    const out=path.resolve(process.env.GEOMEM_OBSERVATION_DIR||path.resolve(__dirname,'../artifacts/competition-live'));fs.mkdirSync(out,{recursive:true});
    await page.screenshot({path:path.join(out,'actual-stack-split.png'),fullPage:true});
    await page.locator('[data-remote-view=third]').click();await page.waitForTimeout(1000);
    await page.screenshot({path:path.join(out,'actual-mapping-third.png'),fullPage:true});
    await page.setViewportSize({width:1280,height:720});await page.locator('[data-remote-view=split]').click();await page.waitForTimeout(1000);
    assert(await page.evaluate(()=>document.documentElement.scrollHeight===innerHeight&&document.documentElement.scrollWidth===innerWidth));
    assert(await page.evaluate(()=>{const boxes=[...document.querySelectorAll('#remote-map-labels button:not([hidden])')].map(e=>e.getBoundingClientRect());return boxes.every((a,i)=>boxes.every((b,j)=>i===j||a.right<=b.left||b.right<=a.left||a.bottom<=b.top||b.bottom<=a.top));}),'map labels must not overlap');
    await page.screenshot({path:path.join(out,'actual-mapping-laptop.png'),fullPage:true});
    await page.locator('#remote-map-objects').click();
    assert.equal(await page.locator('#remote-map-object-list button').count(),await page.evaluate(()=>window.__remoteLive.snapshot.objects.filter(o=>o.state!=='position_invalid').length));
    await page.locator('#remote-drawer-close').click();
    assert.deepEqual(sent,[]);assert.deepEqual(errors,[]);
    fs.writeFileSync(path.join(out,'actual-stack-observation.json'),JSON.stringify({...result,sentCommands:sent,pageErrors:errors},null,2));
    console.log(JSON.stringify(result,null,2));console.log('PASS: actual stack observed; no vehicle command sent. This is not a movement or navigation test.');
  } finally {await browser.close();}
})().catch(error=>{console.error(error);process.exitCode=1;});
