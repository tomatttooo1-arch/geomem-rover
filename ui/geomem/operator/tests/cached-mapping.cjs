// Isolated real-browser regression: no backend proxy or hardware connection exists.
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'C:/Users/lenovo/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
const assert=require('node:assert/strict'),crypto=require('node:crypto'),fs=require('node:fs'),http=require('node:http'),path=require('node:path');
const root=path.resolve(__dirname,'..');
const out=path.resolve(process.env.GEOMEM_CACHED_MAPPING_EVIDENCE_DIR||path.join(root,'artifacts','cached-mapping-isolated-'+new Date().toISOString().replace(/[:.]/g,'-')));
const types={'.html':'text/html; charset=utf-8','.js':'text/javascript','.css':'text/css','.svg':'image/svg+xml','.png':'image/png','.json':'application/json','.webmanifest':'application/manifest+json'};

// A static-only server makes this fixture independent of a running vehicle gateway.
function staticServer(){return http.createServer(async(req,res)=>{
  try{
    const relative=decodeURIComponent(new URL(req.url,'http://localhost').pathname).replace(/^\/+/, '')||'index.html';
    const file=path.resolve(root,relative);
    if(req.method!=='GET'||!file.startsWith(root+path.sep)||!['index.html','styles.css','manifest.webmanifest','sw.js','src','vendor','assets'].includes(relative.split('/')[0])){res.writeHead(404).end();return;}
    const bytes=await fs.promises.readFile(file);res.writeHead(200,{'Content-Type':types[path.extname(file)]||'application/octet-stream','Cache-Control':'no-store'}).end(bytes);
  }catch{res.writeHead(404).end();}
});}

const body=Buffer.alloc(8*15);
for(let i=0;i<8;i++){
  body.writeFloatLE((i%4)*.6,12*i);body.writeFloatLE(Math.floor(i/4)*.8,12*i+4);body.writeFloatLE(.3+(i%2)*.6,12*i+8);
  body.set([40+i*25,190-i*10,220-i*15],8*12+i*3);
}
const sha256=crypto.createHash('sha256').update(body).digest('hex');
let meta={format:'geomem-live-mapping',version:1,source:'rtabmap',session:'fixture-session-a',scope:'fixture-boot-a',frame:'map',cellM:.01,count:8,revision:1,graphNodes:1,graphAgeMs:95000,cloudAgeMs:95000,sha256,mapToOdom:{position:[0,0,0],quaternion:[0,0,0,1]}};
function packet(){const header=Buffer.from(JSON.stringify(meta)),prefix=Buffer.alloc(4);prefix.writeUInt32LE(header.length);return Buffer.concat([prefix,header,body]);}

(async()=>{
  fs.mkdirSync(out,{recursive:true});
  const server=staticServer();await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
  let browser,timer,socket,seq=0,httpOK=true,mappingRequests=0,publishing=true;
  const commands=[],errors=[],stages=[],unexpectedRequests=[];
  const state={type:'snapshot',version:2,seq:0,serverTime:1000,source:'live',link:{state:'up',hubAgeMs:0},map:{id:null,name:null,frame:'odom',units:'m',cloudRev:null,placeholder:false},robot:{position:[0,0,0],yaw:0,speed:0,pose:'odometry',poseAgeMs:0,status:'待命'},memory:{scope:'fixture-boot-a',revision:1,ageMs:0},objects:[],path:[],task:{id:null,state:'idle',message:'隔离地图测试',targetId:null,candidates:[]},estop:{latched:false,epoch:1}};
  const publish=()=>{if(publishing&&socket)socket.send(JSON.stringify({...state,seq:++seq}));};
  try{
    browser=await chromium.launch({headless:true,executablePath:process.env.BROWSER_PATH||'C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe'});
    const page=await browser.newPage({viewport:{width:1440,height:900}});
    page.on('pageerror',e=>errors.push(e.message));
    // Block every service route; only the cloud fixture below returns data.
    await page.route('**/bridge/**',r=>{if(r.request().method()!=='GET')unexpectedRequests.push(r.request().method()+' '+r.request().url());return r.fulfill({status:503,body:'isolated fixture; no bridge or camera'});});
    await page.route('**/rover/**',r=>{if(r.request().method()!=='GET')unexpectedRequests.push(r.request().method()+' '+r.request().url());return r.fulfill({status:503,body:'isolated fixture; no rover'});});
    await page.route('**/mapping/**',r=>{mappingRequests++;return httpOK?r.fulfill({status:200,contentType:'application/octet-stream',body:packet()}):r.fulfill({status:503,body:'isolated mapping feed failure'});});
    await page.routeWebSocket('**/ws',ws=>{
      socket=ws;
      ws.onMessage(raw=>{const message=JSON.parse(raw);if(message.type==='command')commands.push(message);});
      ws.send(JSON.stringify({type:'hello',version:2,bridge:{id:'isolated-cached-mapping-fixture',hubBootId:'fixture-hub',link:'up'},source:'live',capabilities:{control:'controller',findText:true,select:true,cancel:true,teleop:true,estop:true,release:true,mappingStop:true,goto:true}}));publish();
    });
    const read=()=>page.evaluate(()=>({remote:window.__remoteLive.mapping,overview:window.__overviewMap.mapping,geometryVisible:window.__overviewMap.geometryVisible,visiblePoints:window.__overviewMap.visibleCount,caption:document.querySelector('#remote-map-caption').textContent,status:document.querySelector('#map-status').textContent,taskTarget:document.querySelector('#task-target').textContent,viewport:[innerWidth,innerHeight],document:[document.documentElement.scrollWidth,document.documentElement.scrollHeight],armed:document.querySelector('#remote-arm').checked,link:window.__remoteLive.snapshot.link.state,scope:window.__remoteLive.snapshot.memory.scope}));
    const capture=async(name)=>{assert.deepEqual(commands,[]);assert.deepEqual(errors,[]);const result=await read();stages.push({name,...result});return result;};
    const loaded=()=>page.waitForFunction(scope=>window.__remoteLive?.mapping?.scope===scope&&window.__overviewMap?.mapping?.scope===scope,state.memory.scope,{timeout:10000});
    const cleared=()=>page.waitForFunction(()=>!window.__remoteLive?.mapping&&!window.__overviewMap?.mapping&&!window.__overviewMap?.geometryVisible,null,{timeout:5000});
    timer=setInterval(publish,100);
    await page.goto(`http://127.0.0.1:${server.address().port}/#remote`);
    await loaded();await page.locator('[data-remote-view=third]').click();
    const cached=await capture('current scope displays a cached stationary one-node map');
    assert.equal(cached.remote.cached,true);assert.equal(cached.overview.cached,true);assert.equal(cached.remote.graphNodes,1);assert.equal(cached.remote.cloudAgeMs,95000);assert.equal(cached.armed,false);
    assert(cached.caption.includes('缓存 · 1 分钟前更新'));assert(cached.status.includes('缓存 · 1 分钟前更新'));assert(cached.geometryVisible);assert.equal(cached.visiblePoints,8);
    await page.screenshot({path:path.join(out,'cached-map-remote.png'),fullPage:true});
    await page.locator('[data-page=overview]').click();
    await page.screenshot({path:path.join(out,'cached-map-overview.png'),fullPage:true});

    // Cloud bytes/revision stay fixed. The next successful poll must refresh both scenes' metadata.
    meta={...meta,cloudAgeMs:185000,graphAgeMs:185000,mapToOdom:{position:[1,0,0],quaternion:[0,0,0,1]}};
    await page.waitForFunction(()=>window.__remoteLive.mapping?.cloudAgeMs===185000&&window.__overviewMap.mapping?.cloudAgeMs===185000);
    const aged=await capture('same SHA and revision refresh age and map transform metadata');
    assert.equal(aged.remote.sha256,cached.remote.sha256);assert.equal(aged.overview.sha256,cached.overview.sha256);assert.equal(aged.overview.revision,1);assert.deepEqual(aged.overview.mapToOdom.position,[1,0,0]);
    assert(aged.caption.includes('缓存 · 3 分钟前更新'));assert(aged.status.includes('缓存 · 3 分钟前更新'));

    state.memory.scope='fixture-boot-b';publish();await cleared();
    await page.waitForTimeout(1800);await cleared();
    const changed=await capture('scope change clears old cloud and rejects old-scope HTTP replies');
    assert(!changed.caption.includes('RTAB-Map'));assert.equal(changed.scope,'fixture-boot-b');
    meta={...meta,scope:'fixture-boot-b',session:'fixture-session-b'};
    await loaded();await capture('new scope accepts only its own accumulated cloud');

    meta={...meta,cloudAgeMs:120,graphAgeMs:100,revision:2};
    await page.waitForFunction(()=>window.__remoteLive.mapping?.cached===false&&window.__overviewMap.mapping?.cached===false);
    const recent=await capture('fresh cloud removes cached label even with identical bytes');
    assert(!recent.caption.includes('缓存'));assert(!recent.status.includes('缓存'));assert.equal(recent.overview.sha256,cached.overview.sha256);
    meta={...meta,cloudAgeMs:95000,graphAgeMs:95000};
    await page.waitForFunction(()=>window.__overviewMap.mapping?.cached===true);

    state.link.state='down';publish();await cleared();
    await page.waitForTimeout(1800);await cleared();
    const down=await capture('hub link down clears cloud while WebSocket remains connected');
    assert.equal(down.link,'down');assert(!down.caption.includes('RTAB-Map'));
    state.link.state='up';publish();await loaded();await capture('fresh hub recovers current-scope cache');

    httpOK=false;await cleared();
    const failed=await capture('HTTP feed failure clears both scenes');
    assert(!failed.caption.includes('RTAB-Map'));
    httpOK=true;await loaded();await capture('HTTP feed recovery restores current-scope cache');

    // Freshness timeout must clear cached geometry even when no new snapshot arrives.
    publishing=false;await cleared();
    await capture('silent hub exceeding freshness window clears both scenes');
    publishing=true;publish();await loaded();
    await page.setViewportSize({width:390,height:844});await page.waitForTimeout(500);
    const mobileOverview=await capture('mobile overview cache caption and idle task fallback');
    assert.equal(mobileOverview.viewport[0],390);assert(mobileOverview.document[0]<=390,'mobile overview has horizontal overflow');
    assert.equal(mobileOverview.taskTarget,'等待寻物指令 · 实车任务');assert(!mobileOverview.taskTarget.includes('null'));assert(mobileOverview.status.includes('缓存'));
    await page.screenshot({path:path.join(out,'cached-map-mobile-overview.png'),fullPage:true});
    await page.locator('[data-page=remote]').click();await page.waitForTimeout(500);
    const mobileRemote=await capture('mobile remote cached caption fits viewport');
    assert(mobileRemote.document[0]<=390,'mobile remote has horizontal overflow');assert(mobileRemote.caption.includes('缓存'));
    await page.screenshot({path:path.join(out,'cached-map-mobile-remote.png'),fullPage:true});
    assert.deepEqual(commands,[]);assert.deepEqual(unexpectedRequests,[]);assert.deepEqual(errors,[]);
    fs.writeFileSync(path.join(out,'cached-mapping-regression.json'),JSON.stringify({source:'isolated-browser-fixture',staticOnlyServer:true,mappingRequests,sentCommands:commands,unexpectedRequests,pageErrors:errors,stages},null,2));
    console.log(`PASS: current-scope stationary cached map, honest age labels, same-hash metadata updates, scope change, fresh-cloud recovery, hub link down, HTTP failure/recovery, silent-hub expiry, 390x844 layouts and idle task fallback; zero commands. Evidence: ${out}`);
  }finally{
    clearInterval(timer);if(browser)await browser.close();server.closeAllConnections();await new Promise(resolve=>server.close(resolve));
  }
})().catch(error=>{console.error(error);process.exitCode=1;});
