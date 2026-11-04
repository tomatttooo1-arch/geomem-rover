import {LiveAdapter} from './v2/transport.js';
import {snapshotFresh,FRESH_MS,displayName,OBJECT_STATE_ZH,stateLabel,esc,endpointURL} from './v2/model.js';
import {SpaceView} from './scene.js';
import {decodeMapping,decodePushedMapping,inflateVoxelMessage,transformPoint,mappingAgeLabel} from './live-mapping.js';
import {VoxelStore,MappingResync} from './voxel-store.js';
import {beliefCaption,doneHideDelay,FADE_MS} from './belief-layer.js';
import {MapResetController,mapResetBlocker} from './map-reset.js';
import {readAr,drawBoxes} from './fpv-boxes.js';
import {renderHealth} from './health-panel.js';
import {noteFpga} from './fpga-panel.js';

// Existing M8 controls. All commands use the competition bridge; the old camera agent is read-only.
export function setupRemote({onSnapshot=()=>{},onConnection=()=>{},onMapping=()=>{}}={}) {
  const $=id=>document.getElementById(id);
  let adapter=null,snapshot=null,received=0,connection='disconnected',caps={},generation=0;
  let camera=null,cameraAt=0,frameURL=null,held=null,driveTimer=null,viewMode='first',cloud=[];
  let scene=null,basemapId=null,loadingMap=null,mapGeneration=0,mappingPending=false,drawerReturn=null;
  let mapping=null,mappingAt=0,mappingError='',mapObjectsKey='';
  // 10-21: the Nano gateway pushes preview frames and the RTAB-Map cloud over the WebSocket; HTTP polling is
  // only the fallback for an older bridge (no push within PUSH_GRACE_MS).
  const PUSH_GRACE_MS=1500,MAPPING_PUSH_GRACE_MS=6000;
  let previewPushAt=0,previewShowing=false,previewQueued=null,mappingPushAt=0,mappingDecoding=false;
  // 10-22: gateway = hello.gateway from the Nano direct gateway (no rover agent there: never touch /rover/*).
  // Voxel deltas go into one store, applied strictly in arrival order; a lost base asks the bridge for a full.
  let gateway=null,mappingChain=Promise.resolve(),resyncAt=0,storeScope=null;const store=new VoxelStore();
  const mstats={full:0,delta:0,keep:0,resync:0,bytes:0,lastBytes:0,lastApplyMs:0};
  // snapshot arrival gaps over the last 60 s (owner's acceptance readout on the laptop) and pushed preview arrivals
  let lastSnapAt=0;const snapGaps=[],previewTimes=[];
  // 10-27 first-person boxes (same rows as the car screen AR): the newest frame's ar block; a missing one only warns
  let fpvAr=null,fpvWarned='';
  const paintFpv=()=>{const c=$('fpv-boxes');if(c)try{drawBoxes(c,$('first-person-image'),fpvAr,snapshot?.objects);}catch{}};
  try{new ResizeObserver(paintFpv).observe($('camera-stage'));}catch{}
  const log=[],checks={};let stage='prepare',lastCandidateKey='',lastTask='',lastConnection='';
  const guides={
    prepare:['设备检查','先架空核验控制链路，再开始落地彩排。',['Nano 与 KV260 在线','急停、解除、断流停车已架空验证','遥控接管会撤销导航']],
    mapping:['遥控建图','标记起点和朝向，低速绕场，多角度观察物体。',['已启用建图与遥控','地图与物体记忆有更新','已回到起点附近']],
    save:['保存与底图','结束建图，收到回执后收车，再生成 1 cm 底图。',['建图保存已收到完成回执','收车记录 memory bind rc=0','地图、记忆、数据库已保存','底图 SHA256 与地图库一致']],
    demo:['第二趟寻物','原位、原朝向摆车，带地图和记忆起栈，输入目标。',['已摆回起点和原朝向','已加载历史地图和物体记忆','目标已在现场重新观测确认']],
    finish:['收车记录','撤销任务，确认车辆状态，收车并保存运行记录。',['车辆已停止并现场确认','车栈已关闭，运行目录已保留','已导出本次操作记录']]
  };
  const active=()=>!$('remote-page').hidden;
  const fresh=()=>connection==='connected'&&snapshot?.source==='live'&&snapshotFresh(snapshot,received);
  // camera freshness = the frame's own age (pushed preview: hub age at send + time since arrival), independent of snapshots
  const cameraFresh=()=>camera?.source==='live'&&camera.camera&&performance.now()-cameraAt+(camera.ageMs||0)<1800;
  // why fresh() is false, in words: a late snapshot is a delay on a live link, not "not connected"
  function linkReason(){
    if(connection!=='connected')return '转发桥未连接';
    if(!snapshot)return '等待车栈数据';
    if(snapshot.source!=='live')return '非实时数据';
    if(snapshot.link?.state!=='up')return '车栈未连接';
    const age=performance.now()-received;
    return age>=FRESH_MS?`车栈数据延迟 ${(age/1000).toFixed(1)} 秒`:'';
  }
  const snapLate=()=>connection==='connected'&&snapshot?.source==='live'&&snapshot.link?.state==='up'&&!fresh();
  // 10-25: the odometry badge only replaces the idle text -- 急停锁存, 相机前方过近, 正在前往 etc. stay visible
  // 10-25 find-retry: the bridge's optional positionsTried (distinct goals the car went to or tried) after the message.
  function taskMessageText(t){const m=t?.message||'从记忆出发，到现场重新确认。',n=t?.positionsTried;return t&&t.state!=='idle'&&Number.isSafeInteger(n)&&n>0?`${m}（已尝试 ${n} 个位置）`:m;}
  function robotStatusText(r){const st=r?.status||'未知';return r?.pose==='odometry'&&st==='待命'?'仅里程计定位':st;}
  function blocker(cap){if(!fresh())return linkReason()+'，控制锁定';if(caps.control!=='controller')return '当前没有控制权';if(!caps[cap])return '车栈未提供此功能';if(snapshot.estop?.latched!==false)return '急停锁存或状态未知';return '';}
  function record(message){log.unshift({time:new Date().toISOString(),source:'live',message});log.splice(300);$('remote-events').innerHTML=log.slice(0,80).map(r=>`<li><time>${esc(new Date(r.time).toLocaleTimeString())}</time>${esc(r.message)}</li>`).join('');}
  async function request(action,payload={},quiet=false){
    if(!adapter)throw Error('转发桥尚未连接');
    const cap={find:'findText',select:'select',goto:'goto',teleop:'teleop',mapping_stop:'mappingStop'}[action];
    if(cap&&!(action==='teleop'&&payload.stop)&&blocker(cap))throw Error(blocker(cap));
    if(['find','select','goto'].includes(action)&&held)throw Error('请先松开遥控');
    if(['pause','resume'].includes(action))throw Error('车栈不支持暂停/恢复，请撤销后重新发送目标');
    const c=adapter.command(action,payload),gen=generation;
    if(!quiet)record(`${action}：已发送，等待回执`);
    try{await c.acked;if(gen===generation&&!quiet)record(`${action}：已受理，等待执行结果`);return c.id;}
    catch(e){if(gen===generation){if(!quiet)record(`${action}：${e.message}`);if(action==='teleop')disarm();}throw e;}
  }
  function stop(){if(!held)return;held=null;clearInterval(driveTimer);driveTimer=null;document.querySelectorAll('[data-drive]').forEach(b=>b.classList.remove('is-held'));void request('teleop',{stop:true},true).catch(()=>{});}
  // 10-21 owner: no separate "enable driving" switch. Hold to drive, release/blur/blocker stops; nothing to re-arm.
  function disarm(){stop();}
  function pulse(){if(!active()||blocker('teleop')||document.hidden||!$('remote-drawer').hidden||document.querySelector('dialog[open]')){disarm();return;}
    const k=Number($('drive-speed').value)/100,[v,w]=({forward:[.15*k,0],backward:[-.15*k,0],left:[0,.6*k],right:[0,-.6*k]})[held]||[0,0];
    void request('teleop',{v,w},true).catch(()=>{});
  }
  function start(direction){if(held||blocker('teleop'))return;held=direction;document.querySelector(`[data-drive=${direction}]`)?.classList.add('is-held');record('遥控接管：原导航撤销，松手后需重新发目标');pulse();driveTimer=setInterval(pulse,100);}
  document.querySelectorAll('[data-drive]').forEach(b=>{b.addEventListener('pointerdown',e=>{if(e.button!==0)return;e.preventDefault();if(b.dataset.drive==='stop'){stop();void request('teleop',{stop:true},true).catch(()=>{});return;}b.setPointerCapture(e.pointerId);start(b.dataset.drive);});for(const n of ['pointerup','pointercancel','lostpointercapture'])b.addEventListener(n,stop);});
  const typing=el=>el.isContentEditable||el.matches('textarea,select,input:not([type=checkbox]):not([type=radio]):not([type=range])');
  const keys={KeyW:'forward',KeyS:'backward',KeyA:'left',KeyD:'right'};
  document.addEventListener('keydown',e=>{if(typing(e.target))return;if(e.code==='Space'){e.preventDefault();if(!e.repeat&&adapter&&connection==='connected'){disarm();void request('estop').catch(()=>{});}return;}if(!active()||!keys[e.code]||!$('remote-drawer').hidden||document.querySelector('dialog[open]'))return;e.preventDefault();if(!e.repeat)start(keys[e.code]);});
  document.addEventListener('keyup',e=>{if(keys[e.code]===held)stop();});
  for(const n of ['blur','pagehide'])window.addEventListener(n,disarm);
  // Hidden tabs must release bridge sessions (the bridge deliberately caps them at eight).
  // Returning to a tab reconnects telemetry, never re-arms driving or resends a task.
  function suspendConnection(){disarm();adapter?.close();clearMapping(true);}
  document.addEventListener('visibilitychange',()=>{if(document.hidden)suspendConnection();else if(adapter?.closed)connect(adapter.url);});
  window.addEventListener('pagehide',suspendConnection);
  window.addEventListener('pageshow',()=>{if(!document.hidden&&adapter?.closed)connect(adapter.url);});
  document.addEventListener('geomem-page-change',()=>{disarm();closeDrawer();});
  $('drive-speed').oninput=()=>{$('drive-speed-value').value=$('drive-speed').value+'%';};
  $('full-stop').onclick=()=>{disarm();void request('estop').catch(()=>{});};
  $('remote-release').onclick=()=>{disarm();$('remote-release-dialog').dataset.epoch=String(snapshot?.estop?.epoch??'');$('remote-release-dialog').showModal();};
  $('remote-release-confirm').onclick=()=>{const epoch=Number($('remote-release-dialog').dataset.epoch);$('remote-release-dialog').close();if(!fresh()||!Number.isSafeInteger(epoch)||epoch!==snapshot?.estop?.epoch){record('急停状态已变化，请重新确认解除');return;}void request('release',{confirm:true,epoch}).catch(()=>{});};
  $('remote-find-form').onsubmit=e=>{e.preventDefault();void request('find',{text:$('remote-query').value.trim()}).catch(e=>record(e.message));};
  $('remote-cancel').onclick=()=>void request('cancel',{taskId:snapshot?.task?.id}).catch(()=>{});
  $('remote-mapping-stop').onclick=()=>{disarm();$('remote-mapping-dialog').showModal();};
  $('remote-mapping-confirm').onclick=()=>{$('remote-mapping-dialog').close();if(blocker('mappingStop'))return;mappingPending=true;render();void request('mapping_stop').catch(()=>{mappingPending=false;render();});};
  // 10-26 重置地图（map-reset.js）：二次确认 → map_reset {confirm:true} → 完成后清空页面上的地图、记忆和热力图
  const mapReset=new MapResetController({send:(a,p)=>request(a,p),clear:clearAfterMapReset,onChange:()=>render()});
  // 急停锁存时也允许（车本来就不动；hub 同样放行），所以不用 blocker() 的急停门
  const mapResetReason=()=>!fresh()?linkReason()+'，控制锁定':caps.control!=='controller'?'当前没有控制权':mapResetBlocker(snapshot,caps);
  $('remote-map-reset-open').onclick=()=>{disarm();if(mapReset.ask(mapResetReason()))$('remote-map-reset-dialog').showModal();};
  $('remote-map-reset-dialog').addEventListener('close',()=>mapReset.cancel());
  $('remote-map-reset-confirm').onclick=()=>{$('remote-map-reset-dialog').close();void mapReset.confirm(mapResetReason());};
  $('remote-goto-form').onsubmit=e=>{e.preventDefault();void request('goto',{forwardM:Number($('remote-forward').value),turnRad:Number($('remote-turn').value)*Math.PI/180}).catch(e=>record(e.message));};
  function openDrawer(name){disarm();drawerReturn=document.activeElement;$('remote-drawer').hidden=false;$('remote-drawer-title').textContent=({flow:'比赛流程',connection:'设备与连接',log:'操作与回执',advanced:'更多操作',candidates:'选择目标物体',objects:'全部地图物体'})[name];for(const s of document.querySelectorAll('[data-remote-content]'))s.hidden=s.dataset.remoteContent!==name;for(const b of document.querySelectorAll('.remote-drawer-tabs button'))b.classList.toggle('active',b.dataset.remotePanel===name);$('remote-drawer-close').focus();}
  function closeDrawer(){if($('remote-drawer').hidden)return;$('remote-drawer').hidden=true;drawerReturn?.focus?.();}
  for(const b of document.querySelectorAll('[data-remote-panel]'))b.onclick=()=>openDrawer(b.dataset.remotePanel);
  $('remote-drawer-close').onclick=closeDrawer;$('remote-candidates-open').onclick=()=>openDrawer('candidates');$('camera-settings').onclick=()=>openDrawer('connection');
  document.addEventListener('keydown',e=>{if(e.key==='Escape'&&!$('remote-drawer').hidden){e.preventDefault();closeDrawer();}});
  for(const id of ['source-button','notice-connect','settings-button','device-connect'])$(id).onclick=()=>openDrawer('connection');
  function setStage(next){stage=next;const [title,guide,items]=guides[next];$('remote-flow-guide').textContent=guide;$('remote-flow-checks').replaceChildren();items.forEach((text,i)=>{const label=document.createElement('label'),box=document.createElement('input');box.type='checkbox';box.checked=!!checks[next+':'+i];box.onchange=()=>{checks[next+':'+i]=box.checked;record(`${box.checked?'确认':'取消确认'}：${text}（人工勾选）`);};label.append(box,document.createTextNode(text));$('remote-flow-checks').append(label);});for(const b of document.querySelectorAll('[data-flow]'))b.classList.toggle('active',b.dataset.flow===next);}
  for(const b of document.querySelectorAll('[data-flow]'))b.onclick=()=>setStage(b.dataset.flow);setStage('prepare');
  $('remote-export').onclick=()=>{const data={format:'geomem-live-session',source:'live',exportedAt:new Date().toISOString(),stage,checks,map:$('remote-map-path').value,memory:$('remote-memory-path').value,sha256:$('remote-map-sha').value,events:log,snapshot};const url=URL.createObjectURL(new Blob([JSON.stringify(data,null,2)],{type:'application/json'})),a=document.createElement('a');a.href=url;a.download='geomem-live-'+Date.now()+'.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};
  function setView(mode){viewMode=mode;$('remote-visuals').dataset.view=mode;for(const b of document.querySelectorAll('[data-remote-view]')){b.classList.toggle('selected',b.dataset.remoteView===mode);b.setAttribute('aria-pressed',String(b.dataset.remoteView===mode));}requestAnimationFrame(()=>scene?.resize());}
  for(const b of document.querySelectorAll('[data-remote-view]'))b.onclick=()=>setView(b.dataset.remoteView);
  try{scene=new SpaceView($('remote-map-canvas'),$('remote-map-labels'),()=>openDrawer('objects'));scene.declutterLabels=true;scene.clear();scene.setSource('live');}catch(e){record('三维渲染不可用：'+e.message);}
  $('remote-map-objects').onclick=()=>openDrawer('objects');
  $('remote-map-reset').onclick=()=>scene?.reset();
  function legacy(s){return {type:'snapshot',version:1,seq:s.seq,timestamp:new Date(Date.now()-Math.max(0,s.link?.hubAgeMs||0)).toISOString(),map:{id:s.map?.id||'',name:s.map?.name||'实车空间',frame:'map',units:'m'},robot:s.robot?.pose!=='none'?s.robot?{...s.robot,localized:s.robot.pose==='map'}:null:null,objects:s.objects.filter(o=>o.state!=='position_invalid').map(o=>({...o,name:displayName(o),location:OBJECT_STATE_ZH[o.state]||'物体记忆',lastSeen:null,aliases:[],confidence:Number.isFinite(o.confidence)?o.confidence:null})),path:s.path||[],points:cloud,task:s.task?{...s.task,updatedAt:new Date().toISOString()}:null,health:s.health,notices:s.notices};}   // 10-29: the overview's showCarNotices reads health/notices too
  // hides the 3-D cloud; hard also drops the voxel store (new bridge session / other car session / bad data), so the
  // next delta asks for a full. A soft clear (snapshot stall, base map shown) keeps the store in step with the feed.
  // 重置完成：三维体素/点云/底图、轨迹与物体（下一帧快照按新记忆重画）、候选、热力图全部清掉
  function clearAfterMapReset(){hideBelief();clearMapping(true);cloud=[];mapGeneration++;loadingMap=null;basemapId=null;scene?.clearBasemap();scene?.clear();lastCandidateKey='';mapObjectsKey='';resyncAt=0;record('重置地图：已清空页面上的地图、记忆和热力图');paintMap();}
  function clearMapping(hard=false){mapping=null;mappingAt=0;if(hard){store.reset();storeScope=null;}scene?.clearMappingCloud();onMapping(null);}
  function paintMap(){
    if(!snapshot||snapshot.source!=='live')return;
    if(mapping&&(!fresh()||mapping.scope!==snapshot.memory?.scope||performance.now()-mappingAt>10000))clearMapping(mapping.scope!==snapshot.memory?.scope);
    const s=legacy(snapshot);
    if(!fresh())s.robot=null;
    if(mapping&&!basemapId){
      const t=mapping.mapToOdom;
      // Hub objects/path follow its pose tier. RTAB-Map cloud uses optimized map coordinates.
      if(snapshot.robot?.pose==='odometry'){
        if(s.robot){const [x,y,z,w]=t.quaternion;s.robot={...s.robot,position:transformPoint(s.robot.position,t),yaw:s.robot.yaw+Math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))};}
        s.objects=s.objects.map(o=>({...o,position:transformPoint(o.position,t)}));s.path=s.path.map(p=>transformPoint(p,t));
      }else if(snapshot.robot?.pose!=='map'){s.robot=null;s.objects=[];s.path=[];}
      s.points=[];
    }
    scene?.update(s);
    $('remote-map-objects').textContent=`地图物体 ${s.objects.length}`;
    const objectKey=JSON.stringify(s.objects.map(o=>[o.id,o.name,o.position]));
    if(objectKey!==mapObjectsKey){mapObjectsKey=objectKey;$('remote-map-object-list').replaceChildren();for(const o of s.objects){const b=document.createElement('button');b.className='quiet-button';b.textContent=`${o.name} · (${o.position[0].toFixed(2)}, ${o.position[1].toFixed(2)}) m`;b.onclick=()=>{scene?.focus(o.id);$('remote-query').value=o.name;closeDrawer();};$('remote-map-object-list').append(b);}}
    $('remote-map-empty').hidden=!!cloud.length||!!basemapId||!!mapping;
    $('remote-map-caption').textContent=basemapId?'1 cm 真彩底图 · 按实际地图坐标显示':mapping?`RTAB-Map 累积三维 · ${mapping.count.toLocaleString()} 点 · ${mappingAgeLabel(mapping)}`:cloud.length?'局部障碍栅格 · 累积三维待连接':'尚未收到真实地图';
    $('remote-map-caption').title=mapping?`真实 RGB-D 建图 · ${Math.round((mapping.cell||mapping.cellM||.01)*100)} cm 显示网格 · 会话 ${mapping.session}${mapping.encoding==='vdz1'?` · 纪元 ${mapping.epoch} 版本 ${mapping.seq}`:''}`:mappingError;
    onSnapshot(s);
  }
  function showPushedPreview(d,gen){
    previewPushAt=performance.now();previewTimes.push(previewPushAt);while(previewTimes.length&&previewPushAt-previewTimes[0]>10000)previewTimes.shift();
    if(!active()||document.hidden||viewMode==='third')return;
    if(previewShowing){previewQueued=d;return;}          // latest only: never queue more than one frame
    previewShowing=true;
    const url=URL.createObjectURL(new Blob([d.jpeg],{type:'image/jpeg'})),img=$('first-person-image');
    img.onload=()=>{if(frameURL)URL.revokeObjectURL(frameURL);frameURL=url;finish();};
    img.onerror=()=>{URL.revokeObjectURL(url);finish();};
    img.src=url;
    function finish(){
      previewShowing=false;if(gen!==generation)return;
      camera={source:'live',camera:true,ageMs:Number.isFinite(d.header.ageMs)?d.header.ageMs:0};cameraAt=performance.now();img.hidden=false;$('camera-placeholder').hidden=true;
      fpvAr=readAr(d.header);paintFpv();
      if(fpvAr.note!==fpvWarned){fpvWarned=fpvAr.note;if(fpvAr.note)record('第一视角：'+fpvAr.note+'（画面照常）');}
      $('camera-status').textContent='实时画面';$('camera-source').textContent=`车栈实时画面 · ${d.header.w}×${d.header.h}`+(fpvAr.note?' · '+fpvAr.note:fpvAr.boxes.length?` · 识别框 ${fpvAr.boxes.length}`:'');
      $('camera-time').textContent='收到画面 '+new Date().toLocaleTimeString('zh-CN');
      const next=previewQueued;previewQueued=null;if(next)showPushedPreview(next,gen);
    }
  }
  function requestResync(){
    const now=performance.now();if(now-resyncAt<5000)return;   // one request per 5 s until a full arrives
    resyncAt=now;mstats.resync++;adapter?.sendControl({type:'mapping_resync'});
  }
  function showStore(meta,dirty){
    mapping={...meta,count:store.count};mappingAt=performance.now();mappingError='';
    const data={store,meta:mapping,dirty};scene?.setMappingStore(data);onMapping(data);
  }
  async function applyVoxel(d,gen){
    mappingPushAt=performance.now();
    if(gen!==generation)return;
    const t0=performance.now();
    try{
      let meta,raw;
      try{({meta,raw}=await inflateVoxelMessage(d,{cachedScope:snapshot?.memory?.scope}));}
      catch(e){                                     // feed not ready / other session: usually persists, so no
        if(gen!==generation)return;                 // full-every-5-s storm; the next delta resyncs once it clears
        clearMapping();mappingError=e.message;resyncAt=Math.max(resyncAt,performance.now()+25000);return;
      }
      if(gen!==generation)return;
      const show=fresh()&&!basemapId;               // the store is kept in step even while the cloud is hidden
      if(meta.mode==='keep'){
        mstats.keep++;
        if(!(store.ready&&meta.epoch===store.epoch&&meta.seq===store.seq)){requestResync();return;}
        if(!show)return;
        if(mapping)mapping={...mapping,cloudAgeMs:meta.cloudAgeMs,graphAgeMs:meta.graphAgeMs,mapToOdom:meta.mapToOdom,graphNodes:meta.graphNodes,cached:meta.cached};
        else showStore({...meta,cell:store.cell,origin:store.origin,encoding:'vdz1'},null);   // re-show after a stall
        mappingAt=performance.now();return;
      }
      if(store.ready&&storeScope!==meta.scope)store.reset();
      const {dirty}=store.apply(meta,raw);storeScope=meta.scope;
      mstats[meta.mode]++;mstats.lastBytes=d.payload?.byteLength||0;mstats.bytes+=mstats.lastBytes;mstats.lastApplyMs=Math.round(performance.now()-t0);
      if(meta.mode==='full')resyncAt=0;
      if(show)showStore(meta,mapping?dirty:null);
    }catch(e){
      if(gen!==generation)return;
      if(e instanceof MappingResync){requestResync();return;}
      clearMapping(true);mappingError=e.message;requestResync();
    }finally{if(gen===generation)paintMap();}
  }
  async function applyPushedMapping(d,gen){
    if(d.header?.encoding==='vdz1'||d.header?.mode==='keep'){const chained=mappingChain.then(()=>applyVoxel(d,gen));mappingChain=chained.catch(()=>{});return chained;}
    mappingPushAt=performance.now();
    if(!fresh()||basemapId)return;
    if(!d.payload?.byteLength){                       // keep-alive: refresh age of the cloud already shown
      if(mapping&&mapping.sha256===d.header.sha256){mapping={...mapping,cloudAgeMs:d.header.cloudAgeMs,graphAgeMs:d.header.graphAgeMs,cached:d.header.cloudAgeMs>15000};mappingAt=performance.now();paintMap();}
      return;
    }
    if(mappingDecoding)return;                        // the next change is pushed again
    mappingDecoding=true;
    try{
      const decoded=await decodePushedMapping(d,{cachedScope:snapshot?.memory?.scope});
      if(gen!==generation||!fresh()||basemapId)return;
      scene?.setMappingCloud(decoded);onMapping(decoded);
      mapping=decoded.meta;mappingAt=performance.now();mappingError='';
    }catch(e){if(gen===generation){clearMapping();mappingError=e.message;}}
    finally{mappingDecoding=false;if(gen===generation)paintMap();}
  }
  async function pollMapping(){
    const gen=generation;
    try{
      if((!active()&&$('overview-page').hidden)||document.hidden||!fresh()||basemapId)return;
      if(performance.now()-mappingPushAt<MAPPING_PUSH_GRACE_MS)return;   // pushed: no polling
      const r=await fetch('/mapping/cloud.bin?t='+Date.now(),{cache:'no-store',signal:AbortSignal.timeout(8000)});
      if(!r.ok)throw Error('累积三维通道未连接');
      const decoded=await decodeMapping(await r.arrayBuffer(),{cachedScope:snapshot.memory?.scope});
      if(gen!==generation)return;
      if(decoded.meta.scope!==snapshot?.memory?.scope)throw Error('三维地图与车栈不属于同一会话');
      if(!fresh()||basemapId)return;
      scene?.setMappingCloud(decoded);onMapping(decoded);
      mapping=decoded.meta;mappingAt=performance.now();mappingError='';
    }catch(e){if(gen===generation){clearMapping();mappingError=e.message;}}
    finally{if(gen===generation)paintMap();setTimeout(pollMapping,1500);}
  }
  async function loadBasemap(s){const bm=s.map?.basemap,want=bm?.status==='ok'&&((bm.basis==='mapping_start'&&bm.tier==='odom')||bm.sha256===s.map.id)?bm.sha256:null;if(!want){mapGeneration++;loadingMap=null;if(basemapId){scene?.clearBasemap();basemapId=null;}return;}if(want===basemapId||want===loadingMap)return;const gen=++mapGeneration;loadingMap=want;basemapId=null;scene?.clearBasemap();try{const paths=[bm.url,bm.cellsUrl].map(p=>{const u=new URL(p,location.href);if(u.origin!==location.origin||!u.pathname.startsWith('/basemap/'+want+'/'))throw Error('底图地址不匹配');return '/bridge'+u.pathname;});const [m,b]=await Promise.all([fetch(paths[0]).then(r=>{if(!r.ok)throw Error('底图清单读取失败');return r.json();}),fetch(paths[1]).then(r=>{if(!r.ok)throw Error('底图读取失败');return r.arrayBuffer();})]);if(gen!==mapGeneration)return;const hash=[...new Uint8Array(await crypto.subtle.digest('SHA-256',b))].map(n=>n.toString(16).padStart(2,'0')).join('');if(m.format!=='geomem-basemap'||m.source_db?.sha256!==want||m.files?.cells?.bytes!==b.byteLength||m.files.cells.sha256!==hash||m.cell_m!==.01)throw Error('底图身份、精度或 SHA 校验失败');clearMapping();scene?.setBasemap(m,b);basemapId=want;paintMap();}catch(e){if(gen===mapGeneration)record(e.message);}finally{if(gen===mapGeneration)loadingMap=null;}}
  function render(){
    const ok=fresh(),reason=blocker('teleop');if((reason||!active())&&held)stop();
    $('drive-speed').disabled=!!reason;
    document.querySelectorAll('[data-drive]').forEach(b=>b.disabled=b.dataset.drive==='stop'?!(connection==='connected'&&caps.teleop):!!reason);
    $('full-stop').disabled=connection!=='connected';$('remote-release').disabled=!ok||caps.control!=='controller'||!caps.release||snapshot?.estop?.latched!==true;
    $('drive-status').textContent=reason||'W/A/S/D 按住驾驶；松手停止';$('control-lock').textContent=reason?'控制锁定':'按住驾驶';
    $('remote-find').disabled=!!blocker('findText')||!!held;$('remote-goto').disabled=!!blocker('goto')||!!held;$('remote-cancel').disabled=connection!=='connected'||!caps.cancel;
    $('remote-mapping-stop').disabled=!!blocker('mappingStop')||mappingPending||mapReset.pending;
    {const why=mapResetReason(),b=$('remote-map-reset-open');b.disabled=!!why||mapReset.pending||mappingPending;b.textContent=mapReset.buttonLabel();b.title=why||'清空地图和记忆，重新建图';
     const n=$('remote-map-reset-note'),text=mapReset.phase==='confirm'?'':mapReset.text;n.hidden=!text;n.textContent=text;n.dataset.phase=mapReset.phase;}$('remote-mapping-stop').textContent=mappingPending?'等待保存回执…':'结束建图';
    $('remote-task-state').textContent=ok?stateLabel(snapshot.task?.state||'idle'):snapLate()?'数据延迟':'未连接';$('remote-task-message').textContent=ok?taskMessageText(snapshot.task):snapLate()?linkReason()+'，寻物暂不可用':'车栈未就绪，寻物不可用';$('remote-task-message').title=$('remote-task-message').textContent;
    $('remote-robot-status').textContent=ok?robotStatusText(snapshot.robot):snapLate()?'数据延迟':'车栈未连接';$('remote-speed').textContent=ok&&snapshot.robot?snapshot.robot.speed.toFixed(2)+' m/s':'未知';$('remote-memory-count').textContent=ok?snapshot.objects.length+' 个':'未知';$('remote-connection').textContent=ok?'车栈已连接':snapLate()?'数据延迟':cameraFresh()?'相机在线':'未连接';
    if(active()){const late=snapLate()?linkReason():'';$('connection-pill').textContent=ok?'实车车栈在线':late||(cameraFresh()?'实车相机在线':'正在连接实车');$('side-status').textContent=ok?'真实车栈':late?'车栈数据延迟':cameraFresh()?'真实相机':'设备未连接';$('source-notice-text').textContent=ok?'实车控制已接入 · 按住驾驶，松手停止；记忆位置需现场重新确认。':late?`${late} · 链路仍在线，等数据追上后自动恢复；期间控制锁定。`:cameraFresh()?'相机实时在线 · 车栈未启动，遥控和寻物暂不可用。':gateway?'正在连接 Nano 比赛网关。':'正在连接 Nano 相机与比赛转发桥。';$('source-notice').classList.toggle('error',!ok);$('notice-connect').textContent='连接详情';}
    if(snapshot&&renderHealth(snapshot)){$('car-notice').hidden=true;}   // 10-29 health panel replaces the line
    else if(snapshot){const el=$('car-notice'),t=$('car-notice-text'),text=(Array.isArray(snapshot.notices)?snapshot.notices:[]).map(n=>n&&typeof n.text==='string'?n.text:'').filter(Boolean).join('；');if(el&&t&&text!==t.textContent){t.textContent=text;el.hidden=!text;}}
    const candidates=snapshot?.task?.candidates||[],sig=JSON.stringify([candidates,snapshot?.memory?.scope,snapshot?.task?.id,ok,!!held]);$('remote-candidate-count').textContent=candidates.length;$('remote-candidates-open').disabled=!candidates.length;
    if(sig!==lastCandidateKey){lastCandidateKey=sig;$('remote-candidates').replaceChildren();for(const c of candidates){const b=document.createElement('button'),scope=snapshot.memory?.scope,taskId=snapshot.task.id;b.className='quiet-button';b.textContent=`${displayName(c)} · ${OBJECT_STATE_ZH[c.state]||'记忆'} · (${c.position[0].toFixed(2)}, ${c.position[1].toFixed(2)}) m`;b.disabled=!!blocker('select')||!!held;b.onclick=()=>{if(scope!==snapshot?.memory?.scope||taskId!==snapshot?.task?.id)return;void request('select',{candidateId:c.id,scope,taskId}).then(closeDrawer).catch(e=>record(e.message));};$('remote-candidates').append(b);}}
    $('remote-device-details').innerHTML=[['相机',cameraFresh()?'真实图像在线':'无新图像'],['转发桥',ok?'车栈数据有效':snapLate()?linkReason():connection==='connected'?'已连桥，车栈数据未就绪':'未连接'],['快照间隔（60 秒）',gapText()],['控制权',ok?caps.control:'未知'],['急停',ok&&typeof snapshot.estop?.latched==='boolean'?(snapshot.estop.latched?'已锁存':'已解除'):'未知'],['视频来源',gateway?'车栈 ROS 预览（网关推送）':ok?'车栈 ROS 预览':'Nano 已有相机服务（只读）']].map(([k,v])=>`<dt>${esc(k)}</dt><dd>${esc(v)}</dd>`).join('');
    onConnection(ok?'connected':'disconnected');
  }
  // 10-26 搜索热度图：preview/search 画上，done 留几秒后淡出，none（新任务无概率搜索）清掉
  let beliefTimer=null,belief=null;
  function hideBelief(){clearTimeout(beliefTimer);beliefTimer=null;belief=null;scene?.clearBelief();const n=$('remote-belief-note');if(n){n.hidden=true;n.textContent='';}}
  function applyBelief(b){
    clearTimeout(beliefTimer);beliefTimer=null;
    if(b.phase==='none'||(b.phase==='done'&&!doneHideDelay(b))){hideBelief();return;}
    belief=b;scene?.setBelief(b);
    const n=$('remote-belief-note');if(n){n.textContent=beliefCaption(b);n.hidden=false;n.dataset.phase=b.phase;}
    if(b.phase==='done')beliefTimer=setTimeout(()=>{scene?.fadeBelief(FADE_MS);beliefTimer=setTimeout(hideBelief,FADE_MS+50);},doneHideDelay(b));
  }
  function connect(url){disarm();hideBelief();adapter?.close();generation++;const gen=generation;snapshot=null;caps={};received=0;mappingPending=false;cloud=[];gateway=null;lastSnapAt=0;snapGaps.length=0;previewTimes.length=0;mappingChain=Promise.resolve();resyncAt=0;basemapId=loadingMap=null;mapGeneration++;clearMapping(true);scene?.clear();scene?.clearBasemap();adapter=new LiveAdapter(endpointURL(url));$('remote-endpoint').value=url;
    for(const type of ['connection','hello','snapshot','cloud','preview','mapping','belief','result','error'])adapter.addEventListener(type,e=>{if(gen!==generation)return;const d=e.detail;if(type==='preview'){showPushedPreview(d,gen);return;}if(type==='belief'){applyBelief(d);return;}if(type==='mapping'){void applyPushedMapping(d,gen);return;}if(type==='connection'){connection=d.state;if(connection!=='connected'){disarm();mapReset.onDisconnect();clearMapping(true);paintMap();}if(lastConnection!==connection){lastConnection=connection;record(connection==='connected'?'转发桥已连接':'转发桥等待连接');}}if(type==='hello'){caps=d.caps;gateway=d.hello?.gateway?.kind==='nano'&&d.hello.gateway.rover===false?d.hello.gateway:null;if(caps.control!=='controller')disarm();}if(type==='snapshot'){snapshot=d;received=performance.now();noteSnapshot(received);if(d.source==='live')noteFpga(d);if(d.source!=='live'){record('拒绝将非实时数据作为实车控制源');disarm();clearMapping(true);scene?.clear();}else{if(d.task?.state!==lastTask){lastTask=d.task?.state;record('任务：'+stateLabel(lastTask||'idle'));}paintMap();void loadBasemap(d);}}if(type==='cloud'){cloud=[];for(let i=0;i<d.header.count;i++)cloud.push([d.positions[i*3],d.positions[i*3+1],d.positions[i*3+2],d.colors[i*3]/255,d.colors[i*3+1]/255,d.colors[i*3+2]/255]);paintMap();}if(type==='result'){record(`${d.action}：${({done:'执行完成',unknown:'结果未知',rejected:'已拒绝',accepted:'车上接受'})[d.status]||d.status}${d.reason?' · '+d.reason:''}`);if(d.action==='mapping_stop')mappingPending=false;mapReset.onResult(d);}render();});if(document.hidden)adapter.close();else adapter.connect();
  }
  $('remote-bridge-form').onsubmit=e=>{e.preventDefault();try{const url=endpointURL($('remote-endpoint').value);if(new URL(url).host!==location.host)throw Error('请使用当前网站的 /bridge/ws，后台转发本机比赛桥');connect(url);}catch(e){record(e.message);}};
  async function pollCamera(){
    if(!active()||document.hidden){setTimeout(pollCamera,300);return;}
    if(viewMode==='third'){
      if(!fresh()&&!gateway)try{const r=await fetch('/rover/status',{cache:'no-store',signal:AbortSignal.timeout(1400)});const s=await r.json();if(!r.ok||s.source!=='live'||!s.camera||s.frameAgeMs>500)throw Error();camera=s;cameraAt=performance.now();}catch{camera=null;}
      render();setTimeout(pollCamera,500);return;
    }
    if(performance.now()-previewPushAt<PUSH_GRACE_MS){render();setTimeout(pollCamera,300);return;}   // pushed frames arriving
    let next=null;
    try{
      const useBridge=fresh()||!!gateway;let route='/bridge/preview.jpg';   // gateway: no /rover agent exists
      if(!useBridge){const r=await fetch('/rover/status',{cache:'no-store',signal:AbortSignal.timeout(1400)});if(!r.ok)throw Error('Nano 相机通道未连接');const s=await r.json();if(s.source!=='live'||!s.camera||!Number.isFinite(s.frameAgeMs)||s.frameAgeMs>500)throw Error('相机画面已过期');camera=s;cameraAt=performance.now();route='/rover/frame.jpg';}
      const r=await fetch(route+'?t='+Date.now(),{cache:'no-store',signal:AbortSignal.timeout(1400)});if(!r.ok)throw Error(gateway?'车栈预览暂未送达（网关在线）':'没有有效预览');const blob=await r.blob();if(blob.type!=='image/jpeg'||blob.size>(useBridge?163840:524288))throw Error('预览格式无效');next=URL.createObjectURL(blob);const img=$('first-person-image');await new Promise((resolve,reject)=>{img.onload=resolve;img.onerror=reject;img.src=next;});if(frameURL)URL.revokeObjectURL(frameURL);frameURL=next;next=null;fpvAr=null;paintFpv();camera={source:'live',camera:true};cameraAt=performance.now();img.hidden=false;$('camera-placeholder').hidden=true;$('camera-status').textContent='实时画面';$('camera-source').textContent=gateway?'车栈预览 · 轮询（推送暂断）':useBridge?'车栈预览 · 轮询（旧桥）':'Nano · Orbbec 实时相机';$('camera-time').textContent='收到画面 '+new Date().toLocaleTimeString('zh-CN');
    }catch(e){if(next)URL.revokeObjectURL(next);camera=null;$('first-person-image').hidden=true;fpvAr=null;paintFpv();$('camera-placeholder').hidden=false;$('camera-placeholder-text').textContent=e.message||'画面未就绪';$('camera-status').textContent='画面不可用';}
    render();setTimeout(pollCamera,150);
  }
  function noteSnapshot(t){
    if(lastSnapAt)snapGaps.push([t,t-lastSnapAt]);lastSnapAt=t;
    while(snapGaps.length&&t-snapGaps[0][0]>60000)snapGaps.shift();
  }
  function stats(){
    const now=performance.now();while(snapGaps.length&&now-snapGaps[0][0]>60000)snapGaps.shift();
    const g=snapGaps.map(x=>x[1]).sort((a,b)=>a-b),pct=q=>g.length?g[Math.min(g.length-1,Math.floor(q*g.length))]:null;
    const open=lastSnapAt?now-lastSnapAt:null;   // the gap still running counts towards max
    return {windowS:60,snapshots:g.length,snapshotGapP50Ms:pct(.5),snapshotGapP95Ms:pct(.95),snapshotGapMaxMs:g.length||open!==null?Math.max(g.length?g[g.length-1]:0,open||0):null,snapshotAgeMs:open,
      previewFps10s:previewTimes.length/10,gateway:!!gateway,chunks:adapter?.chunkStats?{...adapter.chunkStats}:null,
      mapping:{...mstats,epoch:store.epoch,seq:store.seq,voxels:store.count}};
  }
  function gapText(){const s=stats();return s.snapshotGapP95Ms===null?'—':`p95 ${(s.snapshotGapP95Ms/1000).toFixed(2)} s · 最大 ${(s.snapshotGapMaxMs/1000).toFixed(2)} s`;}
  setInterval(()=>{if(!fresh()){if(mapping){clearMapping();paintMap();}if(scene?.robot)scene.robot.visible=false;}render();const el=$('remote-link-stats');if(el){el.textContent=gapText();const s=stats();el.title=`最近 60 秒 ${s.snapshots} 个快照间隔；预览 ${s.previewFps10s.toFixed(1)} 帧/秒；三维 全量 ${s.mapping.full} 增量 ${s.mapping.delta} 重同步 ${s.mapping.resync}`;}},200);
  void pollCamera();
  void pollMapping();
  const api={connect,request,applyBelief,mapReset,get belief(){return belief;},get scene(){return scene;},get stats(){return stats();},get mapping(){return mapping;},get mappingError(){return mappingError;},get url(){return adapter?.url;},get snapshot(){return snapshot;},get connection(){return connection;}};
  window.__remoteLive=api;return api;
}
