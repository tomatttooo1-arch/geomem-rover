import { icon, hydrateIcons } from './icons.js';
import { STATES, TERMINAL, activeTask, matchObjects, endpointURL, canStart, snapshotFresh, esc } from './model.js';
import { DemoAdapter } from './demo.js';
import { LiveAdapter } from './transport.js';
import { SpaceView } from './scene.js';
import { setupRemote } from './remote.js';
import { mappingAgeLabel } from './live-mapping.js';
import { renderHealth } from './health-panel.js';
const $=id=>document.getElementById(id);
hydrateIcons();
let remoteController=null;
let snapshot=null, adapter=null, source='live', connection='disconnected', receivedAt=0, selectedId=null, currentPage='overview', busy=false, pendingState=null, generation=0, toastTimer, lastCards='';
let pageTransition=null, pageRequest=0, pageAnimation=null;
const reducedMotion=()=>matchMedia('(prefers-reduced-motion: reduce)').matches;
const routeOrder=['overview','remote','memory','history','device'];
function selectionTrack(container,selector,name){
  const glass=document.createElement('span');glass.className='selection-glass';glass.setAttribute('aria-hidden','true');
  if(name)glass.style.viewTransitionName=name;
  container.prepend(glass);let animation;
  const update=(smooth=false)=>{
    const selected=container.querySelector(selector);if(!selected)return;
    const previous=getComputedStyle(glass).transform;
    animation?.cancel();
    const parent=container.getBoundingClientRect(),box=selected.getBoundingClientRect();
    const transform=`translate(${box.left-parent.left-container.clientLeft}px,${box.top-parent.top-container.clientTop}px)`;
    Object.assign(glass.style,{width:`${box.width}px`,height:`${box.height}px`,transform});
    if(smooth&&!reducedMotion()&&previous!=='none')animation=glass.animate([{transform:previous},{transform}],{duration:480,easing:'cubic-bezier(.22,1,.36,1)'});
  };
  new ResizeObserver(()=>update()).observe(container);return update;
}
const moveNavigation=selectionTrack(document.querySelector('.sidebar nav'),'.nav-item.active','nav-selection');
const moveViewSelection=selectionTrack(document.querySelector('.segmented'),'button.selected');
let history=[];try{const saved=JSON.parse(localStorage.getItem('geomem-history')||'[]');if(Array.isArray(saved))history=saved.filter(r=>r&&typeof r.id==='string'&&typeof r.target==='string'&&TERMINAL.has(r.state)&&['demo','live'].includes(r.source)&&typeof r.updatedAt==='string').slice(0,50);}catch{}
let view=null;try{view=new SpaceView($('map-canvas'),$('map-labels'),openObject);view.declutterLabels=true;}catch(e){$('map-fallback').hidden=false;console.warn('WebGL unavailable',e.message);}
function toast(text){$('toast').textContent=text;$('toast').hidden=false;clearTimeout(toastTimer);toastTimer=setTimeout(()=>$('toast').hidden=true,6000);}
function relative(iso){if(!iso)return '观测时间未知';const seconds=Math.max(0,Math.floor((Date.now()-Date.parse(iso))/1000));return seconds<60?'刚刚见到':seconds<3600?`${Math.floor(seconds/60)} 分钟前见到`:seconds<86400?`${Math.floor(seconds/3600)} 小时前见到`:`${Math.floor(seconds/86400)} 天前见到`;}
function fresh(){return snapshotFresh(snapshot,receivedAt);}
function dateText(iso){return iso?new Date(iso).toLocaleString('zh-CN',{hour12:false}):'未知';}
function art(o){const type=o.className==='backpack'?'backpack':o.className==='mouse'?'mouse':o.className==='cup'?'cup':'box';return `<span class="object-art ${type}">${icon(type)}</span>`;}
function connected(){return connection==='connected';}
function setPage(page){
  const request=++pageRequest;
  pageTransition?.skipTransition();
  pageAnimation?.cancel();
  const changed=page!==currentPage;
  const reduce=reducedMotion();
  const direction=routeOrder.indexOf(page)>=routeOrder.indexOf(currentPage)?1:-1;
  document.documentElement.style.setProperty('--page-direction',direction);
  if(changed&&!reduce&&document.startViewTransition){
    pageTransition=document.startViewTransition(()=>{if(request===pageRequest)applyPage(page,false);});
    pageTransition.ready.catch(()=>{});
    pageTransition.finished.catch(()=>{});
    return pageTransition.updateCallbackDone;
  }
  if(changed&&!reduce){
    const content=document.querySelector('.content');
    pageAnimation=content.animate([{opacity:1,transform:'translateX(0)'},{opacity:0,transform:`translateX(${-direction*12}px)`}],{duration:150,easing:'ease-in',fill:'forwards'});
    return pageAnimation.finished.catch(()=>{}).then(()=>{
      if(request!==pageRequest)return;
      pageAnimation.cancel();applyPage(page,true);
      pageAnimation=content.animate([{opacity:0,transform:`translateX(${direction*24}px)`},{opacity:1,transform:'translateX(0)'}],{duration:440,easing:'cubic-bezier(.22,1,.36,1)'});
    });
  }
  applyPage(page,false);
  return Promise.resolve();
}
function applyPage(page,smoothNavigation=false){
  document.body.classList.toggle('remote-active',page==='remote');
  document.body.classList.toggle('overview-active',page==='overview');
  document.dispatchEvent(new Event('geomem-page-change'));
  if(page==='remote'){
    currentPage=page;
    for(const id of ['overview-page','memory-section','history-page','device-page'])$(id).hidden=true;
    $('remote-page').hidden=false;
    $('breadcrumb').textContent='比赛控制台';$('page-title').textContent='看见现场，掌控每一次寻找';$('page-subtitle').textContent='遥控、三维记忆与寻物，集中在当前工作区。';
    document.querySelectorAll('[data-page]').forEach(b=>{b.classList.toggle('active',b.dataset.page===page);if(b.dataset.page===page)b.setAttribute('aria-current','page');else b.removeAttribute('aria-current');});
    moveNavigation(smoothNavigation);location.hash=page;return;
  }
  $('remote-page').hidden=true;
  currentPage=page;const labels={overview:['寻物工作台','空间记忆控制台','查看空间，选择目标，跟踪每一次寻找。'],memory:['空间记忆','物品的位置，记在这里。','查看上次观测的位置、时间与识别分数。'],history:['任务记录','每一次寻找，都有记录。','回看任务结果，让下一次出发更有把握。'],device:['设备与连接','随时了解，小车的状态。','确认数据来源，让每一次操作都有反馈。']};
  $('breadcrumb').textContent=labels[page][0];$('page-title').textContent=labels[page][1];$('page-subtitle').textContent=labels[page][2];
  document.querySelectorAll('[data-page]').forEach(b=>{b.classList.toggle('active',b.dataset.page===page);if(b.dataset.page===page)b.setAttribute('aria-current','page');else b.removeAttribute('aria-current');});
  moveNavigation(smoothNavigation);
  $('overview-page').hidden=page!=='overview';$('memory-section').hidden=page!=='memory';$('history-page').hidden=page!=='history';$('device-page').hidden=page!=='device';$('all-memory').hidden=page==='memory';
  renderObjects();renderHistory();renderConnection();requestAnimationFrame(()=>view?.resize());location.hash=page;window.scrollTo({top:0,behavior:'instant'});
}
document.querySelectorAll('[data-page]').forEach(b=>b.onclick=()=>setPage(b.dataset.page));
$('all-memory').onclick=()=>setPage('memory');
$('overview-memory').onclick=()=>setPage('memory');
function updateConnectionFields(){ $('endpoint').disabled=document.querySelector('input[name="source"]:checked').value==='demo'; }
document.querySelectorAll('input[name="source"]').forEach(input=>input.addEventListener('change',updateConnectionFields));
function openSettings(){document.querySelector(`input[name="source"][value="${source}"]`).checked=true;updateConnectionFields();$('connection-error').textContent='';$('settings-dialog').showModal();}
for(const id of ['source-button','notice-connect','settings-button','device-connect'])$(id).onclick=openSettings;
$('help-button').onclick=()=>$('help-dialog').showModal();
$('settings-help').onclick=()=>closeDialog($('settings-dialog')).then(()=>$('help-dialog').showModal());
function closeDialog(dialog){
  if(!dialog.open)return Promise.resolve();
  if(dialog.closing)return dialog.closing;
  if(reducedMotion()){dialog.close();return Promise.resolve();}
  const style=getComputedStyle(dialog),from={opacity:style.opacity,transform:style.transform};
  dialog.classList.add('is-closing');
  const animation=dialog.animate([from,{opacity:0,transform:'translateY(12px) scale(.975)'}],{duration:190,easing:'cubic-bezier(.4,0,1,1)',fill:'forwards'});
  dialog.closing=animation.finished.catch(()=>{}).then(()=>{dialog.close();animation.cancel();dialog.classList.remove('is-closing');dialog.closing=null;});
  return dialog.closing;
}
document.querySelectorAll('dialog').forEach(d=>d.addEventListener('cancel',e=>{e.preventDefault();closeDialog(d);}));
document.querySelectorAll('.close-dialog').forEach(b=>b.onclick=()=>closeDialog(b.closest('dialog')));
document.querySelectorAll('dialog').forEach(d=>d.addEventListener('click',e=>{if(e.target===d){const r=d.getBoundingClientRect();if(e.clientX<r.left||e.clientX>r.right||e.clientY<r.top||e.clientY>r.bottom)closeDialog(d);}}));
try{$('endpoint').value=localStorage.getItem('geomem-endpoint')||'';}catch{}
$('connection-form').onsubmit=e=>{
  e.preventDefault();const mode=document.querySelector('input[name="source"]:checked').value;
  try {
    const url=mode==='live'?endpointURL($('endpoint').value.trim(),location.protocol==='https:'):null;
    if(activeTask(snapshot?.task)&&source==='live'&&!confirm('切换数据源不会停止实车当前任务。确认小车安全后再切换，是否继续？'))return;
    if(url)try{localStorage.setItem('geomem-endpoint',url);}catch{}
    connect(mode,url);closeDialog($('settings-dialog'));
  }catch(error){$('connection-error').textContent=error.message;}
};
function connect(mode,url){
  if(mode!=='live'){toast('当前控制台只连接真实设备');return;}
  source='live';snapshot=null;receivedAt=0;connection='connecting';view?.clear();view?.setSource('live');
  remoteController?.connect(url||`ws://${location.host}/bridge/ws`);render();
}

// 10-28 车栈开机提示（snapshot.notices，如地面标定退回：相机角度测量异常…）：醒目红条，断线不清（开机事实）
function showCarNotices(snap){
  const el=document.getElementById('car-notice'),t=document.getElementById('car-notice-text');if(!el||!t)return;
  if(renderHealth(snap)){el.hidden=true;return;}   // 10-29 health panel (its ground calib check replaces this line)
  const text=(Array.isArray(snap?.notices)?snap.notices:[]).map(n=>n&&typeof n.text==='string'?n.text:'').filter(Boolean).join('；');
  if(snap&&text!==t.textContent){t.textContent=text;el.hidden=!text;}
}
function renderConnection(){
  if(currentPage!=='remote'){
  if(currentPage!=='remote')document.querySelectorAll('[data-drive]').forEach(b=>b.disabled=true);
  $('drive-status').textContent='尚没有完成底盘接口联调，运动控制锁定';
  $('remote-connection').textContent=source==='demo'?'演示模式 · 不控制实车':connected()?'桥接已连接 · 遥控未验证':'未连接';
  $('remote-robot-status').textContent=source==='live'&&connected()&&fresh()?snapshot?.robot?.status||'未知':'未知';
  $('remote-speed').textContent=source==='live'&&connected()&&fresh()&&snapshot?.robot?`${snapshot.robot.speed.toFixed(2)} m/s`:'—';
  }
  const isFresh=fresh(), label=source==='demo'?'演示模式':connection==='connecting'?'正在连接':!connected()?'连接已断开':!snapshot?'等待首帧数据':!isFresh?'数据已过期':'实车已连接';
  if(currentPage!=='remote'){
  $('connection-pill').innerHTML=`<span class="status-dot"></span>${label}`;$('connection-pill').classList.toggle('warning',source==='live'&&(!connected()||!isFresh));$('side-status').textContent=label;
  $('source-notice').classList.toggle('error',source==='live'&&(!connected()||!isFresh));
  $('source-notice-text').textContent=source==='demo'?'正在体验演示空间。地图、物品和小车运动均为模拟数据。':!connected()?'实车未连接。无法发送任务；屏幕保留的数据不能代表小车当前状态。':!snapshot?'已连接桥接服务，正在等待有效数据。':!isFresh?'数据超过 5 秒未更新或时钟不一致，已暂停显示为实时状态并禁止新任务。':'正在显示实车回传数据。任务结果以小车现场确认反馈为准。';
  $('notice-connect').innerHTML=(source==='demo'?'连接实车':'连接设置')+icon('arrow');
  }
  showCarNotices(source==='live'?snapshot:null);
  $('map-status').textContent=source==='demo'?'演示地图 · 已载入':!snapshot?'等待实车地图':!connected()||!isFresh?'地图连接已断开':view?.mappingCloud?`RTAB-Map ${view.mode==='local'?'局部三维':'累积三维'} · ${view.mappingVisibleCount.toLocaleString()} 点 · ${mappingAgeLabel(view.mappingMeta)}`:'局部障碍栅格 · 累积三维待连接';
  $('map-title').innerHTML=`${esc(snapshot?.map.name||'等待空间地图')} <span class="subtle-tag">${source==='demo'?'演示空间':'实车空间'}</span>`;
  $('metric-fresh').textContent=snapshot?(!connected()?'已断线':!isFresh?'数据过期':`${Math.max(0,Math.floor((Date.now()-receivedAt)/1000))} 秒前`):'等待数据';
  $('metric-robot').textContent=snapshot?.robot?`${source==='demo'?'演示 · ':''}${!connected()||!isFresh?'状态未知':snapshot.robot.status}`:'未知';
  $('metric-position').textContent=snapshot?.robot?(connected()&&isFresh?`${snapshot.robot.position[0].toFixed(2)}, ${snapshot.robot.position[1].toFixed(2)} m`:'历史位置 · 非实时'):'未收到位姿';
  $('metric-objects').textContent=snapshot?`${snapshot.objects.length} 件物品`:'等待数据';
  $('device-details').innerHTML=[['数据来源',source==='demo'?'本地演示数据':'实车 M8 桥接服务'],['连接状态',label],['服务地址',source==='demo'?'无需网络连接':adapter?.url||'未设置'],['地图坐标系',snapshot?`${snapshot.map.frame} · ${snapshot.map.units}`:'未知'],['定位状态',snapshot?.robot?(connected()&&isFresh?(snapshot.robot.localized?'已定位':'未定位'):'未知 · 数据失效'):'未知'],['最后数据时间',dateText(snapshot?.timestamp)],['当前任务',snapshot?.task?`${STATES[snapshot.task.state]}${!connected()||!isFresh?'（历史状态）':''}`:'无'],['电量 / 相机状态','当前协议未提供']].map(([k,v])=>`<dt>${k}</dt><dd>${esc(v)}</dd>`).join('');
  const ready=canStart(snapshot,connected(),isFresh)&&!busy&&!pendingState;
  $('find-button').disabled=!snapshot||!connected()||!isFresh||busy;
  $('start-task').disabled=!ready||!snapshot?.objects.some(o=>o.id===selectedId);
  const unavailable = !connected()?'连接尚未就绪，请先连接实车或切回演示。':!snapshot?'正在等待有效的地图和位姿数据。':!isFresh?'数据已过期，请检查连接或等待数据恢复。':!snapshot.robot?.localized?'小车尚没有完成定位，暂时不能发起寻物。':activeTask(snapshot.task)?'已有任务正在执行，请先完成或停止当前任务。':'';
  const feedback=busy?'正在发送请求，请稍候。':pendingState?'请求已受理，等待小车状态确认。':unavailable;
  if($('query-hint').textContent!==feedback)$('query-hint').textContent=feedback;
  if($('start-hint').textContent!==feedback)$('start-hint').textContent=feedback;
  document.querySelector('.mission-card').setAttribute('aria-busy',String(busy||!!pendingState));
  $('find-button').setAttribute('aria-describedby','query-hint');
  $('start-task').setAttribute('aria-describedby','start-hint');
  $('pause-task').hidden=source==='live';
  $('pause-task').disabled=!connected()||!isFresh||busy||!!pendingState||!activeTask(snapshot?.task);
  // Cancellation remains available on an open connection even if telemetry is stale.
  $('stop-task').disabled=!connected()||busy||!activeTask(snapshot?.task);
  if(source==='live')$('map-stage').style.opacity=snapshot&&(!connected()||!isFresh)?'.55':'1';else $('map-stage').style.opacity='1';
}
function renderObjects(){
  const objects=snapshot?.objects||[];const q=$('memory-filter').value.trim().toLowerCase();let list=objects.filter(o=>`${o.name} ${o.className} ${o.location}`.toLowerCase().includes(q));if(currentPage==='overview')list=list.slice(0,3);
  const key=JSON.stringify(list.map(o=>({...o,lastSeen:relative(o.lastSeen)})));$('object-count').textContent=objects.length;$('memory-count').textContent=objects.length;$('memory-empty').hidden=!!list.length;
  if(key===lastCards)return;lastCards=key;
  $('objects-grid').innerHTML=list.map(o=>`<button class="object-card" data-object="${esc(o.id)}" aria-label="查看${esc(o.name)} ${esc(o.id)}">${art(o)}<span class="object-copy"><span class="object-name">${esc(o.name)}<span class="object-id">${esc(o.id)}</span></span><span class="object-location">${esc(o.location)}</span><small>${relative(o.lastSeen)} · ${source==='demo'?'演示记忆':'历史观测'}</small></span><span class="card-arrow">${icon('arrow')}</span></button>`).join('');
  $('objects-grid').querySelectorAll('[data-object]').forEach(b=>b.onclick=()=>openObject(b.dataset.object));
}
$('memory-filter').oninput=renderObjects;
function renderTask(){
  const t=snapshot?.task;$('task-empty').hidden=!!t;$('task-active').hidden=!t;const state=t?.state||'idle';
  const stateLabel=STATES[state];
  const badge=$('task-state');
  if(badge.textContent!==stateLabel)badge.textContent=stateLabel;
  if(badge.dataset.state!==state){badge.dataset.state=state;const panel=document.querySelector('.mission-card');panel.classList.remove('state-changed');requestAnimationFrame(()=>panel.classList.add('state-changed'));}
  document.querySelector('.task-controls').hidden=!activeTask(t);
  if(!t)return;
  $('task-target').textContent=`${snapshot.objects.find(o=>o.id===t.targetId)?.name||t.targetId||'等待寻物指令'} · ${source==='demo'?'演示任务':'实车任务'}`;
  const steps=[['searching','查询物体记忆'],['planning','规划可达停靠点'],['navigating','前往目标附近'],['confirming','现场观测确认']];let idx=steps.findIndex(([key])=>key===t.state);if(t.state==='succeeded')idx=4;
  $('task-steps').innerHTML=steps.map(([key,label],i)=>`<li class="${i<idx?'done':i===idx?'current':''}">${label}${i===idx?' · 进行中':''}</li>`).join('');
  $('task-message').textContent=t.message;$('pause-task').textContent=t.state==='paused'?'继续任务':'暂停';document.querySelector('.task-controls').hidden=!activeTask(t);
}
function render(){renderConnection();renderObjects();renderTask();}
function openObject(id){
  const o=snapshot?.objects.find(o=>o.id===id);if(!o)return;selectedId=id;view?.focus(id);
  $('detail-body').innerHTML=`<div class="detail-hero">${art(o)}<div><h2>${esc(o.name)}</h2><p>${esc(o.id)} · ${source==='demo'?'演示物体记忆':'物体记忆'}</p></div></div><dl class="detail-info"><dt>上次所在位置</dt><dd>${esc(o.location)}</dd><dt>空间坐标</dt><dd>${o.position.map(n=>n.toFixed(2)).join(', ')} m</dd><dt>最后观察时间</dt><dd>${dateText(o.lastSeen)}</dd><dt>识别分数</dt><dd>${Number.isFinite(o.confidence)?o.confidence.toFixed(2):'未知'}${source==='demo'?'（模拟值）':''} · 不等于真实概率</dd><dt>现场确认</dt><dd>${snapshot.task?.targetId===id&&snapshot.task.state==='succeeded'?(source==='demo'?'演示确认完成':'任务回传已确认'):'尚未进行本次现场确认'}</dd></dl>`;
  renderConnection();if(!$('detail-dialog').open)$('detail-dialog').showModal();
}
$('focus-object').onclick=()=>{closeDialog($('detail-dialog')).then(()=>setPage('overview')).then(()=>requestAnimationFrame(()=>{view?.focus(selectedId);document.querySelector('.map-card').scrollIntoView({block:'start',behavior:reducedMotion()?'instant':'smooth'});}));};
$('search-form').onsubmit=e=>{e.preventDefault();const q=$('search-input').value.trim();if(!q){$('search-input').focus();toast('先输入想找的物品名称');return;}const matches=matchObjects(snapshot?.objects||[],q);if(matches.length===1){$('search-results').replaceChildren();openObject(matches[0].id);}else if(!matches.length){$('search-results').textContent=`记忆中没有找到“${q}”。试试类别名称，或先补充扫描。`;}else{$('search-results').innerHTML=`找到 ${matches.length} 件物品，请选择：`+matches.map(o=>`<button class="result-button" data-result="${esc(o.id)}">${esc(o.name)} · ${esc(o.location)} · ${esc(o.id)}</button>`).join('');$('search-results').querySelectorAll('[data-result]').forEach(b=>b.onclick=()=>openObject(b.dataset.result));}};
document.querySelectorAll('[data-query]').forEach(b=>b.onclick=()=>{$('search-input').value=b.dataset.query;$('search-input').focus();});
function stateMatches(p){const t=snapshot?.task;if(!t)return false;return p.action==='find'?t.id!==p.oldId&&t.targetId===p.targetId:p.action==='cancel'?t.state==='cancelled':p.action==='pause'?t.state==='paused':t.state!=='paused';}
async function sendCommand(action,payload={}){
  if(busy)return;if(action==='find'&&!canStart(snapshot,connected(),fresh())){toast('当前无法开始寻物，请检查连接、定位和已有任务');return;}
  if(action!=='cancel'&&(!fresh()||!connected())){toast('数据不可用，请先恢复连接');return;}
  const gen=generation;busy=true;const wait={action,oldId:snapshot?.task?.id,targetId:payload.targetId,deadline:Date.now()+10000};renderConnection();
  try{await adapter.command(action,payload);if(gen!==generation)return;if(source==='live'&&!stateMatches(wait)){pendingState=wait;toast('请求已受理，正在等待小车状态更新。');}else toast(source==='demo'?'演示任务已更新':'小车已回传新的任务状态');}
  catch(e){if(gen===generation)toast(e.message);}
  finally{if(gen===generation){busy=false;render();}}
}
$('start-task').onclick=()=>{const o=snapshot?.objects.find(x=>x.id===selectedId);closeDialog($('detail-dialog')).then(()=>setPage('remote')).then(()=>{if(o){$('remote-query').value=o.name;$('remote-query').focus();}});};
$('pause-task').onclick=()=>sendCommand(snapshot?.task?.state==='paused'?'resume':'pause',{taskId:snapshot?.task?.id});
$('stop-task').onclick=()=>sendCommand('cancel',{taskId:snapshot?.task?.id});
function rememberTask(){const t=snapshot?.task;if(!t||!TERMINAL.has(t.state)||history.some(r=>r.id===t.id&&r.source===source))return;history.unshift({...t,source,target:snapshot.objects.find(o=>o.id===t.targetId)?.name||t.targetId,map:snapshot.map.name});history=history.slice(0,50);try{localStorage.setItem('geomem-history',JSON.stringify(history));}catch{toast('浏览器未允许保存记录，本次记录仅在当前页面保留');}renderHistory();}
function renderHistory(){$('history-list').innerHTML=history.length?history.map(r=>`<div class="history-row"><span>${esc(r.target)}<small>${esc(r.map)} · ${r.source==='demo'?'演示':'实车'}</small></span><span>${dateText(r.updatedAt)}<small>${esc(r.message)}</small></span><span class="status-tag" data-state="${esc(r.state)}">${STATES[r.state]}</span></div>`).join(''):'<div class="empty-state">还没有结束的任务。完成一次寻找后，记录会出现在这里。</div>';}
$('export-history').onclick=()=>{const blob=new Blob([JSON.stringify({exportedAt:new Date().toISOString(),records:history},null,2)],{type:'application/json'});const a=document.createElement('a');const url=URL.createObjectURL(blob);a.href=url;a.download='geomem-task-history.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);toast('任务记录已导出');};
function changeView(mode){view?.setMode(mode);for(const key of ['global','local']){$(`${key}-view`).classList.toggle('selected',mode===key);$(`${key}-view`).setAttribute('aria-pressed',String(mode===key));}moveViewSelection(true);$('map-hint').textContent=mode==='local'?'跟随小车 · 半径 3.2 m 的局部点云':'拖动旋转 · 滚轮缩放 · 点击物品查看';}
$('global-view').onclick=()=>changeView('global');$('local-view').onclick=()=>changeView('local');
$('zoom-in').onclick=()=>view?.zoom(.85);$('zoom-out').onclick=()=>view?.zoom(1.18);$('reset-view').onclick=()=>view?.reset();
$('fullscreen').onclick=async()=>{try{if(document.fullscreenElement)await document.exitFullscreen();else await document.querySelector('.map-card').requestFullscreen();}catch{toast('当前浏览器不支持全屏地图');}};
document.addEventListener('keydown',e=>{if(e.key==='Escape'){$('layers-panel').hidden=true;$('layers-button').setAttribute('aria-expanded','false');}});
document.addEventListener('click',e=>{if(!e.target.closest('#layers-panel,#layers-button')){$('layers-panel').hidden=true;$('layers-button').setAttribute('aria-expanded','false');}});
$('layers-button').onclick=()=>{$('layers-panel').hidden=!$('layers-panel').hidden;$('layers-button').setAttribute('aria-expanded',String(!$('layers-panel').hidden));};
for(const layer of ['geometry','objects','path'])$(`layer-${layer}`).onchange=e=>view?.setLayer(layer,e.target.checked);
document.addEventListener('keydown',e=>{if(e.key==='/'&&!['INPUT','TEXTAREA'].includes(document.activeElement.tagName)&&!document.querySelector('dialog[open]')){e.preventDefault();setPage('overview').then(()=>$('search-input').focus());}});
window.addEventListener('hashchange',()=>{const key=location.hash.slice(1);if(routeOrder.includes(key)&&key!==currentPage)setPage(key);});
setInterval(()=>{if(pendingState&&Date.now()>pendingState.deadline){pendingState=null;toast('尚未收到对应任务状态，执行结果未知，请核对小车。');}renderConnection();renderObjects();},1000);
view?.clear();view?.setSource('live');
remoteController=setupRemote({
  onSnapshot:s=>{snapshot=s;receivedAt=Date.now();view?.update(s);rememberTask();render();},
  onMapping:data=>{if(data?.store)view?.setMappingStore(data);else if(data)view?.setMappingCloud(data);else view?.clearMappingCloud();renderConnection();},
  onConnection:state=>{connection=state;renderConnection();}
});
window.__overviewMap={get mode(){return view?.mode;},get mapping(){return view?.mappingMeta;},get visibleCount(){return view?.mappingVisibleCount;},get geometryVisible(){return !!view?.mappingCloud?.visible;}};
adapter={get url(){return remoteController.url;},command:(action,payload)=>remoteController.request(action,payload)};
setPage(routeOrder.includes(location.hash.slice(1))?location.hash.slice(1):'remote');
remoteController.connect(`ws://${location.host}/bridge/ws`);
$('camera-fullscreen').onclick=()=>{if(!document.fullscreenElement)document.documentElement.requestFullscreen?.().catch(()=>toast('浏览器未允许全屏'));else document.exitFullscreen?.();};
// Unregister the previous offline cache so live controls always load the current code.
if('serviceWorker' in navigator)navigator.serviceWorker.getRegistrations().then(rs=>Promise.all(rs.map(r=>r.unregister()))).catch(()=>{});
