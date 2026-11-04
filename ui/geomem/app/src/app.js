import { icon, hydrateIcons } from './icons.js';
import { TERMINAL, activeTask, taskDisplayState, stateLabel, manualTargetPosition, matchObjects, topClasses, endpointURL, defaultEndpoint, snapshotFresh, secondsSince, relativeText, startBlocker, ODOM_BADGE, ODOM_FIND_HINT, sourceBanner, sourceZh, sourceDetailOf, odomTier, displayName, classZh, reasonText, OBJECT_STATE_ZH, POSE_ZH, esc } from './model.js';
import { DemoAdapter } from './demo.js';
import { LiveAdapter } from './transport.js';
import { SpaceView } from './scene.js';
const $=id=>document.getElementById(id);
hydrateIcons();
// receivedAt 用 performance.now()（单调钟）；界面任何新鲜度判断都不读墙钟。
let snapshot=null, adapter=null, source='demo', connection='disconnected', connectionInfo={}, receivedAt=0, selectedId=null, selectedScope=null, currentPage='overview', busy=false, pendingCmd=null, generation=0, toastTimer, lastCards='', caps=null, hello=null;
let clouds=new Map(), shownCloudKey='', lastMapId, lastScope, lastQuery='', reachedStep=new Map(), framedOnce=false, cloudWarned='';
let pageTransition=null, pageRequest=0, pageAnimation=null;
const reducedMotion=()=>matchMedia('(prefers-reduced-motion: reduce)').matches;
const routeOrder=['overview','memory','history','device'];
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
let history=[];try{const saved=JSON.parse(localStorage.getItem('geomem-history')||'[]');if(Array.isArray(saved))history=saved.filter(r=>r&&r.v===2&&typeof r.id==='string'&&typeof r.target==='string'&&TERMINAL.has(r.state)&&['demo','live'].includes(r.source)&&typeof r.recordedAt==='string'&&typeof r.message==='string').slice(0,50);}catch{}
let view=null;try{view=new SpaceView($('map-canvas'),$('map-labels'),openObject,displayName);}catch(e){$('map-fallback').hidden=false;console.warn('WebGL unavailable',e.message);}
function toast(text){$('toast').textContent=text;$('toast').hidden=false;clearTimeout(toastTimer);toastTimer=setTimeout(()=>$('toast').hidden=true,6000);}
function fresh(){return snapshotFresh(snapshot,receivedAt);}
// 「多久前见到」= serverTime − lastSeen（车上同一时间轴）；data-seconds 给自动化核对用。
function seenHTML(lastSeen){const s=secondsSince(snapshot,lastSeen);return `<span class="seen" data-seconds="${s===null?'':s.toFixed(2)}">${relativeText(s)}</span>`;}
function art(o){const type=['backpack','mouse','cup'].includes(o.className)?o.className:'box';return `<span class="object-art ${type}">${icon(type)}</span>`;}
function connected(){return connection==='connected';}
const CONTROL_ZH={controller:'本界面持有控制权',observer:'只读（另一台电脑持有控制权）',none:'无控制权'};
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
  currentPage=page;const labels={overview:['寻物工作台','空间记忆控制台','查看空间，选择目标，跟踪每一次寻找。'],memory:['空间记忆','物品的位置，记在这里。','查看上次观测的位置、时间与识别分数。'],history:['任务记录','每一次寻找，都有记录。','回看任务结果，让下一次出发更有把握。'],device:['设备与连接','随时了解，小车的状态。','确认数据来源，让每一次操作都有反馈。']};
  $('breadcrumb').textContent=labels[page][0];$('page-title').textContent=labels[page][1];$('page-subtitle').textContent=labels[page][2];
  document.querySelectorAll('[data-page]').forEach(b=>{b.classList.toggle('active',b.dataset.page===page);if(b.dataset.page===page)b.setAttribute('aria-current','page');else b.removeAttribute('aria-current');});
  moveNavigation(smoothNavigation);
  $('overview-page').hidden=page!=='overview';$('memory-section').hidden=!['overview','memory'].includes(page);$('history-page').hidden=page!=='history';$('device-page').hidden=page!=='device';$('all-memory').hidden=page==='memory';
  renderObjects();renderHistory();renderConnection();requestAnimationFrame(()=>view?.resize());location.hash=page;window.scrollTo({top:0,behavior:'instant'});
}
document.querySelectorAll('[data-page]').forEach(b=>b.onclick=()=>setPage(b.dataset.page));
$('all-memory').onclick=()=>setPage('memory');
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
try{$('endpoint').value=localStorage.getItem('geomem-endpoint')||defaultEndpoint();}catch{$('endpoint').value=defaultEndpoint();}
$('connection-form').onsubmit=e=>{
  e.preventDefault();const mode=document.querySelector('input[name="source"]:checked').value;
  try {
    const url=mode==='live'?endpointURL($('endpoint').value.trim(),location.protocol==='https:'):null;
    if(activeTask(snapshot?.task)&&source==='live'&&!confirm('切换数据源不会停止实车当前任务。确认小车安全后再切换，是否继续？'))return;
    if(url)try{localStorage.setItem('geomem-endpoint',url);}catch{}
    connect(mode,url);closeDialog($('settings-dialog'));
  }catch(error){$('connection-error').textContent=error.message;}
};
function resetData(){snapshot=null;receivedAt=0;busy=false;pendingCmd=null;lastCards='';selectedId=null;selectedScope=null;clouds=new Map();shownCloudKey='';lastMapId=undefined;lastScope=undefined;lastQuery='';framedOnce=false;cloudWarned='';$('search-results').replaceChildren();}
function connect(mode,url){
  generation++;const gen=generation;if(adapter)adapter.close();source=mode;connection='connecting';connectionInfo={};caps=null;hello=null;resetData();
  $('detail-dialog').close();view?.clear();view?.setSource(source);
  adapter=mode==='demo'?new DemoAdapter():new LiveAdapter(url);
  adapter.addEventListener('connection',e=>{if(gen!==generation)return;connection=e.detail.state;connectionInfo=e.detail;if(connection!=='connected'){busy=false;}render();});
  adapter.addEventListener('error',e=>{if(gen!==generation)return;toast(e.detail);});
  adapter.addEventListener('hello',e=>{if(gen!==generation)return;hello=e.detail.hello;caps=e.detail.caps;render();});
  adapter.addEventListener('cloud',e=>{if(gen!==generation)return;const c=e.detail;if(c.header.rev==null){clouds=new Map();syncCloud();return;}clouds.set(c.header.rev,c);while(clouds.size>3)clouds.delete(clouds.keys().next().value);syncCloud();});
  adapter.addEventListener('result',e=>{if(gen!==generation)return;onResult(e.detail);});
  adapter.addEventListener('snapshot',e=>{if(gen!==generation)return;onSnapshot(e.detail);});
  render();adapter.connect();
}
function onSnapshot(s){
  const mapChanged=lastMapId!==undefined&&(s.map.id??null)!==lastMapId;
  const scopeChanged=lastScope!==undefined&&(s.memory?.scope??null)!==lastScope;
  snapshot=s;receivedAt=performance.now();
  if(mapChanged){clouds=new Map([...clouds].filter(([,c])=>(c.header.mapId??null)===(s.map.id??null)));selectedId=null;if(view)view.selected=null;view?.clear();view?.setSource(source);framedOnce=false;toast('地图已更换，已清除选择与视角。');}
  if(scopeChanged){selectedId=null;selectedScope=null;if(view)view.selected=null;$('search-results').replaceChildren();if($('detail-dialog').open)$('detail-dialog').close();toast('小车记忆已重置（物品重新编号），已清除选择，请重新选择。');}
  lastMapId=s.map.id??null;lastScope=s.memory?.scope??null;
  if(s.task&&s.task.id&&!TERMINAL.has(s.task.state)){const idx=stepIndex(s.task.state);if(idx>=0)reachedStep.set(s.task.id,Math.max(idx,reachedStep.get(s.task.id)??-1));}
  view?.update(s);syncCloud();
  if(!framedOnce){framedOnce=true;view?.autoFrame();}
  rememberTask();render();
}
// 只画与当前快照 cloudRev、坐标系、地图 id 一致的点云；云先到、快照后到也不丢（§2.4）。
function syncCloud(){
  if(!snapshot||!view)return;
  const m=snapshot.map,c=m.cloudRev?clouds.get(m.cloudRev):null;
  let ok=!!c,why='';
  if(c&&c.header.frame!==m.frame){ok=false;why='点云坐标系与地图不一致，未显示';}
  else if(c&&(c.header.mapId??null)!==(m.id??null)){ok=false;why='点云与当前地图不一致，未显示';}
  else if(c&&m.placeholder&&c.header.source==='grid'&&String(c.header.rev).startsWith('global:')){ok=false;}
  const key=ok?`${m.cloudRev}|${c.header.count}`:'';
  if(why&&cloudWarned!==m.cloudRev){cloudWarned=m.cloudRev;toast(why);}
  if(key===shownCloudKey)return;
  shownCloudKey=key;view.setCloud(ok?c:null);view.autoFrame();
}
function renderConnection(){
  const isFresh=fresh(),s=snapshot,src=s?.source;
  const label=source==='demo'?'演示模式':connection==='connecting'?'正在连接':connection==='reconnecting'?'正在重连':connection==='waiting'?`连接中断 · ${Math.round((connectionInfo.delayMs||0)/1000)} 秒后重连`:!connected()?'连接已断开':!s?'等待首帧数据':!isFresh?'数据已过期':src==='replay'?'回放 · 非实时':src==='sim'?'仿真数据':src==='inconsistent'?'数据来源不一致':src==='unknown'?'已连接 · 来源未标注':'实车已连接';
  const bad=source==='live'&&(!connected()||!isFresh||src==='inconsistent');
  $('connection-pill').innerHTML=`<span class="status-dot"></span>${label}`;$('connection-pill').classList.toggle('warning',bad);$('side-status').textContent=label;
  $('source-notice').classList.toggle('error',bad);
  $('source-notice-text').textContent=source==='demo'?'正在体验演示空间。地图、物品和小车运动均为模拟数据。':!connected()?'实车未连接，正在自动重连。无法发送任务；屏幕保留的数据不能代表小车当前状态。':!s?'已连接桥接服务，正在等待有效数据。':!isFresh?'超过 2 秒没有收到新数据（或桥报告与小车的链路不通），已停止显示为实时状态并禁止新任务。':'正在显示小车回传数据。任务结果以小车现场确认反馈为准。';
  $('notice-connect').innerHTML=(source==='demo'?'连接实车':'连接设置')+icon('arrow');
  // 来源横幅与三维水印（回放/仿真/来源不一致常驻）。
  const banner=source==='live'&&s&&src!=='live'?sourceBanner(src,sourceDetailOf(s)):null;
  $('source-banner').hidden=!banner;if(banner){$('source-banner').textContent=banner[0];$('source-banner').dataset.level=banner[1];}
  const mark=source==='live'&&s?({replay:'回放 · 非实时',sim:'仿真',inconsistent:'来源不一致'}[src]):null;
  $('map-watermark').hidden=!mark;$('map-watermark').textContent=mark||'';
  const odom=odomTier(s);
  $('map-status').textContent=source==='demo'?'演示地图 · 已载入':!s?'等待实车地图':!connected()||!isFresh?'历史画面 · 非实时':s.map.placeholder?'替身地图（空白）未画 · 只画小车周围障碍':!s.map.cloudRev?'暂无点云 · 只显示物品与小车':!shownCloudKey?'等待点云':odom?'里程坐标 · 数据有效':'实车地图 · 数据有效';
  $('map-title').innerHTML=`${esc(s?.map.name||(s?(source==='demo'?'演示空间':'实车地图'):'等待空间地图'))} <span class="subtle-tag">${source==='demo'?'演示空间':'实车空间'}</span>`;
  $('map-frame').textContent=s?`${s.map.frame} · 米`:'— · 米';
  // 位姿质量徽标：map / odometry / none，位姿年龄 >1 s 标过时。
  const pose=s?.robot?.pose, poseStale=Number.isFinite(s?.robot?.poseAgeMs)&&s.robot.poseAgeMs>1000;
  const badge=!s?null:s.robot?[POSE_ZH[pose]+(poseStale?' · 位置可能已过时':''),pose==='map'&&!poseStale?'ok':pose==='odometry'||poseStale&&pose==='map'?'warn':'error']:odom?[ODOM_BADGE,'warn']:['车辆位置未知','warn'];
  $('pose-badge').hidden=!badge;if(badge){$('pose-badge').textContent=badge[0];$('pose-badge').dataset.level=badge[1];}
  const robotNote=s&&!s.robot?(odom?'车上暂未提供里程档的小车位置，地图只显示物品与周围障碍。':'车辆位置未知：车上没有提供小车位置。'):s?.robot&&poseStale?'位置可能已过时（超过 1 秒没有新位姿）。':'';
  $('robot-note').hidden=!robotNote;$('robot-note').textContent=robotNote;
  $('metric-fresh').textContent=s?(!connected()?'已断线':!isFresh?'数据过期':`${Math.max(0,Math.floor((performance.now()-receivedAt)/1000))} 秒前`):'等待数据';
  $('metric-robot').textContent=s?.robot?`${source==='demo'?'演示 · ':''}${!connected()||!isFresh?'状态未知':s.robot.status}`:s?'车辆位置未知':'未知';
  $('metric-position').textContent=s?.robot?(connected()&&isFresh?`${s.robot.position[0].toFixed(2)}, ${s.robot.position[1].toFixed(2)} m（${s.map.frame}）`:'历史位置 · 非实时'):'车辆位置未知';
  $('metric-objects').textContent=s?`${s.objects.length} 件物品`:'等待数据';
  const capList=caps?[caps.findText&&'按名称找',caps.select&&'选择候选',caps.cancel&&'停止',caps.pause&&'暂停',caps.resume&&'继续'].filter(Boolean).join('、')||'无':'未知';
  $('device-details').innerHTML=[['数据来源',source==='demo'?'本地演示数据':'实车桥接服务'],['连接状态',label],['服务地址',source==='demo'?'无需网络连接':adapter?.url||'未设置'],['数据类别',s?sourceZh(src,sourceDetailOf(s)):'未知'],['控制权',caps?CONTROL_ZH[caps.control]:'未知'],['可用操作',capList],['地图',s?`${s.map.name||'未命名'}${s.map.id?` · ${s.map.id.slice(0,12)}`:''}`:'未知'],['地图坐标系',s?`${s.map.frame} · ${s.map.units}${s.map.placeholder?' · 替身地图':''}`:'未知'],['定位状态',s?.robot?(connected()&&isFresh?POSE_ZH[s.robot.pose]:'未知 · 数据失效'):s?'车辆位置未知':'未知'],['位姿年龄',Number.isFinite(s?.robot?.poseAgeMs)?`${Math.round(s.robot.poseAgeMs)} ms`:'未知'],['记忆',s?.memory?`第 ${s.memory.revision??'—'} 版 · 作用域 ${(s.memory.scope||'未知').slice(0,8)}${Number.isFinite(s.memory.ageMs)?` · ${Math.round(s.memory.ageMs)} ms 前`:''}`:'未收到'],['最后收到数据',s?`${((performance.now()-receivedAt)/1000).toFixed(1)} 秒前（本机计时）`:'未收到'],['当前任务',s?.task?`${stateLabel(taskDisplayState(s.task))}${!connected()||!isFresh?'（历史状态）':''}`:'无'],['电量 / 相机状态','当前协议未提供']].map(([k,v])=>`<dt>${k}</dt><dd>${esc(v)}</dd>`).join('');
  const blocker=startBlocker({snapshot:s,connected:connected(),fresh:isFresh,caps});
  const canFind=!blocker&&!busy&&!pendingCmd;
  $('find-button').disabled=!s;
  $('search-results').querySelectorAll('[data-find]').forEach(b=>{b.disabled=!canFind;});
  $('start-task').hidden=!!caps&&!caps.findText;
  $('start-task').disabled=!canFind||!s?.objects.some(o=>o.id===selectedId);
  const odomHint=!blocker&&odom?ODOM_FIND_HINT:'';
  const feedback=busy?'正在发送请求，请稍候。':pendingCmd?'请求已转发，等待小车回应。':blocker||odomHint;
  if($('query-hint').textContent!==feedback)$('query-hint').textContent=feedback;
  if($('start-hint').textContent!==feedback)$('start-hint').textContent=feedback;
  document.querySelector('.mission-card').setAttribute('aria-busy',String(busy||!!pendingCmd));
  $('find-button').setAttribute('aria-describedby','query-hint');
  $('start-task').setAttribute('aria-describedby','start-hint');
  const observer=!!caps&&caps.control!=='controller';
  $('control-note').hidden=!(source==='live'&&connected()&&observer);
  $('control-note').textContent=caps?.control==='observer'?'另一台电脑持有控制权：本界面只读，找物、选择、停止都不可用。':'本界面没有控制权：找物、选择、停止都不可用。';
  renderTaskControls(isFresh);
  if(source==='live')$('map-stage').style.opacity=s&&(!connected()||!isFresh)?'.55':'1';else $('map-stage').style.opacity='1';
}
// 暂停/继续按能力表决定是否出现在 DOM（车栈不支持则不放按钮，§2.1）。
function renderTaskControls(isFresh){
  const t=snapshot?.task,active=activeTask(t),controls=document.querySelector('.task-controls');
  const canControl=!!caps&&caps.control==='controller'&&connected();
  const wantPause=active&&!!caps&&(t.state==='paused'?caps.resume:caps.pause&&t.state!=='ambiguous');
  let pause=$('pause-task');
  if(wantPause&&!pause){pause=document.createElement('button');pause.className='quiet-button';pause.id='pause-task';pause.onclick=()=>sendCommand(snapshot?.task?.state==='paused'?'resume':'pause',{taskId:snapshot?.task?.id??null});controls.prepend(pause);}
  if(!wantPause&&pause){pause.remove();pause=null;}
  if(pause){pause.textContent=t.state==='paused'?'继续任务':'暂停';pause.disabled=!canControl||!isFresh||busy||!!pendingCmd||snapshot?.source==='inconsistent'||snapshot?.source==='replay';}
  const stop=$('stop-task');stop.hidden=!!caps&&!caps.cancel;
  // 停止在数据过期时仍可发（取消永远是安全方向）；只要求连接在、有控制权。
  stop.disabled=!canControl||busy||!active;
  controls.hidden=!active;
  $('task-candidates').querySelectorAll('[data-candidate]').forEach(b=>{b.disabled=!canControl||!isFresh||busy||!!pendingCmd||!caps?.select||snapshot?.source==='inconsistent'||snapshot?.source==='replay';});
}
function renderObjects(){
  const objects=snapshot?.objects||[];const q=$('memory-filter').value.trim().toLowerCase();let list=objects.filter(o=>`${displayName(o)} ${o.className} ${classZh(o.className)} ${o.location||''}`.toLowerCase().includes(q));if(currentPage==='overview')list=list.slice(0,3);
  const key=JSON.stringify([source,snapshot?.map.frame,list.map(o=>({...o,lastSeen:relativeText(secondsSince(snapshot,o.lastSeen))}))]);$('object-count').textContent=objects.length;$('memory-count').textContent=objects.length;$('memory-empty').hidden=!!list.length;
  renderSuggestions();
  if(key===lastCards){refreshSeen();return;}lastCards=key;
  const odom=odomTier(snapshot);
  $('objects-grid').innerHTML=list.map(o=>{const name=displayName(o),st=o.state&&OBJECT_STATE_ZH[o.state];return `<button class="object-card" data-object="${esc(o.id)}" data-state="${esc(o.state||'')}" aria-label="查看${esc(name)} ${esc(o.id)}">${art(o)}<span class="object-copy"><span class="object-name">${esc(name)}<span class="object-id">${esc(o.id)}</span></span><span class="object-location">${esc(o.location||(odom?'位置为里程坐标，可能有偏差':`${o.position[0].toFixed(1)}, ${o.position[1].toFixed(1)} m`))}</span><small><span data-seen-id="${esc(o.id)}">${seenHTML(o.lastSeen)}</span> · ${source==='demo'?'演示记忆':esc(st||'历史观测')}</small></span><span class="card-arrow">${icon('arrow')}</span></button>`;}).join('');
  $('objects-grid').querySelectorAll('[data-object]').forEach(b=>b.onclick=()=>openObject(b.dataset.object));
}
function refreshSeen(){const by=new Map((snapshot?.objects||[]).map(o=>[o.id,o]));document.querySelectorAll('[data-seen-id]').forEach(el=>{const o=by.get(el.dataset.seenId);if(o)el.innerHTML=seenHTML(o.lastSeen);});}
let lastSuggest='';
function renderSuggestions(){const cls=topClasses(snapshot?.objects||[]);const key=cls.join('|');if(key===lastSuggest)return;lastSuggest=key;$('suggestions').innerHTML=cls.map(c=>`<button type="button" data-query="${esc(classZh(c))}">${esc(classZh(c))}</button>`).join('');$('suggestions').querySelectorAll('[data-query]').forEach(b=>b.onclick=()=>{$('search-input').value=b.dataset.query;$('search-input').focus();});}
$('memory-filter').oninput=renderObjects;
const STEPS=[['searching','检索记忆 / 搜索'],['navigating','前往目标附近'],['confirming','现场观测确认']];
function stepIndex(state){return state==='ambiguous'?0:state==='planning'?0:STEPS.findIndex(([k])=>k===state);}
function renderTask(){
  const t=snapshot?.task,shown=!!t&&t.state!=='idle';$('task-empty').hidden=shown;$('task-active').hidden=!shown;const state=t?taskDisplayState(t):'idle';
  const stateText=stateLabel(state);
  const badge=$('task-state');
  if(badge.textContent!==stateText)badge.textContent=stateText;
  if(badge.dataset.state!==state){badge.dataset.state=state;const panel=document.querySelector('.mission-card');panel.classList.remove('state-changed');requestAnimationFrame(()=>panel.classList.add('state-changed'));}
  if(!shown){$('task-candidates').replaceChildren();$('task-candidates').dataset.key='';$('task-candidates').hidden=true;document.querySelector('.mission-card').classList.remove('is-ambiguous');return;}
  const target=t.targetId?(snapshot.objects.find(o=>o.id===t.targetId)||t.candidates.find(c=>c.id===t.targetId)):null;
  $('task-target').textContent=`${target?displayName(target):t.targetId?t.targetId:lastQuery?`查找「${lastQuery}」`:'寻物任务'} · ${source==='demo'?'演示任务':'实车任务'}`;
  // 暂停/失败/取消时保留已到达的那一步。
  let idx=t.state==='succeeded'?STEPS.length:stepIndex(t.state);const halted=['paused','failed','cancelled'].includes(t.state);
  if(halted)idx=reachedStep.get(t.id)??-1;
  $('task-steps').innerHTML=STEPS.map(([key,label],i)=>`<li class="${i<idx?'done':i===idx?(halted?'halted':'current'):''}">${label}${i===idx?(halted?` · ${stateText}`:t.state==='ambiguous'?' · 请选择目标':' · 进行中'):''}</li>`).join('');
  const manual=manualTargetPosition(t);
  $('task-message').textContent=(t.message||reasonText('',t.reasonCode))+(manual&&!/记忆位置/.test(t.message||'')?`（记忆位置 ${manual.map(n=>n.toFixed(2)).join(', ')} m，${snapshot.map.frame} 系）`:manual?`（${snapshot.map.frame} 系）`:'');
  const cands=t.state==='ambiguous'?t.candidates:[];
  const ckey=JSON.stringify([t.id,cands,snapshot.robot?.position?.map(n=>n.toFixed(1))]);
  if($('task-candidates').dataset.key!==ckey){$('task-candidates').dataset.key=ckey;
    const r=snapshot.robot?.position;
    $('task-candidates').innerHTML=cands.length?`<p class="candidates-title">点一个让小车去找（共 ${cands.length} 个）：</p><div class="candidate-list">${cands.map(c=>{const d=r?Math.hypot(c.position[0]-r[0],c.position[1]-r[1]):null;return `<button class="candidate-button" data-candidate="${esc(c.id)}"><strong>${esc(displayName(c))}</strong><span>${esc([d===null?'距离未知':`距小车 ${d.toFixed(1)} m`,c.state&&OBJECT_STATE_ZH[c.state]].filter(Boolean).join(' · '))}</span><small>${esc(c.id)}</small></button>`;}).join('')}</div>`:'';
    $('task-candidates').querySelectorAll('[data-candidate]').forEach(b=>b.onclick=()=>sendCommand('select',{taskId:snapshot?.task?.id??null,candidateId:b.dataset.candidate,scope:snapshot?.memory?.scope??null}));
  }
  $('task-candidates').hidden=!cands.length;
  document.querySelector('.mission-card').classList.toggle('is-ambiguous',!!cands.length);
}
function render(){renderConnection();renderObjects();renderTask();}
function openObject(id){
  const o=snapshot?.objects.find(o=>o.id===id);if(!o)return;selectedId=id;selectedScope=snapshot.memory?.scope??null;view?.focus(id);
  const name=displayName(o),odom=odomTier(snapshot),st=o.state&&OBJECT_STATE_ZH[o.state];
  const rows=[['物品类别',`${classZh(o.className)}${o.className&&classZh(o.className)!==o.className?`（${o.className}）`:''}`]];
  if(o.location)rows.push(['上次所在位置',o.location]);
  rows.push(['空间坐标',`${o.position.map(n=>n.toFixed(2)).join(', ')} m（${snapshot.map.frame} 系）${odom?' · 里程坐标，可能有偏差':''}`]);
  rows.push(['上次见到',null]);
  if(st)rows.push(['记忆状态',st]);
  if(Number.isFinite(o.confidence))rows.push(['识别分数',`${o.confidence.toFixed(2)}${source==='demo'?'（模拟值）':''} · 不等于真实概率`]);
  rows.push(['现场确认',snapshot.task?.targetId===id&&snapshot.task.state==='succeeded'?(source==='demo'?'演示确认完成':'任务回传已确认'):'尚未进行本次现场确认']);
  $('detail-body').innerHTML=`<div class="detail-hero">${art(o)}<div><h2>${esc(name)}</h2><p>${esc(o.id)} · ${source==='demo'?'演示物体记忆':'物体记忆'}</p></div></div><dl class="detail-info">${rows.map(([k,v])=>`<dt>${k}</dt><dd>${v===null?`<span data-seen-id="${esc(o.id)}">${seenHTML(o.lastSeen)}</span>`:esc(v)}</dd>`).join('')}</dl>`;
  $('start-task').innerHTML=`寻找这类物品（${esc(classZh(o.className))}）${icon('arrow')}`;
  $('start-note').textContent=`小车按类别「${classZh(o.className)}」查找：记忆里有多个时会请你选择具体哪一个，不会直接按这一件出发。`;
  renderConnection();if(!$('detail-dialog').open)$('detail-dialog').showModal();
}
$('focus-object').onclick=()=>{closeDialog($('detail-dialog')).then(()=>setPage('overview')).then(()=>requestAnimationFrame(()=>{view?.focus(selectedId);document.querySelector('.map-card').scrollIntoView({block:'start',behavior:reducedMotion()?'instant':'smooth'});}));};
// 搜索框：先列出记忆里的匹配项（只是查看），再由用户点「让小车找」按文字发起（runner 原生语义）。
$('search-form').onsubmit=e=>{e.preventDefault();const q=$('search-input').value.trim().replace(/^(帮我)?(寻找|查找|找一下|找)\s*/,'').trim();if(!q){$('search-input').focus();toast('先输入想找的物品名称');return;}
  const matches=matchObjects(snapshot?.objects||[],q).slice(0,50);
  const findBtn=!caps||caps.findText?`<button class="primary-button find-text-button" type="button" data-find="${esc(q)}">让小车找「${esc(q)}」${icon('arrow')}</button>`:'';
  $('search-results').innerHTML=findBtn+(matches.length?`<p class="results-title">记忆里有 ${matches.length} 件相关物品（点开可查看）：</p>`+matches.map(o=>`<button class="result-button" data-result="${esc(o.id)}">${esc(displayName(o))} · ${esc(o.location||`${o.position[0].toFixed(1)}, ${o.position[1].toFixed(1)} m`)} · ${esc(o.id)}</button>`).join(''):`<p class="results-title">记忆中没有找到“${esc(q)}”。仍可让小车按名称搜索。</p>`);
  $('search-results').querySelectorAll('[data-result]').forEach(b=>b.onclick=()=>openObject(b.dataset.result));
  $('search-results').querySelectorAll('[data-find]').forEach(b=>b.onclick=()=>findText(b.dataset.find));
  renderConnection();};
function findText(text){lastQuery=text;sendCommand('find',{text:String(text).slice(0,64)});}
function onResult(r){
  if(!pendingCmd||pendingCmd.id!==r.id){if(r.status==='unknown')toast(r.reason&&/未知/.test(r.reason)?r.reason:`结果未知：${reasonText(r.reason,r.reasonCode)||'请核对小车状态'}；未自动重发。`);return;}
  pendingCmd=null;
  const note=r.note?`（${r.note}）`:'';
  const name={find:'查找',select:'选择',cancel:'停止',pause:'暂停',resume:'继续'}[r.action]||'请求';
  if(r.status==='accepted'||r.status==='done')toast((source==='demo'?'演示任务已更新':`小车已${r.status==='done'?'执行':'受理'}${name}请求`)+note);
  else if(r.status==='rejected')toast(`小车拒绝了${name}：${reasonText(r.reason,r.reasonCode)||'未给原因'}${note}`);
  else toast((r.reason&&/未知/.test(r.reason)?r.reason:`${name}结果未知：${reasonText(r.reason,r.reasonCode)||'请核对小车状态'}；未自动重发。`)+note);
  render();
}
function sendCommand(action,payload={}){
  if(busy||!adapter)return;
  if(action==='find'){const b=startBlocker({snapshot,connected:connected(),fresh:fresh(),caps});if(b){toast(b);return;}}
  if(action==='select'&&(snapshot?.source==='inconsistent'||snapshot?.source==='replay'||!caps?.select)){toast('当前不能选择候选物品。');return;}
  if(action!=='cancel'&&(!fresh()||!connected())){toast('数据不可用，请先恢复连接');return;}
  if(action==='cancel'&&!connected()){toast('连接已断开，停止请求未发送；请直接在车上停车。');return;}
  if(caps&&caps.control!=='controller'){toast('另一台电脑持有控制权，本界面只读。');return;}
  const gen=generation;busy=true;renderConnection();
  const {id,acked}=adapter.command(action,payload);
  // 待回执已在下面同步登记；这里不再补登记（否则结果先到、then 后到时会把已收尾的命令重新挂成「等待回应」）。
  acked.then(()=>{if(gen!==generation)return;busy=false;render();},
    e=>{if(gen!==generation)return;busy=false;if(pendingCmd?.id===id)pendingCmd=null;toast(e.message);render();});
  // 演示适配器同步给出结果，结果可能先于 then 回调到达：先登记待回执。
  if(id)pendingCmd={id,action};
}
$('start-task').onclick=()=>{
  const o=snapshot?.objects.find(x=>x.id===selectedId);
  if(!o||(snapshot.memory?.scope??null)!==selectedScope){toast('记忆已重置，请重新选择');closeDialog($('detail-dialog'));return;}
  closeDialog($('detail-dialog')).then(()=>setPage('overview')).then(()=>view?.reset());
  lastQuery=classZh(o.className);sendCommand('find',{text:o.className});
};
$('stop-task').onclick=()=>sendCommand('cancel',{taskId:snapshot?.task?.id??null});
function rememberTask(){const t=snapshot?.task;if(!t||!t.id||!TERMINAL.has(t.state)||history.some(r=>r.id===t.id&&r.source===source))return;const target=t.targetId?(snapshot.objects.find(o=>o.id===t.targetId)||t.candidates.find(c=>c.id===t.targetId)):null;
  history.unshift({v:2,id:t.id,state:t.state,outcome:taskDisplayState(t)!==t.state?taskDisplayState(t):undefined,message:t.message||reasonText('',t.reasonCode),source,target:target?displayName(target):lastQuery?`查找「${lastQuery}」`:'寻物任务',scope:(snapshot.memory?.scope||'').slice(0,8),map:snapshot.map.name||(source==='demo'?'演示空间':'实车地图'),recordedAt:new Date().toISOString()});history=history.slice(0,50);try{localStorage.setItem('geomem-history',JSON.stringify(history));}catch{toast('浏览器未允许保存记录，本次记录仅在当前页面保留');}renderHistory();}
// 任务记录是纯文本（旧作用域的物品编号不再可点，§2.7）；时间为本机记录时刻。
function renderHistory(){$('history-list').innerHTML=history.length?history.map(r=>`<div class="history-row"><span>${esc(r.target)}<small>${esc(r.map)} · ${r.source==='demo'?'演示':'实车'}${r.scope?` · 记忆 ${esc(r.scope)}`:''}</small></span><span>${esc(new Date(r.recordedAt).toLocaleString('zh-CN',{hour12:false}))}<small>本机记录时间 · ${esc(r.message)}</small></span><span class="status-tag" data-state="${esc(taskDisplayState(r))}">${esc(stateLabel(taskDisplayState(r)))}</span></div>`).join(''):'<div class="empty-state">还没有结束的任务。完成一次寻找后，记录会出现在这里。</div>';}
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
window.addEventListener('hashchange',()=>{const key=location.hash.slice(1);if(['overview','memory','history','device'].includes(key)&&key!==currentPage)setPage(key);});
setInterval(()=>{renderConnection();renderObjects();},1000);
setPage(['overview','memory','history','device'].includes(location.hash.slice(1))?location.hash.slice(1):'overview');
// ?source=live 直接连同源桥（桥托管页面时用）；?ws=… 可指定别的桥地址。
{const q=new URLSearchParams(location.search);if(q.get('source')==='live'){try{const url=endpointURL(q.get('ws')||defaultEndpoint(),location.protocol==='https:');$('endpoint').value=url;connect('live',url);}catch(e){toast(e.message);connect('demo');}}else connect('demo');}
// 不再注册 Service Worker（开发/实车下旧缓存会在服务停掉后继续显示旧演示页）；顺手注销以前装过的。
if('serviceWorker' in navigator)navigator.serviceWorker.getRegistrations().then(rs=>rs.forEach(r=>r.unregister())).catch(()=>{});
if(globalThis.caches)caches.keys().then(ks=>ks.filter(k=>k.startsWith('geomem-m8-')).forEach(k=>caches.delete(k))).catch(()=>{});
