// 10-29 「FPGA 实时」: what the KV260 PL is doing right now, for the judges (display only, no control).
// Data: snapshot.fpga.live from the bridge (gmb_proto.compose_fpga_live <- hub STATE perf.fpga / clock / kv.permit
// <- the per-frame kv_link answers). Every number comes from the running stack; a value the bridge does not have is
// null and shows as —. No snapshot for STALE_MS (bridge or hub gone) -> every tile shows —, never the old value.
// Refreshed once a second (the newest snapshot), folds to one line; the tiles drop over the page (no layout shift).
export const STALE_MS=3000;
export const RATE_WINDOW_MS=3000;
const DASH='—';
const escHtml=s=>String(s??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const num=v=>typeof v==='number'&&Number.isFinite(v)?v:null;
const count=v=>Number.isInteger(v)&&v>=0?v:null;

export function fmtMs(v){v=num(v);if(v===null||v<0)return DASH;return v<1?v.toFixed(2):v<10?v.toFixed(1):v.toFixed(0);}
export function fmtCount(v){v=count(v);if(v===null)return DASH;return v>=10000?`${(v/10000).toFixed(1)} 万`:String(v);}

// Permit renewals per second from the PL's FRAMES_ACCEPTED counter (each accepted frame done edge renews the
// permit): samples of the last RATE_WINDOW_MS; a counter that goes back (KV restarted) starts over.
export class RenewalRate{
  constructor(windowMs=RATE_WINDOW_MS){this.windowMs=windowMs;this.samples=[];}
  feed(tMs,n){
    if(count(n)===null){this.samples=[];return;}
    const last=this.samples[this.samples.length-1];
    if(last&&(n<last.n||tMs<=last.t)){if(n<last.n)this.samples=[];else return;}
    this.samples.push({t:tMs,n});
    while(this.samples.length>2&&tMs-this.samples[1].t>=this.windowMs)this.samples.shift();
  }
  value(nowMs){
    const s=this.samples;if(s.length<2)return null;const a=s[0],b=s[s.length-1];
    if(nowMs-b.t>STALE_MS||b.t-a.t<500)return null;
    return (b.n-a.n)*1000/(b.t-a.t);
  }
  clear(){this.samples=[];}
}

// live (snapshot.fpga.live or null) + renewal rate -> {tiles, chips, level}
export function fpgaModel(live,rate=null){
  const f=live&&typeof live==='object'?live:{};
  const frames=num(f.fps)!==null||num(f.geoMs)!==null;
  const tiles=[];
  const geo=num(f.geoMs),px=count(f.pixels);
  tiles.push({key:'geo',label:'几何核',value:fmtMs(geo),unit:geo===null?'':'ms',
    sub:px!==null&&geo!==null?`每帧 ${px>=10000?(px/10000).toFixed(1)+' 万':px+' '}像素 → 体素`:DASH});
  const roi=num(f.roiMs),boxes=count(f.roiBoxes);
  tiles.push({key:'roi',label:'证据核',value:fmtMs(roi),unit:roi===null?'':'ms',
    sub:roi!==null&&boxes!==null?`核对 ${boxes} 个物体框`:frames&&f.roiFrames===0?'视野内无记忆物体':DASH});
  const roll=num(f.rollMs),cells=count(f.rollCells);
  tiles.push({key:'roll',label:'融合核',value:fmtMs(roll),unit:roll===null?'':'ms',
    sub:roll!==null&&cells!==null?`更新 ${fmtCount(cells)} 个地图格`:DASH});
  const fps=num(f.fps);
  tiles.push({key:'fps',label:'处理速度',value:fps===null?DASH:fps.toFixed(1),unit:fps===null?'':'帧/秒',
    sub:fps===null?DASH:'深度帧 → KV260 → 地图'});
  const rtt=num(f.rttMs),bound=num(f.syncBoundMs);
  tiles.push({key:'link',label:'两板往返',value:fmtMs(rtt),unit:rtt===null?'':'ms',
    sub:bound!==null?`对时误差 ≤ ${fmtMs(bound)} ms`:DASH});
  const r=num(rate),permit=f.permit;
  const pv=permit===true?'续约中':permit===false?(f.estop===true?'急停':'已撤销'):DASH;
  tiles.push({key:'permit',label:'行驶许可',value:pv,unit:'',state:permit===true?'on':permit===false?'off':'',
    sub:permit===true&&r!==null?`每秒续约 ${r.toFixed(1)} 次`:permit===false?'车停住，等新帧':DASH});
  const chips=[['几何',geo,'ms'],['证据',roi,'ms'],['融合',roll,'ms']].map(([k,v,u])=>`${k} ${fmtMs(v)}${v===null?'':' '+u}`);
  chips.push(fps===null?`${DASH} 帧/秒`:`${fps.toFixed(1)} 帧/秒`);
  chips.push(`许可 ${pv}`);
  const level=!frames&&permit==null?'idle':permit===false?'warn':frames&&permit===true?'ok':'idle';
  return {tiles,chips,level,pulse:permit===true&&r!==null&&r>0};
}

export function fpgaHtml(m){
  const chips=m.chips.map(c=>`<span class="fpga-chip">${escHtml(c)}</span>`).join('');
  const tiles=m.tiles.map(t=>`<li class="fpga-tile${t.state?' '+t.state:''}" data-key="${t.key}"><span class="fpga-label">${t.key==='permit'?`<b class="fpga-beat${m.pulse?' beating':''}" aria-hidden="true"></b>`:''}${escHtml(t.label)}</span><strong class="fpga-value">${escHtml(t.value)}${t.unit?`<small>${escHtml(t.unit)}</small>`:''}</strong><span class="fpga-sub">${escHtml(t.sub)}</span></li>`).join('');
  return `<summary><span class="fpga-tag">FPGA 实时</span><span class="fpga-chips">${chips}</span><b class="health-caret" aria-hidden="true"></b></summary>`+
    `<div class="fpga-drop"><ul class="fpga-tiles">${tiles}</ul><p class="fpga-note">KV260 可编程逻辑上的三个核：时间为启动到完成中断，近 10 帧中位数；每秒刷新</p></div>`;
}

// ---- page wiring: noteFpga(snapshot) on every snapshot; a 1 s timer renders the newest one
let latest=null,latestAt=0,seen=false,timer=null,lastHtml='';
const rate=new RenewalRate();
const OPEN_KEY='geomem-fpga-open';

export function noteFpga(snap,nowMs=Date.now()){
  if(!snap||typeof snap!=='object'||!snap.fpga)return;
  latest=snap.fpga;latestAt=nowMs;
  const live=latest.live;
  if(live&&typeof live==='object'&&count(live.renewals)!==null)rate.feed(nowMs,live.renewals);else rate.clear();
  if(!seen){seen=true;renderFpga(nowMs);}
  if(!timer&&typeof setInterval==='function')timer=setInterval(()=>renderFpga(Date.now()),1000);
}

export function renderFpga(nowMs=Date.now()){
  const box=typeof document!=='undefined'?document.getElementById('fpga-panel'):null;if(!box)return;
  if(!seen){box.hidden=true;return;}
  const fresh=latest&&nowMs-latestAt<=STALE_MS;
  const m=fpgaModel(fresh?latest.live:null,fresh?rate.value(nowMs):null);
  const html=fpgaHtml(m);
  if(html!==lastHtml){
    const open=box.open;lastHtml=html;box.innerHTML=html;box.open=open;
    box.className=`fpga-panel ${m.level}`;
  }
  if(box.hidden){
    box.hidden=false;
    try{box.open=localStorage.getItem(OPEN_KEY)==='1';}catch{}
    box.addEventListener('toggle',()=>{try{localStorage.setItem(OPEN_KEY,box.open?'1':'0');}catch{}});
  }
}
