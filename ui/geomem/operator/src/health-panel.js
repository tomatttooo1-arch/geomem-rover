// 10-29 car stack health check (snapshot.health from the hub, host/hub/health.py): one bar under the connection
// notice. Red = will make mapping / finding fail, with what to do; yellow = reminders; green = 可以开始建图.
// Warn only: nothing here locks a control. The list opens by itself when the bar turns red; a click folds it.
const LEVEL_ZH={red:'需处理',yellow:'提醒',green:'正常',pending:'检查中'};
const LEVELS=new Set(Object.keys(LEVEL_ZH));
const escHtml=s=>String(s??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
let lastKey='',lastOverall='';

export function healthHtml(h){
  const checks=(Array.isArray(h.checks)?h.checks:[]).filter(c=>c&&LEVELS.has(c.level));
  const red=checks.filter(c=>c.level==='red').length,yellow=checks.filter(c=>c.level==='yellow').length;
  const count=checks.length?`<span class="health-count">${red?`${red} 项需处理`:''}${red&&yellow?' · ':''}${yellow?`${yellow} 项提醒`:''}${!red&&!yellow?`${checks.length} 项正常`:''}<b class="health-caret" aria-hidden="true"></b></span>`:'';
  const items=checks.map(c=>`<li class="health-item ${c.level}"><span class="health-dot" title="${LEVEL_ZH[c.level]}"></span><span class="health-title">${escHtml(c.title)}</span><span class="health-detail">${escHtml(c.detail)}</span>${c.hint?`<span class="health-hint">处理：${escHtml(c.hint)}</span>`:''}</li>`).join('');
  return `<summary><span class="notice-dot"></span><span class="health-summary">${escHtml(h.summary||'')}</span>${count}</summary>${items?`<ul class="health-list">${items}</ul>`:''}`;
}

// -> true when the panel shows a report (the caller then hides the older one-line #car-notice: the ground
// calibration warning is one of the checks); null snapshot = keep what is shown (a start-up fact, like the notice)
export function renderHealth(snap){
  const box=document.getElementById('car-health');if(!box)return false;
  if(!snap)return !box.hidden;
  const h=snap.health;
  if(!h||!LEVELS.has(h.overall)){box.hidden=true;lastKey='';lastOverall='';return false;}
  const key=JSON.stringify(h);
  if(key!==lastKey){
    lastKey=key;
    const wasOpen=box.open;
    box.innerHTML=healthHtml(h);
    box.className=`health-panel ${h.overall}`;
    box.open=h.overall==='red'&&lastOverall!=='red'?true:wasOpen&&h.overall!=='green'?wasOpen:false;
    lastOverall=h.overall;
  }
  box.hidden=false;
  return true;
}
