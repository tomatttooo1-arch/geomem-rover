// 演示数据源：完全本地模拟，按协议第二版的消息形状发 hello / cloud / snapshot / result，与实车走同一条显示路径。
import { matchObjects, displayName, newId, normalizeCapabilities } from './model.js';
const clone = x => structuredClone(x);
const pos = (x,y,z=0) => [x,y,z];
const DEMO_SCOPE = 'demo-scope';
// 演示自己的时间轴（秒），只用本机单调钟推进；与墙钟无关。
const demoNow = () => 1000 + performance.now() / 1000;

export function demoCloud() {
  // Deterministic illustrative samples. These are not sensor measurements.
  const xyz = [], rgb = [];
  let seed = 23;
  const rand = () => ((seed = (seed * 16807) % 2147483647) - 1) / 2147483646;
  const push = (x, y, z, c) => { xyz.push(x, y, z); rgb.push(...c.map(v => Math.round(v * 255))); };
  for (let i=0;i<7000;i++) {
    const x=rand()*8-4, y=rand()*6-3;
    let z=0, color=[.18,.48,.68];
    if (i%5===0) {z=rand()*1.4; push(x,3,z,[.26,.54,.77]); continue;}
    if (i%7===0) {z=rand()*1.25; push(-4,y,z,[.32,.58,.79]); continue;}
    if(x < -2.5 && y > -.7 && y < 1.8) {z=.65; color=[.28,.65,.83];}
    if(x>1.6 && x<3.5 && y>1.7) {z=.85; color=[.48,.67,.87];}
    if(x>-1.4 && x<.4 && y>-.9 && y<.3){z=.52;color=[.36,.7,.86];}
    push(x,y,z+rand()*.025,color);
  }
  const n = xyz.length / 3;
  return {header:{type:'cloud',version:2,mapId:'demo-room',rev:'demo:1',frame:'map',count:n,source:'export',encoding:'f32xyz+u8rgb'}, positions:new Float32Array(xyz), colors:new Uint8Array(rgb)};
}
export function demoSnapshot() {
  const now = demoNow();
  const objects = [
    {id:'bag-01',name:'书包',className:'backpack',color:null,aliases:['背包','bag'],location:'书桌旁 · 东侧',position:pos(2.4,1.65,.55),confidence:.92,lastSeen:now-120,state:'historical_memory'},
    {id:'mouse-01',name:'鼠标',className:'mouse',color:null,aliases:['电脑鼠标'],location:'书桌上 · 东侧',position:pos(2.5,2.55,.95),confidence:.87,lastSeen:now-180,state:'historical_memory'},
    {id:'cup-01',name:'杯子',className:'cup',color:'blue',aliases:['水杯','杯'],location:'茶几上 · 中央',position:pos(-.7,-.3,.63),confidence:.94,lastSeen:now-60,state:'current_confirmed'},
    {id:'cup-02',name:'杯子',className:'cup',color:'white',aliases:['水杯','杯'],location:'书桌上 · 西侧',position:pos(1.9,2.4,.95),confidence:.71,lastSeen:now-600,state:'historical_memory'}
  ];
  return {type:'snapshot',version:2,seq:0,serverTime:now,source:'live',link:{state:'up',hubAgeMs:0},
    map:{id:'demo-room',name:'一间熟悉的房间',frame:'map',units:'m',cloudRev:'demo:1',placeholder:false},
    robot:{position:pos(1,-1.8),yaw:Math.PI/2,speed:0,pose:'map',poseAgeMs:0,status:'待命'},
    memory:{scope:DEMO_SCOPE,revision:1,ageMs:0},objects,path:[],task:null};
}
const DEMO_CAPS = normalizeCapabilities({findText:true,findById:false,select:true,cancel:true,pause:true,resume:true,estop:false,control:'controller'});
export class DemoAdapter extends EventTarget {
  constructor() { super(); this.snapshot=demoSnapshot(); this.ticks=0; this.timer=null; this.taskSeq=0; this.caps=DEMO_CAPS; }
  emit(type, detail) { this.dispatchEvent(new CustomEvent(type,{detail})); }
  connect() {
    this.emit('hello',{hello:{type:'hello',version:2,bridge:{id:'demo',hubBootId:null,link:'up'},source:'live',capabilities:DEMO_CAPS},caps:DEMO_CAPS});
    this.emit('connection',{state:'connected'});
    this.emit('cloud',demoCloud());
    this.publish(); this.timer=setInterval(()=>this.tick(),1000);
  }
  publish() { this.snapshot.serverTime=demoNow(); this.snapshot.seq++; this.emit('snapshot',clone(this.snapshot)); }
  setTask(fields) { const s=this.snapshot; s.task={id:s.task?.id??null,state:'idle',reasonCode:null,message:'',targetId:null,ageMs:0,candidates:[],...s.task,...fields}; }
  startTarget(target) { const s=this.snapshot; this.target=target;this.origin=[...s.robot.position];this.ticks=0;s.path=[];this.setTask({state:'searching',reasonCode:'search',targetId:target.id,candidates:[],message:`正在查询演示空间记忆：${displayName(target)}`}); }
  command(action, payload={}) {
    const id=newId();
    const result=(status,reason,reasonCode=null)=>setTimeout(()=>{this.publish();this.emit('result',{id,action,status,reason,reasonCode});},120);
    const s=this.snapshot,t=s.task,active=t&&!['succeeded','failed','cancelled','idle'].includes(t.state);
    if(action==='find') {
      if(active) return {id,acked:Promise.reject(Error('请先结束当前任务'))};
      const text=String(payload.text||'').trim();
      const matches=matchObjects(s.objects,text);
      if(!text) return {id,acked:Promise.reject(Error('请输入要找的物品'))};
      s.task={id:`demo:${++this.taskSeq}`,state:'idle',reasonCode:null,message:'',targetId:null,ageMs:0,candidates:[]};
      if(!matches.length) { this.setTask({state:'failed',reasonCode:'search_exhausted',message:`演示记忆里没有「${text}」，演示模式不做实地搜索`}); result('rejected','演示记忆里没有这类物品','query_rejected'); }
      else if(matches.length===1) { this.startTarget(matches[0]); result('accepted',''); }
      else { this.setTask({state:'ambiguous',reasonCode:'candidate_selection_required',message:`记忆里有 ${matches.length} 个符合「${text}」的物品，请选择要找哪一个`,candidates:matches.map(o=>({id:o.id,name:displayName(o),className:o.className,color:o.color,position:[...o.position],state:o.state}))}); result('accepted',''); }
    } else if(action==='select') {
      if(t?.state!=='ambiguous'||payload.taskId!==t.id) return {id,acked:Promise.reject(Error('任务已变化，请按当前任务重新操作'))};
      const target=s.objects.find(o=>o.id===payload.candidateId);
      if(!target||!t.candidates.some(c=>c.id===payload.candidateId)) return {id,acked:Promise.reject(Error('所选物品已不在候选中，请重新选择'))};
      this.startTarget(target); result('accepted','');
    } else if(action==='cancel' && active) { this.setTask({state:'cancelled',reasonCode:'user_cancelled',message:'演示任务已停止',candidates:[]});s.robot.speed=0;s.robot.status='已停止';s.path=[];result('done',''); }
    else if(action==='pause' && active && t.state!=='paused' && t.state!=='ambiguous') { this.beforePause=t.state;this.setTask({state:'paused',message:'演示任务已暂停'});s.robot.speed=0;s.robot.status='已暂停';result('done',''); }
    else if(action==='resume' && t?.state==='paused') { this.setTask({state:this.beforePause,message:'演示任务已继续'});s.robot.status=this.beforePause==='navigating'?'行驶中':'待命';result('done',''); }
    else return {id,acked:Promise.reject(Error('当前状态不支持该操作'))};
    return {id,acked:Promise.resolve({accepted:true,status:'forwarded'})};
  }
  tick() {
    const s=this.snapshot,t=s.task;
    if(t && ['searching','navigating','confirming'].includes(t.state)) {
      this.ticks++;
      if(this.ticks<3) {this.setTask({state:'searching',message:'已找到物体记忆，准备选择观察点'});}
      else if(this.ticks<12) {
        if(!s.path.length)s.path=[this.origin,[this.origin[0],.7,0],[this.target.position[0]-.75,.7,0],[this.target.position[0]-.75,this.target.position[1]-.6,0]];
        this.setTask({state:'navigating',reasonCode:'reobserve',message:'沿演示路径前往记忆位置'});s.robot.status='行驶中';s.robot.speed=.25;
        const p=(this.ticks-3)/8*3,i=Math.min(2,Math.floor(p)),f=Math.min(1,p-i),a=s.path[i],b=s.path[i+1];
        s.robot.position=a.map((n,k)=>n+(b[k]-n)*f);s.robot.yaw=Math.atan2(b[1]-a[1],b[0]-a[0]);
      } else if(this.ticks<15) {this.setTask({state:'confirming',reasonCode:'fresh_visual_evidence_required',message:'正在模拟现场 RGB-D 复核，不是真实识别结果'});s.robot.speed=0;s.robot.status='确认目标';}
      else {this.setTask({state:'succeeded',reasonCode:'fresh_matching_observation',message:'演示完成：已模拟确认目标，实车需由小车回传确认结果'});s.robot.speed=0;s.robot.status='待命';this.target.lastSeen=demoNow();this.target.state='current_confirmed';}
    }
    this.publish();
  }
  close() {clearInterval(this.timer);this.emit('connection',{state:'disconnected'});}
}
