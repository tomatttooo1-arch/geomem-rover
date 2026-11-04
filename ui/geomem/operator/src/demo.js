const clone = x => structuredClone(x);
const pos = (x,y,z=0) => [x,y,z];
export function demoSnapshot() {
  const now = Date.now();
  const objects = [
    {id:'bag-01',name:'书包',className:'backpack',aliases:['背包','bag'],location:'书桌旁 · 东侧',position:pos(2.4,1.65,.55),confidence:.92,lastSeen:new Date(now-120000).toISOString()},
    {id:'mouse-01',name:'鼠标',className:'mouse',aliases:['电脑鼠标'],location:'书桌上 · 东侧',position:pos(2.5,2.55,.95),confidence:.87,lastSeen:new Date(now-180000).toISOString()},
    {id:'cup-01',name:'水杯',className:'cup',aliases:['杯子','杯','mug'],location:'茶几上 · 中央',position:pos(-.7,-.3,.63),confidence:.94,lastSeen:new Date(now-60000).toISOString()}
  ];
  // Deterministic illustrative samples. These are not sensor measurements.
  const points = [];
  let seed = 23;
  const rand = () => ((seed = (seed * 16807) % 2147483647) - 1) / 2147483646;
  for (let i=0;i<7000;i++) {
    const x=rand()*8-4, y=rand()*6-3;
    let z=0, color=[.18,.48,.68];
    if (i%5===0) {z=rand()*1.4; points.push([x,3,z,.26,.54,.77]); continue;}
    if (i%7===0) {z=rand()*1.25; points.push([-4,y,z,.32,.58,.79]); continue;}
    if(x < -2.5 && y > -.7 && y < 1.8) {z=.65; color=[.28,.65,.83];}
    if(x>1.6 && x<3.5 && y>1.7) {z=.85; color=[.48,.67,.87];}
    if(x>-1.4 && x<.4 && y>-.9 && y<.3){z=.52;color=[.36,.7,.86];}
    points.push([x,y,z+rand()*.025,...color]);
  }
  return {type:'snapshot',version:1,seq:0,timestamp:new Date().toISOString(),map:{id:'demo-room',name:'一间熟悉的房间',frame:'map',units:'m'},robot:{position:pos(1,-1.8),yaw:Math.PI/2,speed:0,localized:true,status:'待命'},objects,path:[],points,task:null};
}
export class DemoAdapter extends EventTarget {
  constructor() { super(); this.snapshot=demoSnapshot(); this.ticks=0; this.timer=null; }
  emit(type, detail) { this.dispatchEvent(new CustomEvent(type,{detail})); }
  connect() { this.emit('connection',{state:'connected'}); this.publish(); this.timer=setInterval(()=>this.tick(),1000); }
  publish() { this.snapshot.timestamp=new Date().toISOString(); this.snapshot.seq++; this.emit('snapshot',clone(this.snapshot)); }
  async command(action, payload={}) {
    const s=this.snapshot;
    if(action==='find') {
      const target=s.objects.find(o=>o.id===payload.targetId);
      if(!target) throw Error('没有找到该物体记忆');
      if(s.task && !['succeeded','failed','cancelled'].includes(s.task.state)) throw Error('请先结束当前任务');
      this.target=target;this.origin=[...s.robot.position];this.ticks=0;
      s.task={id:crypto.randomUUID(),targetId:target.id,state:'searching',message:'正在查询演示空间记忆',updatedAt:new Date().toISOString()};
      s.path=[];
    } else if(action==='cancel' && s.task) { s.task.state='cancelled';s.task.message='演示任务已停止';s.robot.speed=0;s.robot.status='已停止';s.path=[]; }
    else if(action==='pause' && s.task) { this.beforePause=s.task.state;s.task.state='paused';s.robot.speed=0;s.task.message='演示任务已暂停'; }
    else if(action==='resume' && s.task?.state==='paused') { s.task.state=this.beforePause;s.task.message='演示任务已继续'; }
    else throw Error('当前状态不支持该操作');
    if(s.task)s.task.updatedAt=new Date().toISOString();
    this.publish();return {accepted:true};
  }
  tick() {
    const s=this.snapshot,t=s.task;
    if(t && !['succeeded','failed','cancelled','paused'].includes(t.state)) {
      this.ticks++;
      if(this.ticks<2) {t.state='searching';t.message='已找到物体记忆，准备选择停靠点';}
      else if(this.ticks<4) {t.state='planning';t.message='演示路线已生成，停靠点位于目标前方';s.path=[this.origin,[this.origin[0],.7,0],[this.target.position[0]-.75,.7,0],[this.target.position[0]-.75,this.target.position[1]-.6,0]];}
      else if(this.ticks<13) {
        t.state='navigating';t.message='沿演示路径前往记忆位置';s.robot.status='行驶中';s.robot.speed=.25;
        const p=(this.ticks-4)/8*3,i=Math.min(2,Math.floor(p)),f=Math.min(1,p-i),a=s.path[i],b=s.path[i+1];
        s.robot.position=a.map((n,k)=>n+(b[k]-n)*f);s.robot.yaw=Math.atan2(b[1]-a[1],b[0]-a[0]);
      } else if(this.ticks<16) {t.state='confirming';t.message='正在模拟现场 RGB-D 复核，不是真实识别结果';s.robot.speed=0;s.robot.status='确认目标';}
      else {t.state='succeeded';t.message='演示完成：已模拟确认目标，实车需由 M5 回传确认结果';s.robot.speed=0;s.robot.status='待命';this.target.lastSeen=new Date().toISOString();}
      t.updatedAt=new Date().toISOString();
    }
    this.publish();
  }
  close() {clearInterval(this.timer);this.emit('connection',{state:'disconnected'});}
}
