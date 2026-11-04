// Local rehearsal only. No sockets, ROS, camera devices or motor connections.
import {DemoAdapter, demoCloud} from './demo.js';
import {normalizeCapabilities, newId} from './model.js';
export class CompetitionSim extends DemoAdapter {
  constructor(){super();this.snapshot.source='sim';this.snapshot.estop={latched:false,epoch:0};this.snapshot.mapping={state:'mapping'};this.up=true;this.lastDrive=0;this.velocity={v:0,w:0};this.closed=false;this.jobs=new Set();this.caps=normalizeCapabilities({findText:true,select:true,cancel:true,estop:true,release:true,teleop:true,mappingStop:true,mapReset:true,goto:true,control:'controller'});this.snapshot.objects.push(...[1,2].map((n)=>({id:`bottle-${n}`,name:`瓶子 ${n}`,className:'bottle',position:[n*1.3,n*.6,.5],lastSeen:1000,state:'historical_memory'})));}
  connect(){this.emit('hello',{hello:{type:'hello',version:2,bridge:{id:'local-simulation',hubBootId:'sim-1',link:'up'},source:'sim',capabilities:this.caps},caps:this.caps});this.emit('connection',{state:'connected'});this.emit('cloud',demoCloud());this.publish();this.timer=setInterval(()=>this.advance(),100);}
  publish(){if(this.up&&!this.closed)super.publish();}
  later(fn,ms){const id=setTimeout(()=>{this.jobs.delete(id);if(!this.closed)fn();},ms);this.jobs.add(id);}
  command(action,p={}){
    const id=newId(), reject=(why)=>({id,acked:Promise.reject(Error(why))}),s=this.snapshot;
    if(!this.up)return reject('模拟断线：请求未发送');
    const done=(reason='模拟执行完成')=>this.later(()=>{this.publish();this.emit('result',{id,action,status:'done',reason});},100);
    const stop=action==='teleop'&&(p.stop===true||(p.v===0&&p.w===0));
    if(s.estop.latched&&!['estop','release','cancel'].includes(action)&&!stop)return reject('急停锁存中');
    if(['find','select','goto'].includes(action)&&performance.now()-this.lastDrive<300)return reject('teleop_active：遥控在手，松手后重新发送');
    if(action==='teleop'){
      if(!stop&&(!Number.isFinite(p.v)||!Number.isFinite(p.w)||Math.abs(p.v)>.15||Math.abs(p.w)>.6))return reject('速度无效');
      this.velocity=stop?{v:0,w:0}:{v:p.v,w:p.w};this.lastDrive=stop?0:performance.now();
      if(!stop){this.setTask({state:'cancelled',message:'遥控已接管，原导航目标已撤销',candidates:[]});s.path=[];}
      s.robot.speed=Math.abs(this.velocity.v);this.publish();
    }else if(action==='estop'){s.estop={latched:true,epoch:s.estop.epoch+1};this.velocity={v:0,w:0};s.robot.speed=0;this.setTask({state:'cancelled',message:'模拟急停已锁存',candidates:[]});done();}
    else if(action==='release'){if(p.confirm!==true)return reject('confirm_required');if(p.epoch!==s.estop.epoch)return reject('epoch_mismatch');s.estop.latched=false;done();}
    else if(action==='mapping_stop'){if(s.mapping.state!=='mapping')return reject('本趟没有正在进行的建图');s.mapping={state:'saving'};this.publish();this.later(()=>{s.mapping={state:'exported'};done('模拟地图已导出；未生成真实地图库');},1800);}
    else if(action==='map_reset'){if(p.confirm!==true)return reject('confirm_required');if(!s.mapping?.state||s.mapping.state==='idle')return reject('定位模式（使用已存地图）或本趟没有启用建图，不能重置地图');const session=(s.mapping.session||0)+1;s.mapping={state:'resetting',session:session-1};this.velocity={v:0,w:0};s.robot.speed=0;this.setTask({state:'cancelled',message:'重置地图：已停车',candidates:[]});s.path=[];this.publish();this.later(()=>{s.memory.scope='sim-reset-'+newId();s.objects=[];s.mapping={state:'mapping',session};this.emit('cloud',{header:{count:0},positions:new Float32Array(),colors:new Uint8Array()});done('模拟地图已重置');},800);}
    else if(action==='goto'){if(!Number.isFinite(p.forwardM)||p.forwardM<0||p.forwardM>2||!Number.isFinite(p.turnRad))return reject('相对目标格式无效');s.robot.yaw+=p.turnRad;s.robot.position[0]+=Math.cos(s.robot.yaw)*p.forwardM;s.robot.position[1]+=Math.sin(s.robot.yaw)*p.forwardM;done();}
    else if(action==='cancel'){this.setTask({state:'cancelled',message:'模拟任务已撤销',candidates:[]});this.velocity={v:0,w:0};s.robot.speed=0;s.path=[];done();}
    else{if(action==='select'&&p.scope!==s.memory.scope)return reject('scope_changed');return super.command(action,p);}
    this.publish();return{id,acked:Promise.resolve({accepted:true,status:'forwarded'})};
  }
  advance(){const s=this.snapshot;if(performance.now()-this.lastDrive>300)this.velocity={v:0,w:0};if(!this.up)return;s.robot.yaw+=this.velocity.w*.1;s.robot.position[0]+=Math.cos(s.robot.yaw)*this.velocity.v*.1;s.robot.position[1]+=Math.sin(s.robot.yaw)*this.velocity.v*.1;if(this.velocity.v||this.velocity.w)s.robot.speed=Math.abs(this.velocity.v);if((this.steps=(this.steps||0)+1)%10===0)super.tick();else this.publish();}
  nextRun(){if(this.snapshot.mapping.state!=='exported')throw Error('先结束建图并等待导出');const s=this.snapshot;s.memory.scope='sim-run-'+newId();s.robot.position=[1,-1.8,0];s.robot.speed=0;s.objects.forEach(o=>o.state='historical_memory');s.task=null;s.mapping={state:'idle'};this.velocity={v:0,w:0};this.publish();}
  toggleLink(){this.up=!this.up;this.velocity={v:0,w:0};this.snapshot.robot.speed=0;this.emit('connection',{state:this.up?'connected':'disconnected'});this.publish();}
  close(){this.closed=true;clearInterval(this.timer);for(const id of this.jobs)clearTimeout(id);this.jobs.clear();this.emit('connection',{state:'disconnected'});}
}
