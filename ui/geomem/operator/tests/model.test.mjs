import test from 'node:test';
import assert from 'node:assert/strict';
import {validateSnapshot,matchObjects,endpointURL,canStart,snapshotFresh} from '../src/model.js';
import {demoSnapshot,DemoAdapter} from '../src/demo.js';
import {LiveAdapter} from '../src/transport.js';
test('query aliases and multiple instances remain explicit',()=>{
  const s=demoSnapshot();s.objects.push({...s.objects[0],id:'bag-02'});
  assert.equal(matchObjects(s.objects,'帮我找书包').length,2);
  assert.equal(matchObjects(s.objects,'杯子')[0].className,'cup');
  assert.equal(matchObjects(s.objects,'找').length,0);assert.equal(matchObjects(s.objects,'钥匙').length,0);
});
test('schema rejects bad geometry, coordinates, duplicate IDs and unknown task states',()=>{
  assert.equal(validateSnapshot(demoSnapshot()).version,1);
  for(const mutate of [s=>s.map.units='mm',s=>s.map.frame='odom',s=>s.robot.position[0]=NaN,s=>s.objects.push({...s.objects[0]}),s=>s.points[0][3]=255,s=>s.seq=-1,s=>s.task={id:'t',targetId:'bag-01',state:'done',updatedAt:new Date().toISOString(),message:''}]){const s=demoSnapshot();mutate(s);assert.throws(()=>validateSnapshot(s));}
});
test('freshness checks receive age and source timestamp; unsafe starts blocked',()=>{
  const s=demoSnapshot(),now=Date.now();assert.ok(snapshotFresh(s,now,now));assert.ok(!snapshotFresh(s,now-6000,now));s.timestamp=new Date(now-10000).toISOString();assert.ok(!snapshotFresh(s,now,now));s.timestamp=new Date(now+10000).toISOString();assert.ok(!snapshotFresh(s,now,now));
  assert.ok(canStart(s,true,true));assert.ok(!canStart(s,false,true));assert.ok(!canStart(s,true,false));s.robot.localized=false;assert.ok(!canStart(s,true,true));
});
test('endpoints reject credentials, fragments, wrong protocols and insecure mixed content',()=>{
  assert.equal(endpointURL('ws://localhost:9090/m8'),'ws://localhost:9090/m8');
  for(const s of ['http://localhost','ws://user:secret@host','ws://host/#secret'])assert.throws(()=>endpointURL(s));
  assert.throws(()=>endpointURL('ws://host',true));assert.equal(endpointURL('wss://host/m8',true),'wss://host/m8');
});
test('demo reaches target only after confirming, pauses, and stops without moving again',async()=>{
  const d=new DemoAdapter();await d.command('find',{targetId:'bag-01'});assert.equal(d.snapshot.task.state,'searching');
  for(let i=0;i<6;i++)d.tick();assert.equal(d.snapshot.task.state,'navigating');await d.command('pause');const p=[...d.snapshot.robot.position];d.tick();assert.deepEqual(d.snapshot.robot.position,p);await d.command('resume');
  while(d.ticks<13)d.tick();assert.equal(d.snapshot.task.state,'confirming');assert.equal(d.snapshot.robot.speed,0);while(d.ticks<16)d.tick();assert.equal(d.snapshot.task.state,'succeeded');assert.notDeepEqual(d.snapshot.robot.position,d.target.position);
  await d.command('find',{targetId:'cup-01'});await d.command('cancel');const stopped=[...d.snapshot.robot.position];for(let i=0;i<20;i++)d.tick();assert.equal(d.snapshot.task.state,'cancelled');assert.deepEqual(d.snapshot.robot.position,stopped);
});
test('live adapter ignores reordered snapshots, requires correlated ack and rejects pending on disconnect',async()=>{
  const Original=globalThis.WebSocket;
  class Socket {static OPEN=1;constructor(){this.readyState=1;}send(s){this.sent=JSON.parse(s);}close(){this.readyState=3;}}
  globalThis.WebSocket=Socket;
  try{const a=new LiveAdapter('ws://localhost');a.connect();a.ws.onopen();let count=0;a.addEventListener('snapshot',()=>count++);const s=demoSnapshot();s.seq=4;a.ws.onmessage({data:JSON.stringify(s)});s.seq=3;a.ws.onmessage({data:JSON.stringify(s)});assert.equal(count,1);
    const p=a.command('find',{targetId:'bag-01'});a.ws.onmessage({data:JSON.stringify({type:'ack',version:1,id:'wrong-id',accepted:true})});assert.equal(a.pending.size,1);a.ws.onmessage({data:JSON.stringify({type:'ack',version:1,id:a.ws.sent.id,accepted:true})});assert.equal((await p).accepted,true);
    const pending=a.command('cancel');const check=assert.rejects(pending,/结果未知|是否执行未知/);a.close();await check;
  }finally{globalThis.WebSocket=Original;}
});
