import assert from 'node:assert/strict';
import {LiveAdapter} from '../../app/src/transport.js';
import {normalizeCapabilities} from '../../app/src/model.js';
// Deterministic elapsed-time checks: no socket is ever opened.
const original={setTimeout:globalThis.setTimeout,clearTimeout:globalThis.clearTimeout,WebSocket:globalThis.WebSocket};
let now=0,seq=0;const timers=new Map();
globalThis.setTimeout=(fn,ms)=>{const id=++seq;timers.set(id,{at:now+ms,fn});return id;};
globalThis.clearTimeout=id=>timers.delete(id);
globalThis.WebSocket={OPEN:1};
const advance=ms=>{now+=ms;for(const [id,t]of [...timers])if(t.at<=now){timers.delete(id);t.fn();}};
try{
const a=new LiveAdapter('ws://127.0.0.1/ws'),wire=[],results=[];a.ready=true;a.ws={readyState:1,send:raw=>wire.push(JSON.parse(raw))};a.addEventListener('result',e=>results.push(e.detail));
const ack=c=>a.receive(JSON.stringify({type:'ack',version:2,id:c.id,accepted:true}));
let c=a.command('teleop',{v:.05,w:0});ack(c);await c.acked;assert.equal(a.pending.size,0);advance(10000);assert.equal(results.length,0);
c=a.command('mapping_stop');ack(c);await c.acked;advance(10000);assert(a.pending.has(c.id));advance(174000);assert(a.pending.has(c.id));a.receive(JSON.stringify({type:'result',version:2,id:c.id,status:'done'}));assert.equal(results.at(-1).status,'done');assert.equal(a.pending.size,0);
c=a.command('estop');advance(8000);assert(a.pending.has(c.id));ack(c);await c.acked;advance(10000);assert(a.pending.has(c.id));a.receive(JSON.stringify({type:'result',version:2,id:c.id,status:'done'}));
c=a.command('find',{text:'瓶子'});const failed=c.acked.catch(e=>e.message);const count=wire.length;a.failPending('disconnected');assert.match(await failed,/disconnected/);advance(200000);assert.equal(wire.length,count);
assert.equal(normalizeCapabilities({teleop:true,mappingStop:true,release:true,goto:true}).mappingStop,true);
console.log('PASS: ACK-only teleop; mapping still pending at 184 s; estop long ACK/result; disconnect no command replay.');
}finally{Object.assign(globalThis,original);}
