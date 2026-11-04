import test from 'node:test';
import assert from 'node:assert/strict';
import {decodeMapping,transformPoint,mappingAgeLabel} from '../src/live-mapping.js';

async function packet(change={},corrupt=false){
  const body=new Uint8Array(30),floats=new Float32Array(body.buffer,0,6);floats.set([1,2,3,4,5,6]);body.set([255,0,0,0,128,255],24);
  const sha256=Buffer.from(await crypto.subtle.digest('SHA-256',body)).toString('hex');
  const meta={format:'geomem-live-mapping',version:1,source:'rtabmap',session:'session-a',scope:'boot-a',frame:'map',count:2,graphNodes:10,graphAgeMs:100,cloudAgeMs:120,sha256,mapToOdom:{position:[1,2,0],quaternion:[0,0,0,1]},...change};
  const header=new TextEncoder().encode(JSON.stringify(meta)),out=new Uint8Array(4+header.length+body.length);
  new DataView(out.buffer).setUint32(0,header.length,true);out.set(header,4);out.set(body,4+header.length);if(corrupt)out[out.length-1]^=1;
  return out.buffer;
}
test('RTAB cloud payload verifies bytes, colors and arbitrary JSON header alignment',async()=>{
  const d=await decodeMapping(await packet());assert.deepEqual([...d.positions],[1,2,3,4,5,6]);assert.deepEqual([...d.colors],[255,0,0,0,128,255]);
});
test('reject stale, wrong source/frame, truncated, corrupt or invalid transforms',async()=>{
  for(const bad of [{source:'demo'},{frame:'odom'},{cloudAgeMs:16000},{graphNodes:0},{count:3},{mapToOdom:{position:[0,0,0],quaternion:[0,0,0,2]}}])await assert.rejects(decodeMapping(await packet(bad)));
  await assert.rejects(decodeMapping(await packet({},true)));
  await assert.rejects(decodeMapping(new ArrayBuffer(3)));
});
test('map-to-odom rotation and translation keep marker coordinates aligned',()=>{
  const p=transformPoint([1,0,2],{position:[2,3,1],quaternion:[0,0,Math.SQRT1_2,Math.SQRT1_2]});
  assert(Math.abs(p[0]-2)<1e-10);assert(Math.abs(p[1]-4)<1e-10);assert.equal(p[2],3);
});
test('cached display requires the current live scope and keeps its real update age',async()=>{
  const data=await packet({cloudAgeMs:95000,cached:false});
  await assert.rejects(decodeMapping(data));
  await assert.rejects(decodeMapping(data,{cachedScope:'another-boot'}));
  const decoded=await decodeMapping(data,{cachedScope:'boot-a'});
  assert.equal(decoded.meta.cloudAgeMs,95000);assert.equal(decoded.meta.cached,true);
  assert.equal(mappingAgeLabel(decoded.meta),'缓存 · 1 分钟前更新');
  await assert.rejects(decodeMapping(await packet({},true),{cachedScope:'boot-a'}));
  await assert.rejects(decodeMapping(await packet({cloudAgeMs:NaN}),{cachedScope:'boot-a'}));
  assert.equal((await decodeMapping(await packet({cached:true}),{cachedScope:'boot-a'})).meta.cached,false);
});
