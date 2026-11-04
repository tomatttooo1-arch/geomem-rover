// 10-22 page side of the voxel deltas: replays the Python feed's real wire bytes (fixture written by
// tests/test_mapping_delta.py --write-fixture) through inflateVoxelMessage + VoxelStore and compares every voxel.
import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {VoxelStore,MappingResync,voxelHash} from '../src/voxel-store.js';
import {inflateVoxelMessage} from '../src/live-mapping.js';
import {LiveAdapter} from '../src/v2/transport.js';
import {decodeBinary} from '../src/v2/model.js';

const fx=JSON.parse(readFileSync(new URL('./fixtures/mapping-delta.json',import.meta.url)));
const payloadOf=s=>{const b=Buffer.from(s.payload,'base64');return b.buffer.slice(b.byteOffset,b.byteOffset+b.byteLength);};

test('feed fixture: full, merged delta, keep, wrong base, bad hash, new epoch -- every voxel matches',async()=>{
  const store=new VoxelStore(),seen=[];
  for(const step of fx.steps){
    const {meta,raw}=await inflateVoxelMessage({header:step.header,payload:payloadOf(step)});
    if(meta.mode==='keep'){seen.push('keep');assert.equal(meta.epoch,store.epoch);assert.equal(meta.seq,store.seq);}
    else try{store.apply(meta,raw);seen.push(meta.mode);}
    catch(e){if(e instanceof MappingResync)seen.push('resync');else{seen.push('integrity');assert.equal(store.ready,false);}}
    if(step.expect)assert.deepEqual(store.entries(),step.expect);
  }
  assert.deepEqual(seen,['full','delta','delta','keep','resync','integrity','full','delta','full','delta']);
});

test('swap-remove keeps slots dense and the dirty list covers moved and new slots',async()=>{
  const store=new VoxelStore();
  const [full,delta]=fx.steps;
  let r=await inflateVoxelMessage({header:full.header,payload:payloadOf(full)});
  assert.equal(store.apply(r.meta,r.raw).dirty,null);
  const before=Float32Array.from(store.xyz.subarray(0,store.count*3));
  r=await inflateVoxelMessage({header:delta.header,payload:payloadOf(delta)});
  const {dirty}=store.apply(r.meta,r.raw),touched=new Set(dirty);
  for(const [k,i] of store.index){assert.equal(store.keys[i],k);assert(i<store.count);}
  assert.equal(store.index.size,store.count);
  // every slot not in dirty still holds what it held before (so scene.js may copy only the dirty slots)
  const n=Math.min(store.count,before.length/3);
  for(let i=0;i<n;i++)if(!touched.has(i))for(let a=0;a<3;a++)assert.equal(store.xyz[3*i+a],before[3*i+a]);
  // positions are voxel centres in the map frame
  const i=0,k=store.keys[i],rx=k%65536,ry=Math.floor(k/65536)%65536,rz=Math.floor(k/4294967296);
  assert(Math.abs(store.xyz[0]-(store.origin[0]+rx+.5)*store.cell)<1e-4&&Math.abs(store.xyz[2]-(store.origin[2]+rz+.5)*store.cell)<1e-4&&ry>=0);
});

test('voxel hash matches the Python key_hash fingerprint',()=>{
  // python: live_mapping.key_hash([1 | 2 << 16 | 3 << 32]) == voxelHash(1, 2, 3)
  assert.equal(voxelHash(1,2,3),fx.hash123);
  let h=0;for(const [k] of fx.steps[0].expect){const rx=k%65536,ry=Math.floor(k/65536)%65536,rz=Math.floor(k/4294967296);h^=voxelHash(rx,ry,rz);}
  assert.equal(h>>>0,fx.steps[0].header.keyHash);
});

function binaryMessage(header,payload){
  const head=new TextEncoder().encode(JSON.stringify(header)),pad=(4-(4+head.length)%4)%4;
  const out=new Uint8Array(4+head.length+pad+payload.length);new DataView(out.buffer).setUint32(0,head.length,true);out.set(head,4);out.set(payload,4+head.length+pad);return out.buffer;
}
test('chunk reassembly: in order -> one inner message; a new id drops the partial; out-of-order is discarded',()=>{
  const a=new LiveAdapter('ws://127.0.0.1:1/ws'),got=[];a.addEventListener('mapping',e=>got.push(e.detail));
  const body=new Uint8Array(150000).map((_,i)=>i*7&255),inner=new Uint8Array(binaryMessage({type:'mapping',version:2,mode:'full'},body));
  const parts=(id,size=65536)=>{const n=Math.ceil(inner.length/size);return Array.from({length:n},(_,i)=>decodeBinary(binaryMessage({type:'chunk',version:2,id,i,n,kind:'mapping',bytes:inner.length},inner.subarray(i*size,(i+1)*size))));};
  for(const c of parts(1))a.reassemble(c);
  assert.equal(got.length,1);assert.deepEqual(new Uint8Array(got[0].payload),body);
  const [p0,p1]=parts(2),q=parts(3);a.reassemble(p0);a.reassemble(p1);for(const c of q)a.reassemble(c);   // id 2 aborted by id 3
  assert.equal(got.length,2);assert.equal(a.chunkStats.dropped,1);
  const r=parts(4);a.reassemble(r[0]);a.reassemble(r[2]);a.reassemble(r[1]);                         // gap -> dropped
  assert.equal(got.length,2);
});
