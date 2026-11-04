import test from 'node:test';
import assert from 'node:assert/strict';
import {decodeBelief,cellCenter,beliefCorners,toScene,beliefRGBA,beliefCaption,doneHideDelay,gridPoint} from '../src/belief-layer.js';

const msg=(change={})=>({type:'belief',version:2,phase:'search',task:2,seq:5,frame:'map',resolution:.2,width:3,height:2,origin:[1,-2,0],
  cells:Buffer.from(Int8Array.from([-1,0,50,100,20,-1]).buffer).toString('base64'),next:[1.5,0,1.57],seen:[[0,0,0]],massLeft:.8,result:null,reason:null,ageMs:120,...change});
const near=(a,b,eps=1e-9)=>a.every((v,i)=>Math.abs(v-b[i])<eps);

test('decode: int8 cells (-1 kept), poses, phases',()=>{
  const b=decodeBelief(msg());
  assert.deepEqual([...b.values],[-1,0,50,100,20,-1]);
  assert.deepEqual([b.phase,b.width,b.height,b.next,b.seen.length],['search',3,2,[1.5,0,1.57],1]);
  for(const bad of [{phase:'x'},{width:4},{cells:'%%%'},{origin:[0,0]},{resolution:0},{type:'cloud'}])assert.throws(()=>decodeBelief(msg(bad)));
  const none=decodeBelief(msg({phase:'none',width:0,height:0,cells:''}));assert.equal(none.values.length,0);
});
test('OccupancyGrid convention: row 0 col 0 lower-left corner at origin, rows along +y, cols along +x',()=>{
  const b=decodeBelief(msg());
  assert(near(cellCenter(b,0,0),[1.1,-1.9]));
  assert(near(cellCenter(b,1,2),[1.5,-1.7]));      // row 1 = second row up (+y), col 2 = third column (+x)
  assert(near(beliefCorners(b)[2],[1.6,-1.6]));
});
test('origin yaw rotates the grid like grid_to_points (bridge) and BeliefSearch._xy',()=>{
  const b=decodeBelief(msg({origin:[1,2,Math.PI/2]}));
  // a column step (+x of the grid) goes along +y of the map when yaw = 90 deg
  assert(near(gridPoint(b,1,0),[1,2.2],1e-12));
  assert(near(gridPoint(b,0,1),[.8,2],1e-12));
  // python: ox + c*lx - s*ly, oy + s*lx + c*ly for (lx, ly) = ((i+.5)res, (j+.5)res)
  const [x,y]=cellCenter(b,1,2),c=Math.cos(Math.PI/2),s=Math.sin(Math.PI/2),lx=.5,ly=.3;
  assert(near([x,y],[1+c*lx-s*ly,2+s*lx+c*ly],1e-12));
});
test('scene axes match scene.js world(): (x, y) -> three (x, z, -y)',()=>{
  assert.deepEqual(toScene([1.5,-2],.03),[1.5,.03,2]);
});
test('texture texel order = cells order (DataTexture row 0 at uv v = 0 = the origin side); colour ramps',()=>{
  const b=decodeBelief(msg()),rgba=beliefRGBA(b);
  assert.equal(rgba.length,24);
  assert.equal(rgba[3],0);assert.equal(rgba[23],0);                    // -1 -> transparent
  assert(rgba[3*4+3]>rgba[2*4+3]&&rgba[2*4+3]>rgba[1*4+3]&&rgba[1*4+3]>0);   // 100 > 50 > 0 (seen: faint, not gone)
  assert(rgba[3*4+1]<rgba[1*4+1]);                                       // hotter = redder (less green)
  const pre=beliefRGBA(decodeBelief(msg({phase:'preview'})));assert(pre[3*4+3]<rgba[3*4+3]);   // preview lighter
});
test('captions and done fade timing',()=>{
  assert.equal(beliefCaption(decodeBelief(msg({phase:'preview'}))),'若不在原处，将这样找');
  assert.equal(beliefCaption(decodeBelief(msg({phase:'done',result:'found'}))),'已找到');
  assert.equal(beliefCaption(decodeBelief(msg({phase:'done',result:'not_found'}))),'已搜完，未找到');
  assert.equal(doneHideDelay(decodeBelief(msg({phase:'done',ageMs:500}))),2500);
  assert.equal(doneHideDelay(decodeBelief(msg({phase:'done',ageMs:20000}))),0);
});
