import test from 'node:test';
import assert from 'node:assert/strict';
import {readAr,containRect,mapBox,boxName,drawBoxes,VERDICT_COLOUR} from '../src/fpv-boxes.js';

const ok={state:'ok',dtMs:10,frameId:3,depthWh:[640,480],hidden:0,boxes:[
  {id:'t1',name:'瓶子 1',label:'bottle',number:1,v:0,xyxy:[100,100,199,199]},
  {id:'t2',name:'杯子',label:'cup',number:null,v:3,xyxy:[300,200,339,259]},
  {id:'bad',name:'x',v:9,xyxy:[0,0,1,1]}]};

test('readAr: ok draws (bad verdict dropped), stale/none warn, missing ar = no box data',()=>{
  const a=readAr({ar:ok});assert.equal(a.state,'ok');assert.deepEqual(a.boxes.map(b=>b.id),['t1','t2']);assert.equal(a.note,'');
  assert.deepEqual(readAr({ar:{state:'stale',boxes:[]}}),{state:'stale',boxes:[],note:'识别框过期'});
  assert.equal(readAr({w:640}).note,'无识别框数据');assert.equal(readAr(null).state,'none');
  assert.deepEqual(readAr({ar:{state:'ok',boxes:[]}}).boxes,[]);          // nothing in view: ok, no warning
});
test('car screen colours: 0 white, 2 magenta, 3 grey (tools/ar_display/golden.py)',()=>{
  assert.deepEqual(VERDICT_COLOUR,{0:'#ffffff',2:'#ff00ff',3:'#808080'});
});
test('contain rect + closed-interval box mapping',()=>{
  const r=containRect(800,500,640,480);                 // 16:10 stage, 4:3 picture -> pillarbox
  const exp={x:(800-500*640/480)/2,y:0,w:500*640/480,h:500};for(const k in exp)assert.ok(Math.abs(r[k]-exp[k])<1e-9,k);
  const m=mapBox([0,0,639,479],[640,480],640,480,r);   // whole depth image = whole picture
  assert.ok(Math.abs(m.x-r.x)<1e-9&&Math.abs(m.w-r.w)<1e-9&&Math.abs(m.h-r.h)<1e-9);
  const half=mapBox([0,0,159,119],[320,240],640,480,r);   // stride-2 depth: same picture half
  assert.ok(Math.abs(half.w-r.w/2)<1e-9);
  assert.equal(containRect(0,10,640,480),null);
});
test('name: snapshot object first, then bridge name, then label',()=>{
  assert.equal(boxName(ok.boxes[0],[{id:'t1',name:'瓶子 1（新）'}]),'瓶子 1（新）');
  assert.equal(boxName(ok.boxes[1],[]),'杯子');
  assert.equal(boxName({label:'key',xyxy:[0,0,1,1],v:0},null),'key');
});
test('draw: strokes per box, label bar in the box colour, clears when not ok',()=>{
  const ops=[];const g={setTransform(){},clearRect(){ops.push('clear');},strokeRect(...a){ops.push(['stroke',this.strokeStyle]);},
    fillRect(...a){ops.push(['fill',this.fillStyle]);},fillText(t){ops.push(['text',t]);},measureText:t=>({width:t.length*10}),font:'',textBaseline:''};
  const canvas={width:0,height:0,parentElement:{clientWidth:640,clientHeight:480},getContext:()=>g};
  const img={hidden:false,naturalWidth:640,naturalHeight:480};
  assert.equal(drawBoxes(canvas,img,readAr({ar:ok}),[]),2);
  assert.deepEqual(ops.filter(o=>o[0]==='stroke').map(o=>o[1]),['#808080','#ffffff']);   // first row on top: drawn last
  assert.deepEqual(ops.filter(o=>o[0]==='text').map(o=>o[1]),['杯子','瓶子 1']);
  ops.length=0;assert.equal(drawBoxes(canvas,img,readAr({}),[]),0);assert.deepEqual(ops,['clear']);
});
