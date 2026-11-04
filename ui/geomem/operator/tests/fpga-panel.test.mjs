import test from 'node:test';
import assert from 'node:assert/strict';
import {fmtMs,fmtCount,RenewalRate,fpgaModel,fpgaHtml,STALE_MS} from '../src/fpga-panel.js';

const live={framesAgeMs:100,fps:9.8,geoMs:3.1,pixels:256000,roiMs:0.42,roiBoxes:3,roiFrames:6,rollMs:0.28,
  rollCells:412,rollFrames:10,frames:10,rttMs:14.2,syncBoundMs:0.75,permit:true,renewals:1234,estop:false};
const byKey=m=>Object.fromEntries(m.tiles.map(t=>[t.key,t]));

test('number formats: small ms keep two decimals, large counts in 万, junk is a dash',()=>{
  assert.equal(fmtMs(0.284),'0.28');assert.equal(fmtMs(3.14),'3.1');assert.equal(fmtMs(14.2),'14');
  assert.equal(fmtMs(null),'—');assert.equal(fmtMs(NaN),'—');assert.equal(fmtMs(-1),'—');
  assert.equal(fmtCount(256000),'25.6 万');assert.equal(fmtCount(412),'412');assert.equal(fmtCount(1.5),'—');
});

test('all six tiles from live numbers',()=>{
  const m=fpgaModel(live,9.6),t=byKey(m);
  assert.deepEqual(m.tiles.map(x=>x.key),['geo','roi','roll','fps','link','permit']);
  assert.deepEqual([t.geo.value,t.geo.unit,t.geo.sub],['3.1','ms','每帧 25.6 万像素 → 体素']);
  assert.deepEqual([t.roi.value,t.roi.sub],['0.42','核对 3 个物体框']);
  assert.deepEqual([t.roll.value,t.roll.sub],['0.28','更新 412 个地图格']);
  assert.deepEqual([t.fps.value,t.fps.unit],['9.8','帧/秒']);
  assert.deepEqual([t.link.value,t.link.sub],['14','对时误差 ≤ 0.75 ms']);
  assert.deepEqual([t.permit.value,t.permit.state,t.permit.sub],['续约中','on','每秒续约 9.6 次']);
  assert.equal(m.level,'ok');assert.equal(m.pulse,true);
  assert.deepEqual(m.chips,['几何 3.1 ms','证据 0.42 ms','融合 0.28 ms','9.8 帧/秒','许可 续约中']);
});

test('no data (stale or never) -> every value a dash, never an old number',()=>{
  for(const m of [fpgaModel(null),fpgaModel({}),fpgaModel({...live,fps:null,geoMs:null,pixels:null,roiMs:null,
    roiBoxes:null,roiFrames:null,rollMs:null,rollCells:null,rttMs:null,syncBoundMs:null,permit:null,renewals:null})]){
    for(const t of m.tiles){assert.equal(t.value,'—',t.key);assert.equal(t.unit,'',t.key);assert.equal(t.sub,'—',t.key);}
    assert.equal(m.level,'idle');assert.equal(m.pulse,false);
  }
});

test('evidence core idle when no remembered object is in view; permit off / e-stop',()=>{
  let t=byKey(fpgaModel({...live,roiMs:null,roiBoxes:null,roiFrames:0}));
  assert.deepEqual([t.roi.value,t.roi.sub],['—','视野内无记忆物体']);
  const m=fpgaModel({...live,permit:false},null);t=byKey(m);
  assert.deepEqual([t.permit.value,t.permit.state],['已撤销','off']);assert.equal(m.level,'warn');assert.equal(m.pulse,false);
  t=byKey(fpgaModel({...live,permit:false,estop:true}));assert.equal(t.permit.value,'急停');
  t=byKey(fpgaModel(live,null));assert.equal(t.permit.sub,'—');    // permit on, rate not known yet
});

test('renewal rate from the PL accepted-frame counter',()=>{
  const r=new RenewalRate(3000);
  assert.equal(r.value(0),null);
  r.feed(0,100);r.feed(400,104);assert.equal(r.value(400),null);   // < 0.5 s of samples
  r.feed(1000,110);assert.equal(r.value(1000),10);
  r.feed(1000,111);                                                   // same time: ignored
  for(let t=1100;t<=6000;t+=100)r.feed(t,110+(t-1000)/100);           // 10 per second, window slides
  assert.ok(Math.abs(r.value(6000)-10)<1e-9);assert.ok(6000-r.samples[0].t<=3100);
  assert.equal(r.value(6000+STALE_MS+1),null);                        // no new sample: no rate
  r.feed(6100,3);assert.equal(r.samples.length,1);                    // counter went back (KV restarted)
  r.feed(6200,null);assert.equal(r.samples.length,0);
});

test('html escapes and marks the beating permit',()=>{
  const h=fpgaHtml(fpgaModel(live,9.6));
  assert.match(h,/<summary><span class="fpga-tag">FPGA 实时<\/span>/);
  assert.match(h,/fpga-beat beating/);assert.equal((h.match(/<li class="fpga-tile/g)||[]).length,6);
  assert.match(h,/fpga-tile on" data-key="permit"/);
  assert.doesNotMatch(fpgaHtml(fpgaModel(null)),/beating/);
});
