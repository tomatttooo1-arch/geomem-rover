// 纯 node 单测（无依赖）：sourceDetail 宽松读取与来源横幅（10-15 联调第 2 棒小修 a）。
// 用法：node ui/geomem/tools/test/unit_source_detail.mjs → 全过打印 ok，否则非零退出。
import assert from 'node:assert/strict';
import { sourceDetailOf, sourceBanner, sourceZh, validateSnapshot, validateHello } from '../../app/src/model.js';

const REC = '回放：实车真实录制，非实时';            // T1-A7 预注册字面，不得改
assert.equal(sourceBanner('replay', 'recording')[0], REC);
assert.equal(sourceBanner('replay', 'synthetic')[0], '回放：合成测试数据（不是实车录制），非实时');
assert.equal(sourceBanner('replay', null)[0], '回放数据，非实时');
for (const d of ['synthetic', null]) assert.ok(!sourceBanner('replay', d)[0].includes('实车真实录制'), String(d));
assert.equal(sourceBanner('live', 'recording'), null);
assert.equal(sourceBanner('sim', null)[0], '仿真数据：不是实车画面');
assert.equal(sourceZh('replay', 'recording'), '回放（实车真实录制）');
assert.ok(!sourceZh('replay', 'synthetic').includes('实车'));
assert.ok(!sourceZh('replay', null).includes('实车'));

// 表外值、非字符串、缺省、null 都按 null
for (const v of [undefined, null, 'live', 'RECORDING', 7, {}, [], 'x'.repeat(100)]) {
  const m = v === undefined ? {} : { sourceDetail: v };
  assert.equal(sourceDetailOf(m), null, String(v));
}
assert.equal(sourceDetailOf({ sourceDetail: 'recording' }), 'recording');
assert.equal(sourceDetailOf({ sourceDetail: 'synthetic' }), 'synthetic');
assert.equal(sourceDetailOf(null), null);

// 校验器不因 sourceDetail 整帧拒收（阳性对照：同一帧改坏 source 必拒）
const snap = { type: 'snapshot', version: 2, seq: 1, serverTime: null, source: 'replay', link: { state: 'up', hubAgeMs: 0 },
  map: { id: null, name: null, frame: 'odom', units: 'm', cloudRev: null, placeholder: false }, robot: null,
  memory: null, objects: [], path: [], task: null };
for (const v of [undefined, null, 'recording', 'weird', 42, { a: 1 }]) {
  const s = { ...snap }; if (v !== undefined) s.sourceDetail = v;
  assert.doesNotThrow(() => validateSnapshot(s), String(v));
}
assert.throws(() => validateSnapshot({ ...snap, source: 'bogus' }));
const hello = { type: 'hello', version: 2, bridge: { id: 'b', hubBootId: null, link: 'up' }, source: 'replay', capabilities: {} };
for (const v of [undefined, null, 'synthetic', 'weird', 3]) {
  const h = { ...hello }; if (v !== undefined) h.sourceDetail = v;
  assert.doesNotThrow(() => validateHello(h), String(v));
}
assert.throws(() => validateHello({ ...hello, source: 'bogus' }));
console.log('ok');
