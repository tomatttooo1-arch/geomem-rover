// 纯 node 单测（无依赖）：里程计档允许寻物并提示（10-15 联调第 3 棒，控制器裁定 owner 默认）。
// 用法：node ui/geomem/tools/test/unit_odom_find.mjs → 全过打印 ok，否则非零退出。
import assert from 'node:assert/strict';
import { startBlocker, odomTier, validateSnapshot, POSE_ZH, ODOM_BADGE, ODOM_FIND_HINT } from '../../app/src/model.js';

const caps = { findText: true, control: 'controller' };
const base = { type: 'snapshot', version: 2, seq: 1, serverTime: 100, source: 'sim', link: { state: 'up', hubAgeMs: 0 },
  map: { id: null, name: null, frame: 'map', units: 'm', cloudRev: null, placeholder: false }, robot: null,
  memory: null, objects: [], path: [], task: null };
const robot = pose => ({ position: [0.4, 0.2, 0], yaw: 0, speed: 0, pose, poseAgeMs: 40, status: '待命' });
const blk = s => startBlocker({ snapshot: s, connected: true, fresh: true, caps });

// 仿真档、map 坐标系、桥退到里程计位姿 → 可以找物，且算里程计档（页面出「仅里程计定位」提示）
const odo = { ...base, robot: robot('odometry') };
assert.doesNotThrow(() => validateSnapshot(odo));
assert.equal(blk(odo), '');
assert.equal(odomTier(odo), true);
// 有地图位姿照旧（实车不削弱）：可以找物、不算里程计档
const map = { ...base, source: 'live', robot: robot('map') };
assert.equal(blk(map), ''); assert.equal(odomTier(map), false);
// 阳性对照：没有任何位姿（修前仿真档）→ 挡；定位丢失 → 挡
assert.match(blk(base), /没有收到小车位姿/);
assert.match(blk({ ...base, source: 'live', robot: robot('none') }), /尚没有完成定位/);
// 文字：含「仅里程计定位」，并保留 A8 判据读的「里程推算」
for (const t of [POSE_ZH.odometry, ODOM_BADGE, ODOM_FIND_HINT]) { assert.match(t, /仅里程计定位/); assert.match(t, /里程推算/); }
assert.match(ODOM_FIND_HINT, /仍可发起寻物/);
console.log('ok');
