import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';

// 10-25: the vehicle status cell of the competition console (src/remote.js). The real car runs on odometry; the
// odometry badge used to replace every status, so 急停锁存 and the camera-too-close hint never reached the screen.
const src = readFileSync(new URL('../src/remote.js', import.meta.url), 'utf8');
const m = src.match(/function robotStatusText\(r\)\{.*?\}\n/);
assert.ok(m, 'robotStatusText not found in remote.js');
const robotStatusText = new Function(m[0] + 'return robotStatusText;')();

test('odometry badge only while idle; other statuses stay visible', () => {
  assert.equal(robotStatusText({pose: 'odometry', status: '待命'}), '仅里程计定位');
  assert.equal(robotStatusText({pose: 'odometry', status: '相机前方过近，请后退'}), '相机前方过近，请后退');
  assert.equal(robotStatusText({pose: 'odometry', status: '急停锁存（相机前方过近，请后退）'}), '急停锁存（相机前方过近，请后退）');
  assert.equal(robotStatusText({pose: 'odometry', status: '正在前往'}), '正在前往');
  assert.equal(robotStatusText({pose: 'map', status: '待命'}), '待命');
  assert.equal(robotStatusText({pose: 'map'}), '未知');
  assert.equal(robotStatusText(null), '未知');
});
