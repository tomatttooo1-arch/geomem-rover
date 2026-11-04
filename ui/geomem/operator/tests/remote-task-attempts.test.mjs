import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';

// 10-25 find-retry: the competition console's task line shows how many positions the car has tried (src/remote.js).
const src = readFileSync(new URL('../src/remote.js', import.meta.url), 'utf8');
const m = src.match(/function taskMessageText\(t\)\{.*?\}\n/);
assert.ok(m, 'taskMessageText not found in remote.js');
const taskMessageText = new Function(m[0] + 'return taskMessageText;')();

test('attempted positions follow the task message', () => {
  assert.equal(taskMessageText({state: 'navigating', message: '没能到达，已停车并清理地图，马上重试', positionsTried: 1}),
    '没能到达，已停车并清理地图，马上重试（已尝试 1 个位置）');
  assert.equal(taskMessageText({state: 'failed', message: '导航重试几次仍没能到达', positionsTried: 2}),
    '导航重试几次仍没能到达（已尝试 2 个位置）');
  assert.equal(taskMessageText({state: 'navigating', message: '正在前往目标附近'}), '正在前往目标附近');
  assert.equal(taskMessageText({state: 'navigating', message: '正在前往目标附近', positionsTried: 0}), '正在前往目标附近');
  assert.equal(taskMessageText({state: 'navigating', message: 'x', positionsTried: '2'}), 'x');
  assert.equal(taskMessageText({state: 'idle', message: '待命', positionsTried: 2}), '待命');
  assert.equal(taskMessageText(null), '从记忆出发，到现场重新确认。');
  assert.ok(src.includes("textContent=ok?taskMessageText(snapshot.task):"), 'the task line uses it');
});
