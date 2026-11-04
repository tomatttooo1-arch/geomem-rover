import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {MapResetController, mapResetBlocker, mapResetOutcome, MAP_RESET_CONFIRM, MAP_RESET_DONE, MAP_RESET_LOCALIZATION} from '../src/map-reset.js';
import {normalizeCapabilities, reasonText} from '../src/v2/model.js';

// 10-26「重置地图」: button -> confirm -> command map_reset {confirm:true} -> result -> page cleared.
const caps = {mapReset: true};
const snap = state => ({mapping: state === undefined ? {} : {state, session: 0, map_loaded: null}});

test('availability: only with a mapping session; localization mode greyed with the reason', () => {
  assert.equal(mapResetBlocker(snap('mapping'), caps), '');
  assert.equal(mapResetBlocker({mapping: {state: 'exported', map_loaded: true}}, caps), '');
  assert.equal(mapResetBlocker(snap('failed'), caps), '');
  assert.equal(mapResetBlocker(snap(), caps), MAP_RESET_LOCALIZATION);          // hub has no mapping row
  assert.equal(mapResetBlocker({}, caps), MAP_RESET_LOCALIZATION);
  assert.match(mapResetBlocker(snap('saving'), caps), /正在保存/);
  assert.match(mapResetBlocker({mapping: {state: 'exported', map_loaded: null}}, caps), /正在保存/);
  assert.match(mapResetBlocker(snap('resetting'), caps), /正在重置/);
  assert.equal(mapResetBlocker(snap('mapping'), {}), '车栈未提供此功能');
  assert.equal(normalizeCapabilities({mapReset: true}).mapReset, true);
  assert.equal(normalizeCapabilities({}).mapReset, false);
  assert.equal(reasonText('', 'map_reset_no_mapping'), '定位模式（使用已存地图）或本趟没有启用建图，不能重置地图');
});

test('controller: confirm sends map_reset {confirm:true}; done clears the page and says so', async () => {
  const sent = [], states = [];
  let cleared = 0;
  const c = new MapResetController({send: async (a, p) => { sent.push([a, p]); return 'id-1'; }, clear: () => cleared++,
    onChange: s => states.push(s.phase)});
  assert.equal(c.ask(''), true);
  assert.equal(c.phase, 'confirm');
  assert.equal(c.text, MAP_RESET_CONFIRM);
  assert.equal(c.text, '将清空地图和记忆，车会停下');
  await c.confirm('');
  assert.deepEqual(sent, [['map_reset', {confirm: true}]]);
  assert.equal(c.pending, true);
  assert.equal(c.buttonLabel(), '正在重置…');
  assert.equal(c.onResult({id: 'other', action: 'map_reset', status: 'done'}), false);   // not ours
  assert.equal(c.onResult({id: 'id-1', action: 'mapping_stop', status: 'done'}), false);
  assert.equal(cleared, 0);
  assert.equal(c.onResult({id: 'id-1', action: 'map_reset', status: 'done'}), true);
  assert.equal(cleared, 1);
  assert.equal(c.phase, 'done');
  assert.equal(c.text, MAP_RESET_DONE);
  assert.equal(c.text, '已重置，可以重新建图');
  assert.deepEqual(states, ['confirm', 'pending', 'done']);
  assert.equal(c.buttonLabel(), '重置地图');
});

test('controller: result before the id resolves (settle emits before the ack promise) still counts', async () => {
  let cleared = 0, c;
  c = new MapResetController({send: async () => { c.onResult({id: 'x', action: 'map_reset', status: 'done'}); return 'x'; },
    clear: () => cleared++});
  await c.confirm('');
  assert.equal(c.phase, 'done');
  assert.equal(cleared, 1);
});

test('controller: refused / failed / unknown / disconnect never clear the page', async () => {
  let cleared = 0;
  const reject = new MapResetController({send: async () => { throw Error('定位模式（使用已存地图）或本趟没有启用建图，不能重置地图'); },
    clear: () => cleared++});
  await reject.confirm('');
  assert.equal(reject.phase, 'failed');
  assert.match(reject.text, /定位模式/);
  const fail = new MapResetController({send: async () => 'f', clear: () => cleared++});
  await fail.confirm('');
  fail.onResult({id: 'f', action: 'map_reset', status: 'rejected', reason: '地图重置没有全部完成：重置建图没有完成（建图进程 30 秒内没有回应）'});
  assert.equal(fail.phase, 'failed');
  assert.match(fail.text, /30 秒内没有回应/);
  const unk = new MapResetController({send: async () => 'u', clear: () => cleared++});
  await unk.confirm('');
  unk.onResult({id: 'u', action: 'map_reset', status: 'unknown', reason: '没有收到执行结果'});
  assert.match(unk.text, /结果未知/);
  const dc = new MapResetController({send: async () => 'd', clear: () => cleared++});
  await dc.confirm('');
  dc.onDisconnect();
  assert.match(dc.text, /连接断开/);
  const blocked = new MapResetController({send: async () => { throw Error('must not send'); }, clear: () => cleared++});
  assert.equal(blocked.ask(MAP_RESET_LOCALIZATION), false);
  assert.equal(blocked.text, MAP_RESET_LOCALIZATION);
  assert.equal(await blocked.confirm(MAP_RESET_LOCALIZATION), null);
  assert.equal(cleared, 0);
  assert.deepEqual(mapResetOutcome({status: 'done'}), {ok: true, clear: true, text: MAP_RESET_DONE});
});

test('wiring: button + dialog on /#remote, result routed, page cleared (voxels, cloud, base map, scene, heat map)', () => {
  const html = readFileSync(new URL('../index.html', import.meta.url), 'utf8');
  assert.match(html, /id="remote-map-reset-open"[^>]*>重置地图</);
  assert.match(html, /id="remote-map-reset-dialog"[\s\S]*将清空地图和记忆，车会停下/);
  assert.match(html, /id="remote-map-reset-confirm"/);
  const src = readFileSync(new URL('../src/remote.js', import.meta.url), 'utf8');
  const fn = src.match(/function clearAfterMapReset\(\)\{(.*?)\}\n/);
  assert.ok(fn, 'clearAfterMapReset not found');
  for (const step of ['hideBelief()', 'clearMapping(true)', 'cloud=[]', 'scene?.clearBasemap()', 'scene?.clear()', 'paintMap()'])
    assert.ok(fn[1].includes(step), step);
  assert.match(src, /mapReset\.onResult\(d\)/);
  assert.match(src, /mapReset\.onDisconnect\(\)/);
  const transport = readFileSync(new URL('../src/v2/transport.js', import.meta.url), 'utf8');
  assert.match(transport, /p\.action === 'map_reset' \? 40000/);
});
