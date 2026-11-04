// 10-22 RTAB-Map display voxels kept on the page and updated by deltas from the Nano feed
// (ui/geomem/tools/competition/live_mapping.py VoxelFeed / encode_vdz; Python reference decoder: decode_vdz).
// No three.js here (node-testable); scene.js copies the dirty slots into its BufferGeometry.
//
// Wire (header.encoding 'vdz1', payload inflated): adds x|y|z planes, removes x|y|z planes (uint16, keys ascending,
// each plane delta-coded mod 2^16), then adds r|g|b planes (uint8). Voxel key = rx + ry*2^16 + rz*2^32 (relative
// to the epoch's integer origin), drawn at (origin + r + 0.5) * cell in the map frame.
// header: {mode:'full'|'delta', epoch, seq, baseSeq, cell, origin:[3], adds, removes, rawBytes, count, keyHash}.

export class MappingResync extends Error {}

export function voxelHash(rx, ry, rz) {
  let h = Math.imul(rx | (ry << 16), 0x9E3779B1) ^ Math.imul(rz, 0x85EBCA77);
  h ^= h >>> 16; h = Math.imul(h, 0x85EBCA6B); h ^= h >>> 13; h = Math.imul(h, 0xC2B2AE35); h ^= h >>> 16;
  return h >>> 0;
}

const safe = v => Number.isSafeInteger(v) && v >= 0;
export function checkVoxelHeader(h) {
  if (h.encoding !== 'vdz1' || !['full', 'delta'].includes(h.mode) || !safe(h.epoch) || !safe(h.seq) ||
    !(h.cell > 0 && h.cell <= .5) || h.origin?.length !== 3 || !h.origin.every(Number.isSafeInteger) ||
    !safe(h.adds) || !safe(h.removes) || h.adds + h.removes > 6e6 || !safe(h.count) || h.count > 3e6 || !safe(h.keyHash) ||
    (h.mode === 'delta' && !safe(h.baseSeq)) || h.rawBytes !== 9 * h.adds + 6 * h.removes)
    throw Error('三维增量格式无效');
}

export class VoxelStore {
  constructor() { this.reset(); }
  reset() {
    this.epoch = this.seq = null; this.cell = 0; this.origin = [0, 0, 0];
    this.index = new Map(); this.capacity = 0; this.count = 0; this.hash = 0;
    this.keys = new Float64Array(0); this.xyz = new Float32Array(0); this.rgb = new Uint8Array(0);
    this.version = 0;   // bumps on every applied message; scene.js syncs incrementally only from version - 1
  }
  get ready() { return this.epoch !== null; }
  ensure(n) {
    if (n <= this.capacity) return;
    const cap = Math.max(n, Math.ceil(this.capacity * 1.5), 4096);
    const keys = new Float64Array(cap), xyz = new Float32Array(cap * 3), rgb = new Uint8Array(cap * 3);
    keys.set(this.keys.subarray(0, this.count)); xyz.set(this.xyz.subarray(0, this.count * 3)); rgb.set(this.rgb.subarray(0, this.count * 3));
    this.keys = keys; this.xyz = xyz; this.rgb = rgb; this.capacity = cap;
  }
  // Applies one message (raw = inflated payload ArrayBuffer, or null when adds+removes is 0).
  // Returns {full, dirty} (dirty = slot indices touched, null after a full). Throws MappingResync when the delta's
  // base is not what this store holds, Error on a malformed message or a failed count/keyHash check; the caller
  // then resets the store and asks the bridge for a full.
  apply(h, raw) {
    checkVoxelHeader(h);
    const full = h.mode === 'full';
    if (!full && (h.epoch !== this.epoch || h.baseSeq !== this.seq)) throw new MappingResync('三维增量与本地基线不符');
    const n = h.adds, m = h.removes, bytes = raw ? raw.byteLength : 0;
    if (bytes !== h.rawBytes) throw Error('三维增量数据不完整');
    if (full) { this.index = new Map(); this.count = 0; this.hash = 0; this.cell = h.cell; this.origin = h.origin.slice(); }
    else if (h.cell !== this.cell || h.origin.some((v, i) => v !== this.origin[i])) throw new MappingResync('三维纪元参数不符');
    const u = bytes ? new Uint16Array(raw, 0, 3 * (n + m)) : new Uint16Array(0), c = bytes ? new Uint8Array(raw, 6 * (n + m), 3 * n) : new Uint8Array(0);
    const dirty = full ? null : [];
    // removes first (a merged delta never names a key in both lists)
    let rx = 0, ry = 0, rz = 0;
    for (let j = 0; j < m; j++) {
      rx = (rx + u[3 * n + j]) & 0xFFFF; ry = (ry + u[3 * n + m + j]) & 0xFFFF; rz = (rz + u[3 * n + 2 * m + j]) & 0xFFFF;
      const k = rx + ry * 65536 + rz * 4294967296, i = this.index.get(k);
      if (i === undefined) continue;                      // removed after being added within the merged span
      const last = this.count - 1;
      if (i !== last) {
        const lk = this.keys[last]; this.keys[i] = lk; this.index.set(lk, i);
        this.xyz.copyWithin(3 * i, 3 * last, 3 * last + 3); this.rgb.copyWithin(3 * i, 3 * last, 3 * last + 3);
        dirty?.push(i);
      }
      this.index.delete(k); this.count = last; this.hash ^= voxelHash(rx, ry, rz);
    }
    this.ensure(this.count + n);
    rx = ry = rz = 0;
    const [ox, oy, oz] = this.origin, s = this.cell;
    for (let j = 0; j < n; j++) {
      rx = (rx + u[j]) & 0xFFFF; ry = (ry + u[n + j]) & 0xFFFF; rz = (rz + u[2 * n + j]) & 0xFFFF;
      const k = rx + ry * 65536 + rz * 4294967296;
      let i = this.index.get(k);
      if (i === undefined) {
        i = this.count++; this.index.set(k, i); this.keys[i] = k; this.hash ^= voxelHash(rx, ry, rz);
        this.xyz[3 * i] = (ox + rx + .5) * s; this.xyz[3 * i + 1] = (oy + ry + .5) * s; this.xyz[3 * i + 2] = (oz + rz + .5) * s;
      }
      this.rgb[3 * i] = c[j]; this.rgb[3 * i + 1] = c[n + j]; this.rgb[3 * i + 2] = c[2 * n + j];
      dirty?.push(i);
    }
    this.epoch = h.epoch; this.seq = h.seq; this.version++;
    if (this.count !== h.count || (this.hash >>> 0) !== h.keyHash) {
      const got = this.count; this.reset();
      throw Error(`三维增量校验失败（本地 ${got} 个体素，应为 ${h.count}）`);
    }
    return {full, dirty};
  }
  // [{key:[rx,ry,rz], rgb:[r,g,b]}] sorted by key -- tests only
  entries() {
    const out = [];
    for (let i = 0; i < this.count; i++) { const k = this.keys[i]; out.push([k, this.rgb[3 * i], this.rgb[3 * i + 1], this.rgb[3 * i + 2]]); }
    return out.sort((a, b) => a[0] - b[0]);
  }
}
