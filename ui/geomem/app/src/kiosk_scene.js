// 展台第三视角（10-17）：照效果稿 render.html 的 style=fv 分支（F：2 cm 真彩体素底图、暖灰渐变背景、
// ACES 色调、PCF 软阴影、白车橙条、视野锥、橙色轨迹管、钉子＋标签、目标光柱）。数值出处见 docs/HANDOVER_KIOSK.md §1。
// 坐标：map 系 (x 前, y 左, z 上，米) → three (x, z, −y)。按需渲染：相机/数据变了才画。
import * as THREE from 'three';
import { OrbitControls } from '../vendor/orbitcontrols.js';
import { buildCarModel, CAR_CAMERA } from './car_model.js';

export const T3 = (x, y, z = 0) => new THREE.Vector3(x, z, -y);
const ACCENT = 0xff7a1a;                   // 浅色档 accent（轨迹、目标环、光柱、视野锥）
export const KIOSK_ZH = { bottle: '水瓶', backpack: '书包', cup: '水杯', 'cell phone': '手机' };
export const OCOL = { bottle: 0x3fa9ff, backpack: 0x7c5cff, cup: 0x2fd38a, 'cell phone': 0xff5c8a };
const OTHER_COLS = [0xf2a33a, 0x14b8a6, 0xe056fd, 0x8d99ae, 0xff8c42, 0x4cc9f0];
export const classColor = c => OCOL[c] ?? OTHER_COLS[[...String(c)].reduce((a, ch) => (a * 31 + ch.charCodeAt(0)) >>> 0, 7) % OTHER_COLS.length];

// 效果稿全景机位（three 系 camera (0.4,5.4,5.6) lookAt (3.0,0,−0.35)）与当时样例库的格心包围盒；
// 别的地图按包围盒等比换算（同方向、同相对位置），样例库上正好还原效果稿机位。
const REF = { min: [-2.67, -3.39], max: [8.03, 2.71], cam: [0.4, -5.6, 5.4], look: [3.0, 0.35, 0] };
export function overviewPose(min, max) {
  const span = Math.max(max[0] - min[0], max[1] - min[1], 1e-3), rspan = Math.max(REF.max[0] - REF.min[0], REF.max[1] - REF.min[1]);
  const rel = i => (REF.look[i] - REF.min[i]) / (REF.max[i] - REF.min[i]);
  const look = [min[0] + rel(0) * (max[0] - min[0]), min[1] + rel(1) * (max[1] - min[1]), 0];
  const k = span / rspan;
  const cam = [look[0] + (REF.cam[0] - REF.look[0]) * k, look[1] + (REF.cam[1] - REF.look[1]) * k, (REF.cam[2] - REF.look[2]) * k];
  return { cam, look };
}

function makeCar() {
  const g = new THREE.Group();
  g.add(buildCarModel({ envless: true }));                     // 实车外观（10-21，照实车照片；car_model.js）
  // 视野锥：俯 7°（10-17 相机改装）、水平半角 35°、垂直半角 27°、长 2.4 m（碰地截断）
  const pitch = THREE.MathUtils.degToRad(CAR_CAMERA.pitchDeg), hf = THREE.MathUtils.degToRad(35), vf = THREE.MathUtils.degToRad(27), L = 2.4;
  const o = new THREE.Vector3(CAR_CAMERA.x, CAR_CAMERA.h, 0), dirs = [];
  for (const [a, b] of [[-hf, -vf], [hf, -vf], [hf, vf], [-hf, vf]]) {
    const v = new THREE.Vector3(Math.cos(b), Math.sin(b), Math.sin(a)).normalize(); v.applyAxisAngle(new THREE.Vector3(0, 0, 1), -pitch); dirs.push(v); }
  const ends = dirs.map(v => { const t = v.y < 0 ? Math.min(L, -CAR_CAMERA.h / v.y) : L; return o.clone().add(v.multiplyScalar(t)); });
  const pos = []; for (let i = 0; i < 4; i++) pos.push(...o.toArray(), ...ends[i].toArray(), ...ends[(i + 1) % 4].toArray());
  const fg = new THREE.BufferGeometry(); fg.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  const fr = new THREE.Group(); fr.add(new THREE.Mesh(fg, new THREE.MeshBasicMaterial({ color: ACCENT, transparent: true, opacity: 0.13, side: THREE.DoubleSide, depthWrite: false })));
  const lp = []; for (const e of ends) lp.push(...o.toArray(), ...e.toArray()); for (let i = 0; i < 4; i++) lp.push(...ends[i].toArray(), ...ends[(i + 1) % 4].toArray());
  const lg = new THREE.BufferGeometry(); lg.setAttribute('position', new THREE.Float32BufferAttribute(lp, 3));
  fr.add(new THREE.LineSegments(lg, new THREE.LineBasicMaterial({ color: ACCENT, transparent: true, opacity: 0.55 })));
  g.add(fr);
  return g;
}

function labelSprite(text, sub, color, big) {
  const c = document.createElement('canvas'), s = 2; c.width = 360 * s; c.height = 120 * s; const g = c.getContext('2d'); g.scale(s, s);
  const font = '"Noto Sans SC","Noto Sans CJK SC","Microsoft YaHei",sans-serif';
  g.font = `700 ${big ? 38 : 30}px ${font}`;
  const w = Math.min(356, Math.max(big ? 300 : 210, g.measureText(text).width + 44)), h = big ? 84 : 62;
  g.fillStyle = 'rgba(255,255,255,0.92)'; g.beginPath(); g.roundRect(0, 0, w, h, 16); g.fill();
  g.fillStyle = '#' + color.toString(16).padStart(6, '0'); g.beginPath(); g.roundRect(0, 0, 10, h, [16, 0, 0, 16]); g.fill();
  g.fillStyle = '#1d2430'; g.fillText(text, 24, big ? 44 : 36);
  g.font = `400 ${big ? 22 : 18}px ${font}`; g.globalAlpha = 0.7; g.fillText(sub, 24, big ? 72 : 55);
  const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace;
  const sp = new THREE.Sprite(new THREE.SpriteMaterial({ map: t, depthTest: false, transparent: true }));
  sp.scale.set(0.9, 0.3, 1); sp.center.set(0, 0); sp.renderOrder = 10; return sp;
}

function disposeTree(o) { o.traverse(x => { x.geometry?.dispose(); if (x.material) { x.material.map?.dispose(); x.material.dispose(); } }); }

export class KioskScene {
  constructor(canvas, { onCamMode } = {}) {
    this.canvas = canvas; this.onCamMode = onCamMode || (() => {});
    const r = this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
    r.setPixelRatio(Math.min(devicePixelRatio || 1, 2));
    r.outputColorSpace = THREE.SRGBColorSpace; r.toneMapping = THREE.ACESFilmicToneMapping; r.toneMappingExposure = 1.0;
    r.shadowMap.enabled = true; r.shadowMap.type = THREE.PCFSoftShadowMap; r.shadowMap.autoUpdate = false; r.shadowMap.needsUpdate = true;
    const scene = this.scene = new THREE.Scene();
    { const c = document.createElement('canvas'); c.width = 2; c.height = 512; const g = c.getContext('2d');
      const gr = g.createLinearGradient(0, 0, 0, 512); gr.addColorStop(0, '#f4efe6'); gr.addColorStop(1, '#d9d2c5');
      g.fillStyle = gr; g.fillRect(0, 0, 2, 512); const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace; scene.background = t; }
    scene.add(new THREE.HemisphereLight(0xffffff, 0x8a8478, 1.5));
    const sun = this.sun = new THREE.DirectionalLight(0xffffff, 1.6); sun.castShadow = true; sun.shadow.mapSize.set(4096, 4096);
    scene.add(sun, sun.target); this.placeSun([-2.67, -3.39], [8.03, 2.71]);
    this.camera = new THREE.PerspectiveCamera(42, 16 / 9, 0.05, 200);
    this.miniCam = new THREE.PerspectiveCamera(50, 360 / 220, 0.05, 200);
    this.controls = new OrbitControls(this.camera, canvas); this.controls.enableDamping = false; this.controls.screenSpacePanning = true;
    this.controls.addEventListener('change', () => { this.dirty = true; });
    this.controls.addEventListener('start', () => { if (this.camMode !== 'free') { this.camMode = 'free'; this.onCamMode('free'); } });
    this.basemap = null; this.basemapInfo = null;
    this.cloud = null; this.cloudPoints = 0;
    this.car = makeCar(); this.car.visible = false; scene.add(this.car);
    this.trail = []; this.trailMesh = null; this.trailBuiltAt = 0; this.trailDirtyN = 0;
    this.pins = new THREE.Group(); scene.add(this.pins); this.pinSig = '';
    this.bbox = { min: [-3, -3], max: [3, 3], source: 'default' };
    this.view = 'third'; this.camMode = 'overview'; this.robot = null; this.dirty = true; this.frameMs = null;
    this.miniRect = null;
    this.resize(); this.overview();
    new ResizeObserver(() => this.resize()).observe(document.body);
    this.loop = () => { this.raf = requestAnimationFrame(this.loop); this.renderIfDirty(); };
    this.raf = requestAnimationFrame(this.loop);
  }
  placeSun(min, max) {
    // 效果稿：sun (−0.5, 9, 4.5) → target (2.5, 0, 0.5)（three 系），阴影相机 ±7、near 1、far 30；别的地图按包围盒中心平移、半宽取 max(7, 半跨度+1)。
    const cx = (min[0] + max[0]) / 2, cy = (min[1] + max[1]) / 2, half = Math.max(7, Math.max(max[0] - min[0], max[1] - min[1]) / 2 + 1);
    const t = T3(cx, cy, 0);
    if (Math.abs(cx - 2.68) < 0.02 && Math.abs(cy + 0.34) < 0.02) t.set(2.5, 0, 0.5);     // 样例库：用效果稿原值
    this.sun.target.position.copy(t); this.sun.position.set(t.x - 3.0, 9, t.z + 4.0);
    Object.assign(this.sun.shadow.camera, { left: -half, right: half, top: half, bottom: -half, near: 1, far: 30 });
    this.sun.shadow.camera.updateProjectionMatrix(); this.renderer.shadowMap.needsUpdate = true; this.dirty = true;
  }
  resize() {
    const w = this.canvas.clientWidth || innerWidth, h = this.canvas.clientHeight || innerHeight;
    this.renderer.setSize(w, h, false); this.camera.aspect = w / h; this.camera.updateProjectionMatrix(); this.dirty = true;
  }
  // ---------------------------------------------------------------- base map / fallback cloud
  setBasemap(manifest, cellsBuf) {
    this.clearBasemap();
    const n = manifest.files.cells.count, S = manifest.cell_m;
    if (cellsBuf.byteLength !== n * 9) throw Error('底图格数据长度与清单不符');
    const idx = new Int16Array(cellsBuf, 0, n * 3), rgb = new Uint8Array(cellsBuf, n * 6, n * 3);
    const lut = new Float32Array(256), col = new THREE.Color();
    for (let i = 0; i < 256; i++) lut[i] = col.setRGB(i / 255, 0, 0, THREE.SRGBColorSpace).r;   // 同 setRGB(…, SRGBColorSpace)
    const im = new THREE.InstancedMesh(new THREE.BoxGeometry(S * 0.92, S * 0.92, S * 0.92), new THREE.MeshStandardMaterial({ roughness: 0.8 }), n);
    const m = im.instanceMatrix.array, c = new Float32Array(n * 3);
    let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
    for (let i = 0; i < n; i++) {
      const x = (idx[3 * i] + 0.5) * S, y = (idx[3 * i + 1] + 0.5) * S, z = (idx[3 * i + 2] + 0.5) * S, o = 16 * i;
      m[o] = 1; m[o + 5] = 1; m[o + 10] = 1; m[o + 15] = 1; m[o + 12] = x; m[o + 13] = z; m[o + 14] = -y;
      c[3 * i] = lut[rgb[3 * i]]; c[3 * i + 1] = lut[rgb[3 * i + 1]]; c[3 * i + 2] = lut[rgb[3 * i + 2]];
      if (x < x0) x0 = x; if (x > x1) x1 = x; if (y < y0) y0 = y; if (y > y1) y1 = y;
    }
    im.instanceMatrix.needsUpdate = true; im.instanceColor = new THREE.InstancedBufferAttribute(c, 3);
    im.computeBoundingSphere(); im.castShadow = true; im.receiveShadow = true;
    this.basemap = im; this.basemapInfo = { sha256: manifest.source_db.sha256, count: n, name: manifest.source_db.name };
    this.scene.add(im);
    if (this.cloud) this.cloud.visible = false;
    this.setBBox([x0, y0], [x1, y1], 'basemap');
    this.renderer.shadowMap.needsUpdate = true; this.dirty = true;
  }
  clearBasemap() {
    if (!this.basemap) return;
    this.scene.remove(this.basemap); this.basemap.geometry.dispose(); this.basemap.material.dispose(); this.basemap.dispose?.();
    this.basemap = null; this.basemapInfo = null; if (this.cloud) this.cloud.visible = true;
    this.renderer.shadowMap.needsUpdate = true; this.dirty = true;
  }
  setCloud(cloud) {   // 桥的栅格点云（无底图时的回退层），{header, positions, colors}
    if (this.cloud) { this.scene.remove(this.cloud); this.cloud.geometry.dispose(); this.cloud.material.dispose(); this.cloud = null; }
    this.cloudPoints = cloud?.header?.count || 0;
    if (this.cloudPoints > 0) {
      const g = new THREE.BufferGeometry();
      g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(cloud.positions), 3));
      g.setAttribute('color', new THREE.BufferAttribute(new Uint8Array(cloud.colors), 3, true));
      g.computeBoundingBox();
      const p = new THREE.Points(g, new THREE.PointsMaterial({ size: 0.035, vertexColors: true, sizeAttenuation: true }));
      p.rotation.x = -Math.PI / 2; p.frustumCulled = false; p.visible = !this.basemap; this.cloud = p; this.scene.add(p);
      if (!this.basemap) { const b = g.boundingBox; this.setBBox([b.min.x, b.min.y], [b.max.x, b.max.y], 'cloud'); }
    }
    this.dirty = true;
  }
  setBBox(min, max, source) {
    const changed = this.bbox.source !== source || Math.abs(this.bbox.min[0] - min[0]) + Math.abs(this.bbox.min[1] - min[1]) + Math.abs(this.bbox.max[0] - max[0]) + Math.abs(this.bbox.max[1] - max[1]) > 0.05;
    this.bbox = { min, max, source };
    if (changed) { this.placeSun(min, max); if (this.camMode === 'overview') this.overview(); }
  }
  // ---------------------------------------------------------------- live layers
  setRobot(robot) {   // {x, y, yaw} map/odom 系或 null（未定位）
    this.robot = robot;
    if (!robot) { if (this.car.visible) { this.car.visible = false; this.renderer.shadowMap.needsUpdate = true; this.dirty = true; } return; }
    const moved = !this.car.visible || Math.hypot(this.car.position.x - robot.x, -this.car.position.z - robot.y) > 1e-4 || Math.abs(this.car.rotation.y - robot.yaw) > 1e-4;
    this.car.visible = true; this.car.position.copy(T3(robot.x, robot.y, 0)); this.car.rotation.y = robot.yaw;
    if (moved) { this.renderer.shadowMap.needsUpdate = true; this.dirty = true; }
    const last = this.trail.at(-1);
    if (!last || Math.hypot(last[0] - robot.x, last[1] - robot.y) > 0.02) { this.trail.push([robot.x, robot.y]); if (this.trail.length > 5000) this.trail.shift(); this.trailDirtyN++; }
    if (this.camMode === 'follow') this.follow();
  }
  resetTrail() { this.trail = []; this.trailDirtyN++; this.rebuildTrail(true); }
  rebuildTrail(force = false) {
    if (!this.trailDirtyN) return;
    const now = performance.now(); if (!force && now - this.trailBuiltAt < 500) return;
    this.trailBuiltAt = now; this.trailDirtyN = 0;
    if (this.trailMesh) { this.scene.remove(this.trailMesh); this.trailMesh.geometry.dispose(); this.trailMesh.material.dispose(); this.trailMesh = null; }
    if (this.trail.length < 2) { this.dirty = true; return; }
    const pts = this.trail.map(p => T3(p[0], p[1], 0.012));
    const curve = new THREE.CatmullRomCurve3(pts, false, 'centripetal');
    this.trailMesh = new THREE.Mesh(new THREE.TubeGeometry(curve, Math.min(1200, Math.max(8, pts.length * 3)), 0.018, 8),
      new THREE.MeshBasicMaterial({ color: ACCENT, transparent: true, opacity: 0.9 }));
    this.scene.add(this.trailMesh); this.dirty = true;
  }
  // pins: [{id, className, position:[x,y,z], name, sub, isTarget, labeled}]
  setPins(pins) {
    const sig = JSON.stringify(pins.map(p => [p.id, p.className, p.position.map(v => +v.toFixed(3)), p.isTarget, p.labeled, p.name, p.sub]));
    if (sig === this.pinSig) return; this.pinSig = sig;
    for (const c of [...this.pins.children]) { this.pins.remove(c); disposeTree(c); }
    for (const p of pins) {
      const col = classColor(p.className), isT = p.isTarget;
      const pin = new THREE.Group(); pin.position.copy(T3(p.position[0], p.position[1], 0)); pin.userData.id = p.id; pin.userData.className = p.className;
      const stem = new THREE.Mesh(new THREE.CylinderGeometry(0.006, 0.006, 0.42, 8), new THREE.MeshBasicMaterial({ color: col, transparent: true, opacity: 0.8 }));
      stem.position.y = 0.21; pin.add(stem);
      const head = new THREE.Mesh(new THREE.SphereGeometry(isT ? 0.05 : 0.035, 24, 16), new THREE.MeshStandardMaterial({ color: col, emissive: col, emissiveIntensity: 0.35 }));
      head.position.y = 0.44; pin.add(head);
      const ring = new THREE.Mesh(new THREE.RingGeometry(isT ? 0.16 : 0.07, isT ? 0.19 : 0.09, 48), new THREE.MeshBasicMaterial({ color: isT ? ACCENT : col, transparent: true, opacity: 0.9, side: THREE.DoubleSide, depthWrite: false }));
      ring.rotation.x = -Math.PI / 2; ring.position.y = 0.006; pin.add(ring);
      if (isT) {
        const r2 = new THREE.Mesh(new THREE.RingGeometry(0.28, 0.30, 64), new THREE.MeshBasicMaterial({ color: ACCENT, transparent: true, opacity: 0.45, side: THREE.DoubleSide, depthWrite: false }));
        r2.rotation.x = -Math.PI / 2; r2.position.y = 0.006; pin.add(r2);
        const beam = new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.12, 1.2, 32, 1, true), new THREE.MeshBasicMaterial({ color: ACCENT, transparent: true, opacity: 0.12, side: THREE.DoubleSide, depthWrite: false }));
        beam.position.y = 0.6; beam.userData.beam = true; pin.add(beam);
      }
      if (p.labeled) { const lb = labelSprite(p.name, p.sub, isT ? ACCENT : col, isT); lb.position.set(0.04, 0.48, 0); pin.add(lb); }
      this.pins.add(pin);
    }
    this.dirty = true;
  }
  // ---------------------------------------------------------------- cameras
  overview() {
    const { cam, look } = overviewPose(this.bbox.min, this.bbox.max);
    this.camera.fov = 42; this.camera.updateProjectionMatrix();
    this.camera.position.copy(T3(...cam)); this.controls.target.copy(T3(...look)); this.camera.lookAt(this.controls.target); this.controls.update();
    this.camMode = 'overview'; this.onCamMode('overview'); this.dirty = true;
  }
  followPose() {
    const r = this.robot; if (!r) return null;
    const car = T3(r.x, r.y, 0), fwd = new THREE.Vector3(Math.cos(r.yaw), 0, -Math.sin(r.yaw));
    const p = car.clone().addScaledVector(fwd, -1.1); p.y = 0.85;
    const t = car.clone().addScaledVector(fwd, 1.5); t.y = 0.05;
    return { p, t };
  }
  follow() {
    const f = this.followPose(); if (!f) return false;
    this.camera.fov = 55; this.camera.updateProjectionMatrix();
    this.camera.position.copy(f.p); this.controls.target.copy(f.t); this.camera.lookAt(f.t); this.controls.update();
    if (this.camMode !== 'follow') { this.camMode = 'follow'; this.onCamMode('follow'); }
    this.dirty = true; return true;
  }
  setView(view, miniEl) { this.view = view; this.miniEl = miniEl; this.controls.enabled = view === 'third'; this.dirty = true; }
  // ---------------------------------------------------------------- render
  renderIfDirty() {
    this.rebuildTrail();
    if (!this.dirty) return;
    this.dirty = false;
    const t0 = performance.now(); this.draw(); this.frameMs = performance.now() - t0;
  }
  draw() {
    const r = this.renderer;
    if (this.view === 'third') { r.setScissorTest(false); r.setViewport(0, 0, this.canvas.width / r.getPixelRatio(), this.canvas.height / r.getPixelRatio()); r.render(this.scene, this.camera); return; }
    // 第一视角：整张画布清成透明，只在小窗里画三维地图（同一个 renderer）
    r.setScissorTest(false); r.setClearColor(0x000000, 0); r.clear();
    const el = this.miniEl; if (!el) return;
    const cr = this.canvas.getBoundingClientRect(), mr = el.getBoundingClientRect();
    const x = mr.left - cr.left, y = cr.bottom - mr.bottom, w = mr.width, h = mr.height;
    const rob = this.robot, c = rob ? T3(rob.x, rob.y, 0) : T3(...overviewPose(this.bbox.min, this.bbox.max).look);
    const fwd = rob ? new THREE.Vector3(Math.cos(rob.yaw), 0, -Math.sin(rob.yaw)) : new THREE.Vector3(1, 0, 0);
    this.miniCam.aspect = w / h; this.miniCam.updateProjectionMatrix();
    this.miniCam.position.copy(c).addScaledVector(fwd, -2.2).setY(3.6); this.miniCam.lookAt(c.clone().addScaledVector(fwd, 0.8));
    r.setScissorTest(true); r.setScissor(x, y, w, h); r.setViewport(x, y, w, h); r.render(this.scene, this.miniCam); r.setScissorTest(false);
  }
  // ?bench=N：同步连画 N 帧（gl.finish ＋ 读 1 像素强制完成），分「只画」与「每帧重算阴影图」两种
  bench(N = 40) {
    const gl = this.renderer.getContext(), px = new Uint8Array(4);
    const sync = () => { gl.finish(); gl.readPixels(0, 0, 1, 1, gl.RGBA, gl.UNSIGNED_BYTE, px); };
    const run = shadow => { this.draw(); sync(); const t0 = performance.now();
      for (let i = 0; i < N; i++) { this.camera.position.x += 0.001; if (shadow) this.renderer.shadowMap.needsUpdate = true; this.draw(); }
      sync(); return (performance.now() - t0) / N; };
    const out = { frames: N, drawMs: run(false), drawWithShadowMs: run(true), instances: this.basemap?.count ?? 0 };
    this.camera.position.x -= 0.002 * N; this.dirty = true; return out;
  }
}
