// 实车外观模型（2026-10-21，照 owner 拍的五张实车照片「小车建模/」搭的程序化模型，不加载外部资源）。
// 两份同字节副本：ui/geomem/app/src/car_model.js（展台）与 ui/geomem/operator/src/car_model.js（比赛控制台），
// 两个页面各自只托管自己的目录；测试 tests/test_geomem_car_model.py 核两份一致。
//
// 坐标：three 系，车头 +x、上 +y、车体左侧 −z（map 系 (x 前, y 左, z 上) → three (x, z, −y)）；原点＝底盘中心在地面上的投影。
// 尺寸（米）：麦轮直径 0.081、左右轮心距 0.205、前后轮心距约 0.20（DECISIONS D-041、10-16 标定）；
// 相机镜头 0.39 m、下倾 7°（10-17 实测 0.3916 m / 7.0°）；其余按照片估，误差约 1 cm。
// 结构：下层板（电池、KV260）、上层板（Orin Nano、底盘控制板、线束）、车头 7 寸屏、车头相机支架（八角板＋舵机＋云台）、Gemini 2。
import * as THREE from 'three';

export const CAR_CAMERA = { x: 0.125, h: 0.39, pitchDeg: 7 };     // 视野锥起点（three 系 x 与离地高度）

const C = {
  plate: 0x16181b, brass: 0xb48a3c, rubber: 0x121417, tip: 0xd9d9d4, hub: 0xc9cdd3, motor: 0xb9bec5,
  battery: 0x1f63d3, pcbGreen: 0x1d5a3c, pcbBlack: 0x1b1d20, fan: 0x0d0e10, glass: 0x050607, frame: 0x1c3058,
  white: 0xf1f1ee, yellow: 0xe8c23a, red: 0xd23b2f, cable: 0x0f1012, silver: 0xbfc4ca,
};

// opts.envless：场景没有环境贴图（展台页）时把金属度压低，否则金属件发黑。opts.scale：整体放大（控制台为了看得清）。
export function buildCarModel(opts = {}) {
  const envless = !!opts.envless, scale = opts.scale || 1;
  const root = new THREE.Group(); root.name = 'car-model';
  const body = new THREE.Group(); root.add(body); body.scale.setScalar(scale);
  const mats = new Map();
  const mat = (color, rough = 0.6, metal = 0, extra = {}) => {
    const k = [color, rough, metal, JSON.stringify(extra)].join('|');
    if (!mats.has(k)) mats.set(k, new THREE.MeshStandardMaterial({ color, roughness: rough, metalness: envless ? metal * 0.45 : metal, ...extra }));
    return mats.get(k);
  };
  // P：车体系 (x 前, l 左, z 上) → three 系
  const P = (x, l, z) => new THREE.Vector3(x, z, -l);
  const add = (geo, m, x, l, z, parent = body) => {
    const mesh = new THREE.Mesh(geo, m); mesh.position.copy(P(x, l, z)); mesh.castShadow = true; mesh.receiveShadow = true;
    parent.add(mesh); return mesh;
  };
  // 长方体：lx 前后、ly 左右、lz 高
  const box = (lx, ly, lz, m, x, l, z, parent) => add(new THREE.BoxGeometry(lx, lz, ly), m, x, l, z, parent);
  // 圆角长方体（xz 截面圆角，沿左右挤出）
  const rbox = (lx, ly, lz, r, m, x, l, z, parent) => {
    const s = new THREE.Shape(), w = lx / 2 - r, h = lz / 2 - r;
    s.moveTo(-w, -lz / 2); s.lineTo(w, -lz / 2); s.quadraticCurveTo(lx / 2, -lz / 2, lx / 2, -h); s.lineTo(lx / 2, h);
    s.quadraticCurveTo(lx / 2, lz / 2, w, lz / 2); s.lineTo(-w, lz / 2); s.quadraticCurveTo(-lx / 2, lz / 2, -lx / 2, h);
    s.lineTo(-lx / 2, -h); s.quadraticCurveTo(-lx / 2, -lz / 2, -w, -lz / 2);
    const b = Math.min(r * 0.6, ly / 4);
    const g = new THREE.ExtrudeGeometry(s, { depth: ly - 2 * b, bevelEnabled: true, bevelThickness: b, bevelSize: b * 0.8, bevelSegments: 2, curveSegments: 4 });
    g.translate(0, 0, -(ly - 2 * b) / 2);
    return add(g, m, x, l, z, parent);
  };
  // 竖直圆柱（沿 z 上）
  const vcyl = (r, z0, z1, m, x, l, seg = 12) => add(new THREE.CylinderGeometry(r, r, z1 - z0, seg), m, x, l, (z0 + z1) / 2);
  // 左右向圆柱（轴沿车体左右）
  const lcyl = (r, len, m, x, l, z, seg = 16, parent) => { const c = add(new THREE.CylinderGeometry(r, r, len, seg), m, x, l, z, parent); c.rotation.x = Math.PI / 2; return c; };
  const tube = (pts, r, m) => {
    const curve = new THREE.CatmullRomCurve3(pts.map(p => P(...p)));
    const t = add(new THREE.TubeGeometry(curve, Math.max(16, pts.length * 10), r, 6, false), m, 0, 0, 0); return t;
  };

  const mPlate = mat(C.plate, 0.55, 0.05), mBrass = mat(C.brass, 0.32, 0.85), mHub = mat(C.hub, 0.22, 0.95);
  const mRubber = mat(C.rubber, 0.85), mTip = mat(C.tip, 0.5), mMotor = mat(C.motor, 0.3, 0.85);
  const mPcbB = mat(C.pcbBlack, 0.55, 0.1), mFan = mat(C.fan, 0.45, 0.2), mCable = mat(C.cable, 0.6);

  // ---- 麦轮 ×4：轮心 x ±0.100、左右 ±0.1025、离地 0.0405
  const R = 0.0405, WW = 0.036, NR = 10;
  const rollerGeo = new THREE.CapsuleGeometry(0.0078, 0.026, 3, 10), tipGeo = new THREE.CylinderGeometry(0.0052, 0.0052, 0.0016, 10);
  const rollers = new THREE.InstancedMesh(rollerGeo, mRubber, 4 * NR), tips = new THREE.InstancedMesh(tipGeo, mTip, 8 * NR);
  rollers.castShadow = tips.castShadow = true; rollers.receiveShadow = true;
  const up = new THREE.Vector3(0, 1, 0), mtx = new THREE.Matrix4(), q = new THREE.Quaternion(), one = new THREE.Vector3(1, 1, 1);
  let ri = 0, ti = 0;
  for (const [wx, wl] of [[0.1, 0.1025], [0.1, -0.1025], [-0.1, 0.1025], [-0.1, -0.1025]]) {
    const hand = Math.sign(wx * wl);                     // 对角两轮辊子同向（X 形布置）
    const outer = Math.sign(wl);                          // 轮毂外侧朝车外
    const c = P(wx, wl, R);
    for (let i = 0; i < NR; i++) {
      const a = (i / NR) * Math.PI * 2;
      const radial = new THREE.Vector3(Math.cos(a), Math.sin(a), 0), tang = new THREE.Vector3(-Math.sin(a), Math.cos(a), 0);
      const axis = new THREE.Vector3(0, 0, 1);            // 车轮转轴：three 的 z（车体左右）
      const dir = tang.clone().multiplyScalar(Math.SQRT1_2).addScaledVector(axis, hand * Math.SQRT1_2).normalize();
      const pos = c.clone().addScaledVector(radial, R - 0.0085);
      q.setFromUnitVectors(up, dir); mtx.compose(pos, q, one); rollers.setMatrixAt(ri++, mtx);
      for (const s of [-1, 1]) { mtx.compose(pos.clone().addScaledVector(dir, s * 0.0205), q, one); tips.setMatrixAt(ti++, mtx); }
    }
    // 轮毂（银色）＋两侧黑色夹板＋中心盖
    lcyl(0.0235, WW * 0.86, mHub, wx, wl, R, 24);
    for (const s of [-1, 1]) lcyl(0.031, 0.0028, mPlate, wx, wl + s * WW * 0.47, R, 12);
    lcyl(0.022, 0.003, mHub, wx, wl + outer * (WW * 0.47 + 0.002), R, 28);        // 外侧银色轮毂面
    lcyl(0.014, 0.004, mat(0x9ea4ab, 0.3, 0.9), wx, wl + outer * (WW * 0.47 + 0.003), R, 20);
    lcyl(0.004, 0.006, mFan, wx, wl + outer * (WW * 0.47 + 0.004), R, 10);
    // 电机（银色圆柱，轴向左右）与电机座
    lcyl(0.0125, 0.06, mMotor, wx, wl - outer * 0.05, R, 18);
    lcyl(0.0128, 0.006, mat(0x2a2c30, 0.6), wx, wl - outer * 0.072, R, 18);
    box(0.03, 0.004, 0.035, mPlate, wx, wl - outer * 0.0175, 0.058);
  }
  body.add(rollers, tips);

  // ---- 下层板（z 0.073）与铜柱
  box(0.27, 0.165, 0.004, mPlate, -0.005, 0, 0.073);
  box(0.3, 0.165, 0.004, mPlate, 0.0, 0, 0.172);                         // 上层板
  for (const x of [-0.13, -0.02, 0.13]) for (const l of [0.075, -0.075]) vcyl(0.0028, 0.075, 0.170, mBrass, x, l, 6);

  // ---- 电池（蓝色热缩膜，车尾下层）
  rbox(0.12, 0.07, 0.058, 0.01, mat(C.battery, 0.32, 0.05), -0.068, 0.012, 0.104);
  box(0.004, 0.04, 0.03, mat(0x2f8f6a, 0.2, 0, { transparent: true, opacity: 0.55 }), -0.129, 0.012, 0.104);
  tube([[-0.12, -0.02, 0.12], [-0.135, -0.04, 0.135], [-0.14, -0.07, 0.11], [-0.125, -0.085, 0.09]], 0.0032, mCable);
  tube([[-0.11, 0.03, 0.134], [-0.1, 0.05, 0.15], [-0.09, 0.07, 0.165]], 0.0016, mat(C.red, 0.5));

  // ---- KV260（绿色载板＋黑色散热风扇罩，标签朝车体右侧）
  box(0.115, 0.13, 0.0022, mat(C.pcbGreen, 0.5, 0.1), 0.058, -0.005, 0.084);
  const kvFan = box(0.07, 0.062, 0.03, mFan, 0.062, -0.035, 0.1005);
  vcyl(0.022, 0.1155, 0.117, mat(0x1c1e22, 0.4, 0.3), 0.062, -0.035, 24);
  {
    const cv = document.createElement('canvas'); cv.width = 256; cv.height = 112; const g = cv.getContext('2d');
    g.fillStyle = '#0d0e10'; g.fillRect(0, 0, 256, 112); g.fillStyle = '#f4f4f4';
    g.font = '700 46px "Arial Black", Arial, sans-serif'; g.fillText('AMD', 18, 50);
    g.font = '700 30px Arial, sans-serif'; g.fillText('KRIA KV260', 18, 92);
    const tex = new THREE.CanvasTexture(cv); tex.colorSpace = THREE.SRGBColorSpace;
    const lab = new THREE.Mesh(new THREE.PlaneGeometry(0.06, 0.026), new THREE.MeshStandardMaterial({ map: tex, roughness: 0.5 }));
    lab.position.copy(P(0.062, -0.0662, 0.1005)); kvFan.parent.add(lab);   // 朝 three +z（车体右侧）
  }
  for (const [x, l, c] of [[0.11, -0.06, C.silver], [0.11, -0.04, C.silver], [0.11, 0.04, C.silver]]) box(0.016, 0.014, 0.012, mat(c, 0.35, 0.7), x, l, 0.092);
  add(new THREE.SphereGeometry(0.0022, 8, 6), mat(0x39ff6a, 0.4, 0, { emissive: 0x39ff6a, emissiveIntensity: 1.2 }), 0.09, 0.03, 0.088);

  // ---- 车头 7 寸屏（竖立朝前）
  box(0.006, 0.162, 0.098, mat(C.frame, 0.45, 0.2), 0.152, 0, 0.122);
  box(0.0025, 0.154, 0.088, mat(C.glass, 0.08, 0.1, { emissive: 0x0a1622, emissiveIntensity: 0.6 }), 0.1555, 0, 0.123);

  // ---- 上层：Orin Nano（车尾偏左）
  box(0.1, 0.08, 0.002, mPcbB, -0.075, 0.03, 0.178);
  box(0.07, 0.048, 0.022, mFan, -0.07, 0.035, 0.19);
  vcyl(0.016, 0.201, 0.2025, mat(0x26282c, 0.35, 0.3), -0.07, 0.035, 20);
  add(new THREE.CylinderGeometry(0.004, 0.004, 0.003, 8), mat(0x50555c, 0.4, 0.5), -0.07, 0.035, 0.2035);
  box(0.03, 0.01, 0.023, mat(0xe8e8e8, 0.5), -0.07, 0.009, 0.191);    // 模组白标签侧
  for (const [l, w] of [[0.06, 0.014], [0.042, 0.014], [0.022, 0.016], [0.002, 0.016]]) box(0.016, w, 0.013, mat(C.silver, 0.3, 0.8), -0.122, l, 0.186);
  // 底盘控制板（车中偏右）＋接插件
  box(0.075, 0.06, 0.002, mPcbB, 0.01, -0.04, 0.178);
  for (const [x, l, c] of [[0.035, -0.065, C.white], [0.03, -0.015, C.white], [-0.01, -0.012, C.yellow], [0.0, -0.068, C.white], [-0.02, -0.05, 0x2fae5a]]) box(0.012, 0.01, 0.008, mat(c, 0.5), x, l, 0.183);
  add(new THREE.SphereGeometry(0.002, 8, 6), mat(0x39ff6a, 0.4, 0, { emissive: 0x39ff6a, emissiveIntensity: 1.4 }), 0.02, -0.04, 0.181);

  // ---- 车头相机支架：铜柱 → 八角板 → 舵机 → 两层圆板 → 云台 → Gemini 2
  const mx = 0.105;
  for (const dx of [-0.03, 0.03]) for (const l of [-0.032, 0.032]) vcyl(0.0026, 0.174, 0.253, mBrass, mx + dx, l, 6);
  box(0.022, 0.04, 0.04, mFan, mx, 0, 0.208);                                   // 舵机
  const oct = vcyl(0.068, 0.253, 0.257, mPlate, mx, 0, 8); oct.rotation.y = Math.PI / 8;
  vcyl(0.01, 0.257, 0.27, mFan, mx, 0, 12);
  vcyl(0.052, 0.27, 0.273, mPlate, mx, 0, 28);
  for (const dx of [-0.028, 0.028]) for (const l of [-0.028, 0.028]) vcyl(0.0024, 0.273, 0.322, mBrass, mx + dx, l, 6);
  box(0.004, 0.03, 0.035, mPlate, mx + 0.012, 0, 0.297);                        // 舵机支架
  vcyl(0.052, 0.322, 0.325, mPlate, mx, 0, 28);
  vcyl(0.016, 0.325, 0.352, mFan, mx, 0, 20);                                    // 云台
  lcyl(0.005, 0.016, mFan, mx + 0.004, 0.022, 0.344, 10);                       // 云台旋钮
  add(new THREE.SphereGeometry(0.011, 16, 12), mat(0x1a1b1e, 0.35, 0.4), mx, 0, 0.359);
  vcyl(0.005, 0.362, 0.378, mFan, mx, 0, 10);
  // Gemini 2：90 × 25 × 30 mm，绕左右轴下倾 7°
  const cam = new THREE.Group(); cam.position.copy(P(mx + 0.006, 0, CAR_CAMERA.h)); cam.rotation.z = -THREE.MathUtils.degToRad(CAR_CAMERA.pitchDeg); body.add(cam);
  rbox(0.026, 0.09, 0.03, 0.009, mat(0x1d1f22, 0.45, 0.15), 0, 0, 0, cam);
  box(0.002, 0.078, 0.017, mat(0x1b2230, 0.08, 0.5), 0.0135, 0, 0.001, cam);
  for (const [l, r] of [[0.03, 0.0045], [-0.03, 0.0045], [0.008, 0.004], [-0.012, 0.0032]]) {
    const ln = add(new THREE.CylinderGeometry(r, r, 0.0016, 18), mat(0x4a505a, 0.15, 0.8), 0.0148, l, 0.001, cam); ln.rotation.z = Math.PI / 2;
    const lc = add(new THREE.CylinderGeometry(r * 0.6, r * 0.6, 0.0018, 18), mat(0x07080a, 0.05, 0.3), 0.0150, l, 0.001, cam); lc.rotation.z = Math.PI / 2;
  }
  add(new THREE.SphereGeometry(0.0016, 8, 6), mat(0xb04dff, 0.3, 0, { emissive: 0xb04dff, emissiveIntensity: 2 }), 0.0152, -0.002, 0.001, cam);
  for (let i = 0; i < 6; i++) box(0.006, 0.07, 0.0016, mat(0x2a2c30, 0.5, 0.4), -0.016, 0, -0.011 + i * 0.0045, cam);   // 背部散热鳍

  // ---- 线束：相机 USB 线、上层几束杂线
  tube([[mx - 0.01, 0.045, 0.39], [mx - 0.03, 0.08, 0.37], [0.03, 0.09, 0.3], [-0.03, 0.075, 0.23], [-0.06, 0.06, 0.205]], 0.0035, mCable);
  tube([[0.07, -0.08, 0.18], [0.03, -0.09, 0.215], [-0.02, -0.07, 0.205], [-0.06, -0.085, 0.18], [-0.11, -0.09, 0.15]], 0.0028, mCable);
  tube([[0.06, 0.08, 0.18], [0.02, 0.095, 0.2], [-0.03, 0.09, 0.19], [-0.09, 0.085, 0.176]], 0.003, mCable);
  tube([[0.0, -0.02, 0.18], [0.02, 0.0, 0.2], [0.05, 0.02, 0.19], [0.07, 0.04, 0.178]], 0.0016, mat(C.red, 0.5));
  tube([[-0.01, -0.06, 0.18], [0.03, -0.03, 0.205], [0.075, -0.01, 0.2], [0.09, 0.02, 0.18]], 0.0014, mat(C.yellow, 0.5));
  tube([[-0.04, -0.05, 0.18], [-0.01, -0.04, 0.195], [0.02, -0.06, 0.19], [0.06, -0.075, 0.178]], 0.0014, mat(0x3a9be8, 0.5));

  root.userData.camera = { ...CAR_CAMERA, x: CAR_CAMERA.x * scale, h: CAR_CAMERA.h * scale };
  root.userData.dispose = () => { root.traverse(o => { o.geometry?.dispose(); if (o.material) { o.material.map?.dispose(); o.material.dispose(); } }); };
  return root;
}
