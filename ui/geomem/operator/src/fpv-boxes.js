// 10-27（owner）：控制台第一视角叠加与车屏 AR 同源的识别框。
// 数据：预览帧头里的 ar 块 = Nano 上 hub 把几何生产者每帧的 ROI 行（KV260 核过、车屏 AR 核画的那几行，
// 同样 ≤5 个、同类 IoU>0.5 去重）按时间戳配到这一帧（host/hub/fpv_boxes.py），桥补上中文名（gmb_proto.fpv_ar）。
// 坐标是深度图像素（深度已硬件配准到彩色相机，640×480 与预览同一像素系）；配不上的帧 hub 已沿用最近一帧好框。
// 样式照车屏（tools/ar_display/golden.py）：判定 0 白、2 品红、3 灰；框线 2 px，框左上角一条同色标签条、黑字。
export const VERDICT_COLOUR = {0: '#ffffff', 2: '#ff00ff', 3: '#808080'};

// 帧头 ar → {state, boxes, note}；state: ok（画框，可为空）/ stale（框过期，不画）/ none（还没有框数据，不画）
export function readAr(header) {
  const ar = header && typeof header === 'object' ? header.ar : undefined;
  if (!ar || typeof ar !== 'object') return {state: 'none', boxes: [], note: '无识别框数据'};
  const boxes = Array.isArray(ar.boxes) ? ar.boxes.filter(b => b && Array.isArray(b.xyxy) && b.xyxy.length === 4
    && b.xyxy.every(Number.isFinite) && b.xyxy[0] <= b.xyxy[2] && b.xyxy[1] <= b.xyxy[3] && b.v in VERDICT_COLOUR) : [];
  if (ar.state === 'ok') return {state: 'ok', boxes, depthWh: Array.isArray(ar.depthWh) ? ar.depthWh : null, dtMs: ar.dtMs, note: ''};
  if (ar.state === 'stale') return {state: 'stale', boxes: [], note: '识别框过期'};
  return {state: 'none', boxes: [], note: '无识别框数据'};
}

// object-fit: contain 下图片在舞台里的实际矩形（CSS 像素）
export function containRect(stageW, stageH, imgW, imgH) {
  if (!(stageW > 0 && stageH > 0 && imgW > 0 && imgH > 0)) return null;
  const s = Math.min(stageW / imgW, stageH / imgH), w = imgW * s, h = imgH * s;
  return {x: (stageW - w) / 2, y: (stageH - h) / 2, w, h, s};
}

// 深度像素闭区间框 → 舞台坐标。depthWh 缺省 = 预览尺寸（同一 640×480 像素系）
export function mapBox(xyxy, depthWh, imgW, imgH, rect) {
  const [dw, dh] = depthWh && depthWh[0] > 0 && depthWh[1] > 0 ? depthWh : [imgW, imgH];
  const sx = rect.w / dw, sy = rect.h / dh;
  return {x: rect.x + xyxy[0] * sx, y: rect.y + xyxy[1] * sy, w: (xyxy[2] - xyxy[0] + 1) * sx, h: (xyxy[3] - xyxy[1] + 1) * sy};
}

export function boxName(b, objects) {
  const o = Array.isArray(objects) && b.id ? objects.find(x => x && x.id === b.id) : null;
  return (o && typeof o.name === 'string' && o.name) || (typeof b.name === 'string' && b.name) || b.label || '物体';
}

// 10-28（owner）：地图不显示的物体（position_invalid：当面拿走/回来发现不在）第一视角也不画框。
// hub 已按同一规则过滤（host/hub/fpv_boxes.py），这里按控制台当前快照再过一遍，与地图（remote.js legacy）同源。
export function shownBoxes(boxes, objects) {
  if (!Array.isArray(objects) || !objects.length) return boxes;
  const gone = new Set(objects.filter(o => o && o.state === 'position_invalid').map(o => o.id));
  return gone.size ? boxes.filter(b => !(b.id && gone.has(b.id))) : boxes;
}

// 在 canvas 上画（canvas 尺寸跟随舞台，devicePixelRatio 清晰）。请求顺序靠前的在最上层（同车屏：倒序画）。
export function drawBoxes(canvas, img, ar, objects) {
  const stage = canvas.parentElement, dpr = globalThis.devicePixelRatio || 1;
  const W = stage.clientWidth, H = stage.clientHeight;
  if (canvas.width !== Math.round(W * dpr) || canvas.height !== Math.round(H * dpr)) { canvas.width = Math.round(W * dpr); canvas.height = Math.round(H * dpr); }
  const g = canvas.getContext('2d'); g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, W, H);
  const boxes = ar && ar.state === 'ok' ? shownBoxes(ar.boxes, objects) : [];
  if (!boxes.length || img.hidden) return 0;
  const rect = containRect(W, H, img.naturalWidth, img.naturalHeight); if (!rect) return 0;
  const lw = Math.max(2, 2 * rect.w / 640), fs = Math.max(12, Math.round(13 * rect.w / 640));
  g.font = `600 ${fs}px "PingFang SC","Microsoft YaHei","Noto Sans CJK SC",sans-serif`; g.textBaseline = 'middle';
  const list = boxes.map(b => ({b, r: mapBox(b.xyxy, ar.depthWh, img.naturalWidth, img.naturalHeight, rect), c: VERDICT_COLOUR[b.v]})).reverse();
  for (const {r, c} of list) { g.strokeStyle = c; g.lineWidth = lw; g.strokeRect(r.x + lw / 2, r.y + lw / 2, Math.max(0, r.w - lw), Math.max(0, r.h - lw)); }
  for (const {b, r, c} of list) {
    const text = boxName(b, objects), bh = fs + 6, bw = g.measureText(text).width + 10;
    const by = r.y - bh >= rect.y ? r.y - bh : r.y;   // 车屏：上方有空放框上，否则贴框内顶
    g.fillStyle = c; g.fillRect(r.x, by, bw, bh); g.fillStyle = '#000'; g.fillText(text, r.x + 5, by + bh / 2 + 1);
  }
  return list.length;
}
