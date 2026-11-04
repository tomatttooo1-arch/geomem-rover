// Read-only accumulated RTAB-Map cloud; separate from the hub's rolling obstacle map.
function checkMeta(meta,cachedScope,version=1){
  if(meta.format!=='geomem-live-mapping'||meta.version!==version||meta.source!=='rtabmap'||meta.frame!=='map'||meta.error||
    !Number.isSafeInteger(meta.count)||meta.count<1||meta.count>3000000||typeof meta.session!=='string'||!meta.session||typeof meta.scope!=='string'||!meta.scope||
    !Number.isFinite(meta.graphAgeMs)||meta.graphAgeMs<0||!Number.isSafeInteger(meta.graphNodes)||meta.graphNodes<1||!Number.isFinite(meta.cloudAgeMs)||meta.cloudAgeMs<0||
    (cachedScope!==undefined&&cachedScope!==meta.scope)||(meta.cloudAgeMs>15000&&cachedScope!==meta.scope))
    throw Error('RTAB-Map 三维数据未就绪或已过期');
  const t=meta.mapToOdom;
  if(!t||t.position?.length!==3||t.quaternion?.length!==4||![...t.position,...t.quaternion].every(Number.isFinite)||Math.abs(Math.hypot(...t.quaternion)-1)>.001)
    throw Error('地图坐标变换无效');
}

export async function decodeMapping(buffer,{cachedScope}={}) {
  if(buffer.byteLength<4||buffer.byteLength>45010000)throw Error('三维地图大小无效');
  const length=new DataView(buffer).getUint32(0,true);
  if(length<2||length>10000||4+length>buffer.byteLength)throw Error('三维地图清单无效');
  const meta=JSON.parse(new TextDecoder().decode(new Uint8Array(buffer,4,length)));
  checkMeta(meta,cachedScope);
  const payload=buffer.slice(4+length);
  if(payload.byteLength!==meta.count*15)throw Error('三维地图数据不完整');
  const hash=[...new Uint8Array(await crypto.subtle.digest('SHA-256',payload))].map(x=>x.toString(16).padStart(2,'0')).join('');
  if(hash!==meta.sha256)throw Error('三维地图校验失败');
  const positions=new Float32Array(payload,0,meta.count*3),colors=new Uint8Array(payload,meta.count*12);
  if(!positions.every(Number.isFinite))throw Error('三维地图坐标无效');
  return {meta:{...meta,cached:meta.cloudAgeMs>15000},positions,colors};
}

// 10-21 pushed over the bridge WebSocket: header = feed status + {type:'mapping', version:2}; a header-only
// frame is a keep-alive (returns {keepalive:true, meta}); otherwise q5z: zlib(planar uint16 xyz offsets in
// `scale` metres from integer `origin`, then planar RGB).
export async function decodePushedMapping({header,payload},{cachedScope}={}) {
  const meta={...header,version:1};delete meta.type;
  checkMeta(meta,cachedScope);
  if(!payload||!payload.byteLength)return {keepalive:true,meta:{...meta,cached:meta.cloudAgeMs>15000}};
  const n=meta.count;
  if(meta.encoding!=='q5z'||!(meta.scale>0&&meta.scale<=.05)||meta.origin?.length!==3||!meta.origin.every(Number.isSafeInteger)||meta.rawBytes!==n*9)
    throw Error('三维地图压缩格式无效');
  if(typeof DecompressionStream!=='function')throw Error('浏览器不支持解压，请换新版 Edge/Chrome');
  const raw=await new Response(new Blob([payload]).stream().pipeThrough(new DecompressionStream('deflate'))).arrayBuffer();
  if(raw.byteLength!==n*9)throw Error('三维地图数据不完整');
  const q=new Uint16Array(raw,0,n*3),rgb=new Uint8Array(raw,n*6,n*3);   // planar: x[n] y[n] z[n] | r[n] g[n] b[n]
  const positions=new Float32Array(n*3),colors=new Uint8Array(n*3),[ox,oy,oz]=meta.origin,k=meta.scale;
  for(let i=0;i<n;i++){
    positions[3*i]=(q[i]+ox)*k;positions[3*i+1]=(q[n+i]+oy)*k;positions[3*i+2]=(q[2*n+i]+oz)*k;
    colors[3*i]=rgb[i];colors[3*i+1]=rgb[n+i];colors[3*i+2]=rgb[2*n+i];
  }
  return {meta:{...meta,cached:meta.cloudAgeMs>15000},positions,colors};
}

// 10-22 voxel messages from a /delta-capable feed (header.encoding 'vdz1', mode full|delta, or mode 'keep' with
// no payload). Validates the feed status like the q5z path and inflates the payload; voxel-store.js applies it.
export async function inflateVoxelMessage({header,payload},{cachedScope}={}) {
  const meta={...header,version:1};delete meta.type;
  checkMeta(meta,cachedScope);
  meta.cached=meta.cloudAgeMs>15000;
  if(meta.mode==='keep')return {meta,raw:null};
  if(!payload?.byteLength){if(meta.rawBytes===0)return {meta,raw:null};throw Error('三维增量数据缺失');}
  if(typeof DecompressionStream!=='function')throw Error('浏览器不支持解压，请换新版 Edge/Chrome');
  const raw=await new Response(new Blob([payload]).stream().pipeThrough(new DecompressionStream('deflate'))).arrayBuffer();
  return {meta,raw};
}

export function mappingAgeLabel(meta) {
  const seconds=Math.floor(meta.cloudAgeMs/1000);
  return meta.cached?`缓存 · ${seconds<60?`${seconds} 秒`:`${Math.floor(seconds/60)} 分钟`}前更新`:'最近更新';
}

export function transformPoint(p,t) {
  const [x,y,z,w]=t.quaternion,[px,py,pz]=p;
  const tx=2*(y*pz-z*py),ty=2*(z*px-x*pz),tz=2*(x*py-y*px);
  return [px+w*tx+y*tz-z*ty+t.position[0],py+w*ty+z*tx-x*tz+t.position[1],pz+w*tz+x*ty-y*tx+t.position[2]];
}
