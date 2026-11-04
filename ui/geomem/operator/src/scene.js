import * as THREE from 'three';
import { OrbitControls } from '../vendor/orbitcontrols.js';
import { buildCarModel } from './car_model.js';
import { beliefCorners, beliefRGBA, toScene, BELIEF_Y } from './belief-layer.js';
let SRGB_LUT=null;
function srgbLut(){if(!SRGB_LUT){SRGB_LUT=new Float32Array(256);const c=new THREE.Color();for(let v=0;v<256;v++){c.setRGB(v/255,v/255,v/255,THREE.SRGBColorSpace);SRGB_LUT[v]=c.r;}}return SRGB_LUT;}
const world = p => new THREE.Vector3(p[0],p[2],-p[1]);
export class SpaceView {
  constructor(canvas, labelRoot, onSelect) {
    this.canvas=canvas;this.labelRoot=labelRoot;this.onSelect=onSelect;this.mode='global';this.source='demo';this.selected=null;this.showObjects=true;this.showGeometry=true;this.showPath=true;
    this.renderer=new THREE.WebGLRenderer({canvas,antialias:true,alpha:true});this.renderer.setPixelRatio(Math.min(devicePixelRatio,2));
    this.renderer.setClearColor(0xe7f1f8,0);this.renderer.outputColorSpace=THREE.SRGBColorSpace;this.renderer.toneMapping=THREE.ACESFilmicToneMapping;this.renderer.toneMappingExposure=.85;this.renderer.shadowMap.enabled=true;this.renderer.shadowMap.type=THREE.PCFSoftShadowMap;
    this.renderer.shadowMap.autoUpdate=false;this.renderer.shadowMap.needsUpdate=true;
    this.needsRender=true;this.reducedMotion=matchMedia('(prefers-reduced-motion: reduce)');this.cameraTween=null;this.robotTween=null;this.scene=new THREE.Scene();this.camera=new THREE.PerspectiveCamera(36,1,.1,150);
    this.controls=new OrbitControls(this.camera,canvas);this.controls.enableDamping=true;this.controls.dampingFactor=.09;this.controls.minDistance=3;this.controls.maxDistance=30;this.controls.maxPolarAngle=Math.PI/2.1;this.controls.enablePan=true;this.controls.addEventListener('change',()=>{this.needsRender=true;});this.controls.addEventListener('start',()=>{this.cameraTween=null;});
    this.scene.add(new THREE.HemisphereLight(0xf6fbff,0x718ba1,.9));
    const sun=new THREE.DirectionalLight(0xfff8ed,2);sun.position.set(-3,8,5);sun.castShadow=true;sun.shadow.mapSize.set(1024,1024);sun.shadow.camera.left=-7;sun.shadow.camera.right=7;sun.shadow.camera.top=7;sun.shadow.camera.bottom=-7;sun.shadow.bias=-.0005;sun.shadow.normalBias=.025;sun.shadow.radius=3;this.scene.add(sun);
    // Local studio reflections give curved surfaces readable highlights without external assets.
    const studio=new THREE.Scene();studio.background=new THREE.Color(0xc8dce9);
    for(const [x,y,z,w,h,intensity] of [[-4,6,2,5,5,4],[5,3,-4,3,6,2],[0,7,0,6,3,3]]){const panel=new THREE.Mesh(new THREE.PlaneGeometry(w,h),new THREE.MeshBasicMaterial({color:new THREE.Color().setScalar(intensity),side:THREE.DoubleSide}));panel.position.set(x,y,z);panel.lookAt(0,0,0);studio.add(panel);}
    const pmrem=new THREE.PMREMGenerator(this.renderer);this.studioEnvironment=pmrem.fromScene(studio,.05);this.scene.environment=this.studioEnvironment.texture;pmrem.dispose();studio.traverse(o=>{o.geometry?.dispose();o.material?.dispose();});
    this.environment=new THREE.Group();this.scene.add(this.environment);this.makeRoom();
    this.cloud=new THREE.Points(new THREE.BufferGeometry(),new THREE.PointsMaterial({size:.022,vertexColors:true,transparent:true,opacity:.16,sizeAttenuation:true}));this.scene.add(this.cloud);
    this.pathLine=new THREE.Line(new THREE.BufferGeometry(),new THREE.LineDashedMaterial({color:0x087da3,dashSize:.13,gapSize:.08}));this.scene.add(this.pathLine);
    this.markers=new THREE.Group();this.scene.add(this.markers);
    this.robot=new THREE.Group();this.scene.add(this.robot);
    {const car=buildCarModel({scale:1.6});car.rotation.y=Math.PI/2;this.robot.add(car);} // 实车外观（10-21，照实车照片；放大 1.6 倍便于全局视角看清；car_model.js）
    const halo=new THREE.Mesh(new THREE.RingGeometry(.44,.47,48),new THREE.MeshBasicMaterial({color:0x66edff,transparent:true,opacity:.7,side:THREE.DoubleSide}));halo.rotation.x=-Math.PI/2;halo.position.y=.014;this.robot.add(halo);
    const arrowShape=new THREE.Shape();arrowShape.moveTo(0,-.78);arrowShape.lineTo(-.1,-.59);arrowShape.lineTo(.1,-.59);arrowShape.closePath();const arrow=new THREE.Mesh(new THREE.ShapeGeometry(arrowShape),new THREE.MeshBasicMaterial({color:0x9afcff,side:THREE.DoubleSide}));arrow.rotation.x=Math.PI/2;arrow.position.y=.03;this.robot.add(arrow);
    this.resizeObserver=new ResizeObserver(()=>this.resize());this.resizeObserver.observe(canvas.parentElement);this.resize();this.reset(false);
    this.lastRender=0;
    this.animate = time => {
      this.frame=requestAnimationFrame(this.animate);
      if(document.hidden || !this.canvas.getClientRects().length || time-this.lastRender<15)return;
      this.lastRender=time;
      this.advanceMotion(time);
      const moving=this.controls.update();
      if(!moving&&!this.needsRender)return;
      this.needsRender=false;this.updateLabels();this.renderer.render(this.scene,this.camera);
    };
    this.frame=requestAnimationFrame(this.animate);
  }
  box(group,w,h,d,x,y,z,color){
    const room=group===this.environment, floor=room&&y<0, wall=room&&h>.6&&(w>4||d>4);
    const bevel=Math.min(.045,w/5,h/5,d/5),shape=new THREE.Shape();
    shape.moveTo(-w/2+bevel,-h/2+bevel);shape.lineTo(w/2-bevel,-h/2+bevel);shape.lineTo(w/2-bevel,h/2-bevel);shape.lineTo(-w/2+bevel,h/2-bevel);shape.closePath();
    const geometry=new THREE.ExtrudeGeometry(shape,{depth:d-2*bevel,bevelEnabled:true,bevelThickness:bevel,bevelSize:bevel,bevelSegments:3,steps:1,curveSegments:1});geometry.translate(0,0,-d/2+bevel);
    const tint=floor?0x9ab9cd:wall?0x8fc6dc:(w>1?0x82aabe:0x577f9a);
    const material=new THREE.MeshPhysicalMaterial({color:room?tint:color,metalness:floor?.15:room?.08:.35,roughness:floor?.28:.22,clearcoat:1,clearcoatRoughness:.1,envMapIntensity:.35,transparent:wall,opacity:wall?.28:1,depthWrite:!wall});
    const mesh=new THREE.Mesh(geometry,material);mesh.position.set(x,y,z);mesh.castShadow=!wall;mesh.receiveShadow=true;group.add(mesh);
    if(wall||floor){const edges=new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(w,h,d)),new THREE.LineBasicMaterial({color:0xf1fcff,transparent:true,opacity:.65}));edges.position.copy(mesh.position);group.add(edges);}
    return mesh;
  }
  makeRoom(){
    const g=this.environment;
    this.box(g,8.3,.12,6.3,0,-.10,0,0xe8ecdf);
    const grid=new THREE.GridHelper(8,32,0x6f9eb7,0x92b2c5);grid.position.y=-.025;grid.scale.z=.75;grid.material.transparent=true;grid.material.opacity=.22;g.add(grid);
    const ground=new THREE.Mesh(new THREE.PlaneGeometry(200,200),new THREE.ShadowMaterial({color:0x527992,opacity:.18}));ground.rotation.x=-Math.PI/2;ground.position.y=-.18;ground.receiveShadow=true;g.add(ground);
    this.box(g,8.2,.65,.13,0,.28,-3.08,0xdde4d2);this.box(g,.13,.65,6.2,-4.08,.28,0,0xd8e1cc);
    // Sofa, desk, coffee table and storage are illustrative demo geometry only.
    this.box(g,1.1,.35,2.65,-3.27,.26,-.55,0xb2bea0);this.box(g,.24,.65,2.65,-3.80,.54,-.55,0xa5b392);
    this.box(g,1.2,.38,.18,-3.27,.51,-1.83,0xafbb9b);this.box(g,1.2,.38,.18,-3.27,.51,.72,0xafbb9b);
    for(const z of [-1.35,-.53,.3])this.box(g,.88,.14,.76,-3.17,.51,z,0xc1cbb0);
    this.box(g,1.9,.12,1.1,2.55,.82,-2.38,0xd7cfb5);for(const x of [1.75,3.35])for(const z of [-2.8,-1.95])this.box(g,.07,.8,.07,x,.38,z,0xaeb89e);
    this.box(g,.7,.45,.07,2.55,1.11,-2.71,0x687f70);this.box(g,.19,.12,.22,2.55,.9,-2.66,0xa4b39a);
    this.box(g,1.8,.1,1.15,-.55,.49,.30,0xd7d7c2);for(const x of [-1.2,.1])for(const z of [-.1,.7])this.box(g,.06,.45,.06,x,.23,z,0xb8c2a7);
    this.box(g,2.7,.018,2.25,-.60,.001,.3,0xe2e6d7);
    this.box(g,1.4,.65,.6,-1.25,.33,-2.65,0xcbd1bb);this.box(g,1.46,.06,.65,-1.25,.68,-2.65,0xb9c5a6);
    for(const x of [-1.7,-1.25,-.8])this.box(g,.03,.57,.02,x,.34,-2.34,0xaab99a);
    const pot=new THREE.Mesh(new THREE.CylinderGeometry(.23,.16,.35,24),new THREE.MeshPhysicalMaterial({color:0xe9f2f7,roughness:.2,metalness:.15,clearcoat:1}));pot.position.set(3.35,.18,1.95);pot.castShadow=true;g.add(pot);
    for(let i=0;i<7;i++){const leaf=new THREE.Mesh(new THREE.SphereGeometry(.23,16,12),new THREE.MeshPhysicalMaterial({color:i%2?0x7da7ad:0x9ac3c6,roughness:.38,clearcoat:.5}));leaf.scale.set(.7,1.6,.7);leaf.position.set(3.35+Math.sin(i*2.4)*.2,.65+i*.035,1.95+Math.cos(i*2.4)*.18);leaf.rotation.z=Math.sin(i)*.5;leaf.castShadow=true;g.add(leaf);}
    this.box(g,.42,.55,.24,2.4,.32,-1.65,0x879876);this.box(g,.13,.07,.2,2.5,.92,-2.55,0x7c8877);
    const cup=new THREE.Mesh(new THREE.CylinderGeometry(.10,.09,.19,20),new THREE.MeshPhysicalMaterial({color:0xa4e0ef,metalness:.3,roughness:.2}));cup.position.copy(world([-.7,-.3,.64]));g.add(cup);
  }
  resize(){this.needsRender=true;const {width,height}=this.canvas.parentElement.getBoundingClientRect();if(!width||!height)return;this.renderer.setSize(width,height,false);this.camera.aspect=width/height;this.camera.updateProjectionMatrix();}
  moveCamera(position,target,smooth=true){
    if(!smooth||this.reducedMotion.matches){this.camera.position.copy(position);this.controls.target.copy(target);this.cameraTween=null;}
    else this.cameraTween={from:this.camera.position.clone(),to:position.clone(),targetFrom:this.controls.target.clone(),targetTo:target.clone(),start:performance.now(),duration:720};
    this.needsRender=true;
  }
  advanceMotion(time){
    if(this.beliefFade){const f=this.beliefFade,p=Math.max(0,Math.min(1,(time-f.start)/f.duration));this.beliefOpacity(1-p);if(p===1){this.beliefFade=null;this.clearBelief();}}
    if(this.cameraTween){const t=this.cameraTween,p=this.reducedMotion.matches?1:Math.max(0,Math.min(1,(time-t.start)/t.duration)),e=1-(1-p)**4;this.camera.position.lerpVectors(t.from,t.to,e);this.controls.target.lerpVectors(t.targetFrom,t.targetTo,e);this.needsRender=true;if(p===1)this.cameraTween=null;}
    if(this.robotTween){this.renderer.shadowMap.needsUpdate=true;const t=this.robotTween,p=this.reducedMotion.matches?1:Math.max(0,Math.min(1,(time-t.start)/t.duration));const old=this.robot.position.clone();this.robot.position.lerpVectors(t.from,t.to,p);this.robot.rotation.y=t.yawFrom+t.yawDelta*p;if(this.mode==='local'&&!this.cameraTween){const delta=this.robot.position.clone().sub(old);this.camera.position.add(delta);this.controls.target.add(delta);}this.needsRender=true;if(p===1)this.robotTween=null;}
  }
  reset(smooth=true){
    if(this.mode==='local'&&this.snapshot?.robot){this.localCamera(smooth);return;}
    if(this.mappingCloud){this.fitMapping(smooth);return;}
    const target=new THREE.Vector3(0,.45,0),direction=new THREE.Vector3(7,7.4,9).normalize();
    const right=new THREE.Vector3().crossVectors(new THREE.Vector3(0,1,0),direction).normalize(),up=new THREE.Vector3().crossVectors(direction,right);
    const tangent=Math.tan(THREE.MathUtils.degToRad(this.camera.fov/2));let distance=0;
    for(const x of [-4.2,4.2])for(const y of [-.2,1.5])for(const z of [-3.2,3.2]){const p=new THREE.Vector3(x,y,z).sub(target);distance=Math.max(distance,p.dot(direction)+Math.abs(p.dot(up))/tangent*1.12,p.dot(direction)+Math.abs(p.dot(right))/(tangent*this.camera.aspect)*1.12);}
    this.moveCamera(target.clone().addScaledVector(direction,distance),target,smooth);
  }
  localCamera(smooth=true){const p=this.robot.position.clone();this.moveCamera(p.clone().add(new THREE.Vector3(3,3.6,4)),p.clone().add(new THREE.Vector3(0,.3,0)),smooth);}
  setMode(mode){this.needsRender=true;this.renderer.shadowMap.needsUpdate=true;this.mode=mode;this.environment.visible=this.showGeometry && this.source==='demo' && mode==='global';this.cloud.material.size=mode==='local'?.045:.022;this.cloud.material.opacity=mode==='local'||this.source==='live'?.95:.16;this.updateCloud();this.updateMappingRegion(true);this.reset();}
  setSource(source){this.needsRender=true;this.renderer.shadowMap.needsUpdate=true;this.source=source;this.cloud.material.opacity=this.mode==='local'||source==='live'?.95:.16;this.environment.visible=this.showGeometry && source==='demo' && this.mode==='global';}
  setLayer(name,value){this.needsRender=true;this.renderer.shadowMap.needsUpdate=true;if(name==='geometry'){this.showGeometry=value;this.cloud.visible=value&&!this.mappingCloud&&!this.basemap;if(this.mappingCloud)this.mappingCloud.visible=value;if(this.basemap)this.basemap.visible=value;this.environment.visible=value&&this.source==='demo'&&this.mode==='global';}if(name==='objects'){this.showObjects=value;this.markers.visible=value;this.updateLabels();}if(name==='path'){this.showPath=value;this.pathLine.visible=value;}}
  updateCloud(){if(!this.snapshot)return;const center=this.snapshot.robot?.position;const pts=this.snapshot.points.filter(p=>this.mode!=='local'||(center&&Math.hypot(p[0]-center[0],p[1]-center[1])<3.2));const coordinates=[],colors=[];for(const p of pts){coordinates.push(p[0],p[2],-p[1]);colors.push(...p.slice(3));}this.cloud.geometry.dispose();this.cloud.geometry=new THREE.BufferGeometry();this.cloud.geometry.setAttribute('position',new THREE.Float32BufferAttribute(coordinates,3));this.cloud.geometry.setAttribute('color',new THREE.Float32BufferAttribute(colors,3));this.cloud.geometry.computeBoundingSphere();}
  setBasemap(manifest, buffer){
    this.clearBasemap();
    const n=manifest.files.cells.count,cell=manifest.cell_m;
    if(!Number.isSafeInteger(n)||n<1||n>3000000||buffer.byteLength!==n*9||cell!==.01)throw Error('底图格数据无效');
    const indices=new Int16Array(buffer,0,n*3),rgb=new Uint8Array(buffer,n*6,n*3);
    const mesh=new THREE.InstancedMesh(new THREE.BoxGeometry(cell*.92,cell*.92,cell*.92),new THREE.MeshStandardMaterial({roughness:.8}),n);
    const matrix=new THREE.Matrix4(),color=new THREE.Color();
    for(let i=0;i<n;i++){
      matrix.makeTranslation((indices[i*3]+.5)*cell,(indices[i*3+2]+.5)*cell,-(indices[i*3+1]+.5)*cell);
      mesh.setMatrixAt(i,matrix);mesh.setColorAt(i,color.setRGB(rgb[i*3]/255,rgb[i*3+1]/255,rgb[i*3+2]/255,THREE.SRGBColorSpace));
    }
    mesh.instanceMatrix.needsUpdate=true;mesh.instanceColor.needsUpdate=true;mesh.computeBoundingSphere();
    mesh.receiveShadow=true;this.basemap=mesh;this.scene.add(mesh);this.cloud.visible=false;
    const center=mesh.boundingSphere.center.clone(),r=Math.max(2,mesh.boundingSphere.radius);
    this.moveCamera(center.clone().add(new THREE.Vector3(r*.65,r*1.5,r*1.8)),center);
    this.needsRender=true;this.renderer.shadowMap.needsUpdate=true;
  }
  clearBasemap(){
    if(!this.basemap)return;
    this.scene.remove(this.basemap);this.basemap.geometry.dispose();this.basemap.material.dispose();this.basemap.dispose?.();
    this.basemap=null;this.cloud.visible=this.showGeometry&&!this.mappingCloud;this.needsRender=true;
  }
  setMappingCloud({positions,colors,meta}){
    if(this.mappingCloud&&this.mappingMeta?.sha256===meta.sha256&&this.mappingMeta.session===meta.session&&this.mappingMeta.scope===meta.scope){this.mappingMeta=meta;return;}
    const first=!this.mappingCloud;
    this.clearMappingCloud();
    const xyz=new Float32Array(positions.length),rgb=new Float32Array(colors.length),color=new THREE.Color();
    for(let i=0;i<meta.count;i++){
      xyz[i*3]=positions[i*3];xyz[i*3+1]=positions[i*3+2];xyz[i*3+2]=-positions[i*3+1];
      color.setRGB(colors[i*3]/255,colors[i*3+1]/255,colors[i*3+2]/255,THREE.SRGBColorSpace);
      rgb[i*3]=color.r;rgb[i*3+1]=color.g;rgb[i*3+2]=color.b;
    }
    const geometry=new THREE.BufferGeometry();
    geometry.setAttribute('position',new THREE.BufferAttribute(xyz,3));geometry.setAttribute('color',new THREE.BufferAttribute(rgb,3));geometry.computeBoundingSphere();
    this.mappingCloud=new THREE.Points(geometry,new THREE.PointsMaterial({size:.018,vertexColors:true,toneMapped:false}));
    this.mappingCloud.visible=this.showGeometry;
    this.mappingMeta=meta;this.mappingPositions=positions;
    this.scene.add(this.mappingCloud);this.cloud.visible=false;
    this.updateMappingRegion(true);
    if(first){if(this.mode==='local'&&this.snapshot?.robot)this.localCamera();else this.fitMapping();}this.needsRender=true;
  }
  // 10-22 voxel deltas: one geometry per store capacity, refilled only at the dirty slots (voxel-store.js);
  // a scene that missed a version (or a new capacity / full) copies all `count` slots once.
  setMappingStore({store,meta,dirty}){
    const n=store.count,first=!this.mappingCloud;
    if(!this.mappingStoreMode||this.mappingStoreCap!==store.capacity){
      this.clearMappingCloud();
      const geometry=new THREE.BufferGeometry();
      geometry.setAttribute('position',new THREE.BufferAttribute(new Float32Array(store.capacity*3),3));
      geometry.setAttribute('color',new THREE.BufferAttribute(new Float32Array(store.capacity*3),3));
      this.mappingCloud=new THREE.Points(geometry,new THREE.PointsMaterial({size:Math.max(.018,store.cell),vertexColors:true,toneMapped:false}));
      this.mappingCloud.frustumCulled=false;this.mappingCloud.visible=this.showGeometry;
      this.scene.add(this.mappingCloud);this.cloud.visible=false;
      this.mappingStoreMode=true;this.mappingStoreCap=store.capacity;this.mappingStoreVersion=-1;
    }
    if(this.mappingStoreVersion===store.version){this.mappingMeta={...meta,count:n};return;}
    const g=this.mappingCloud.geometry,pos=g.attributes.position,col=g.attributes.color,P=pos.array,C=col.array,X=store.xyz,R=store.rgb,lut=srgbLut();
    const put=i=>{P[3*i]=X[3*i];P[3*i+1]=X[3*i+2];P[3*i+2]=-X[3*i+1];C[3*i]=lut[R[3*i]];C[3*i+1]=lut[R[3*i+1]];C[3*i+2]=lut[R[3*i+2]];};
    const whole=!dirty||this.mappingStoreVersion!==store.version-1;
    if(whole)for(let i=0;i<n;i++)put(i);else for(const i of dirty)if(i<n)put(i);
    pos.needsUpdate=col.needsUpdate=true;g.setDrawRange(0,n);
    this.mappingStoreVersion=store.version;
    this.mappingMeta={...meta,count:n};this.mappingPositions=X;
    if(whole||!g.boundingSphere){                     // fitMapping only; culling is off (the store grows)
      let lo=[Infinity,Infinity,Infinity],hi=[-Infinity,-Infinity,-Infinity];
      for(let i=0;i<n;i++)for(let a=0;a<3;a++){const v=P[3*i+a];if(v<lo[a])lo[a]=v;if(v>hi[a])hi[a]=v;}
      if(n){const c=new THREE.Vector3((lo[0]+hi[0])/2,(lo[1]+hi[1])/2,(lo[2]+hi[2])/2);g.boundingSphere=new THREE.Sphere(c,Math.hypot(hi[0]-lo[0],hi[1]-lo[1],hi[2]-lo[2])/2);}
    }
    this.updateMappingRegion(true);
    if(first){if(this.mode==='local'&&this.snapshot?.robot)this.localCamera();else this.fitMapping();}this.needsRender=true;
  }
  updateMappingRegion(force=false){
    if(!this.mappingCloud)return;
    const center=this.snapshot?.robot?.position;
    const key=this.mode==='local'?center?.join(',')||'no-pose':'global';
    if(!force&&key===this.mappingRegionKey)return;
    this.mappingRegionKey=key;
    if(this.mode==='global'){this.mappingCloud.geometry.setIndex(null);this.mappingVisibleCount=this.mappingMeta.count;}
    else{
      const indices=[],p=this.mappingPositions;
      if(center)for(let i=0;i<this.mappingMeta.count;i++){const dx=p[i*3]-center[0],dy=p[i*3+1]-center[1];if(dx*dx+dy*dy<3.2**2)indices.push(i);}
      this.mappingCloud.geometry.setIndex(indices);this.mappingVisibleCount=indices.length;
    }
    this.needsRender=true;
  }
  fitMapping(smooth=true){
    const b=this.mappingCloud?.geometry.boundingSphere;if(!b)return;
    const r=Math.max(1.5,b.radius),target=b.center.clone();
    this.moveCamera(target.clone().add(new THREE.Vector3(r*.65,r*.9,r*2.3)),target,smooth);
  }
  clearMappingCloud(){
    this.mappingStoreMode=false;this.mappingStoreCap=0;this.mappingStoreVersion=-1;
    this.mappingMeta=null;this.mappingPositions=null;this.mappingVisibleCount=0;this.mappingRegionKey=null;
    if(!this.mappingCloud)return;
    this.scene.remove(this.mappingCloud);this.mappingCloud.geometry.dispose();this.mappingCloud.material.dispose();this.mappingCloud=null;
    this.cloud.visible=this.showGeometry&&!this.basemap;this.needsRender=true;
  }
  update(snapshot){
    this.needsRender=true;
    this.renderer.shadowMap.needsUpdate=true;
    const oldPosition=this.snapshot?.robot?.position;this.snapshot=snapshot;this.markers.visible=this.showObjects;
    this.updateMappingRegion();
    this.robot.visible=!!snapshot.robot;
    if(snapshot.robot){
      const target=world(snapshot.robot.position),yaw=snapshot.robot.yaw-Math.PI/2;
      if(!oldPosition||this.reducedMotion.matches){this.robot.position.copy(target);this.robot.rotation.y=yaw;this.robotTween=null;}
      else {const delta=Math.atan2(Math.sin(yaw-this.robot.rotation.y),Math.cos(yaw-this.robot.rotation.y));if(target.distanceToSquared(this.robot.position)>.000001||Math.abs(delta)>.0001)this.robotTween={from:this.robot.position.clone(),to:target,yawFrom:this.robot.rotation.y,yawDelta:delta,start:performance.now(),duration:this.source==='demo'?950:200};}
    }
    this.updateCloud();this.pathLine.geometry.dispose();this.pathLine.geometry=new THREE.BufferGeometry().setFromPoints(snapshot.path.map(p=>world(p).add(new THREE.Vector3(0,.035,0))));this.pathLine.computeLineDistances();
    const key=JSON.stringify(snapshot.objects.map(o=>[o.id,o.name,o.position]));
    if(key!==this.objectsKey){this.objectsKey=key;for(const c of [...this.markers.children]){c.geometry.dispose();c.material.dispose();this.markers.remove(c);}this.labelRoot.replaceChildren();this.labels=[];
      for(const o of snapshot.objects){const m=new THREE.Mesh(new THREE.RingGeometry(.15,.19,32),new THREE.MeshBasicMaterial({color:0x7eebff,side:THREE.DoubleSide}));m.rotation.x=-Math.PI/2;m.position.copy(world(o.position)).add(new THREE.Vector3(0,.06,0));this.markers.add(m);const b=document.createElement('button');b.className='object-label';b.textContent=o.name;b.setAttribute('aria-label',`查看${o.name} ${o.id}`);b.onclick=()=>this.onSelect(o.id);this.labelRoot.append(b);this.labels.push({el:b,object:o,marker:m});}
    }
    this.updateLabels();
  }
  // 10-26 搜索热度图：地面上一层半透明格（belief-layer.js 定坐标与颜色）＋下一个看点（红圈加箭头）＋已看点（白点）
  setBelief(b){
    this.beliefFade=null;
    if(!b.width||!b.height){this.beliefPhase=b.phase;this.beliefOpacity(1);return;}
    const rgba=beliefRGBA(b);
    if(!this.beliefMesh||this.beliefTexture.image.width!==b.width||this.beliefTexture.image.height!==b.height){
      this.clearBelief();
      this.beliefTexture=new THREE.DataTexture(rgba,b.width,b.height,THREE.RGBAFormat);
      this.beliefTexture.magFilter=this.beliefTexture.minFilter=THREE.NearestFilter;this.beliefTexture.colorSpace=THREE.SRGBColorSpace;
      const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.Float32BufferAttribute(new Float32Array(12),3));
      g.setAttribute('uv',new THREE.Float32BufferAttribute([0,0,1,0,1,1,0,1],2));g.setIndex([0,1,2,0,2,3]);
      this.beliefMesh=new THREE.Mesh(g,new THREE.MeshBasicMaterial({map:this.beliefTexture,transparent:true,depthWrite:false,side:THREE.DoubleSide,toneMapped:false}));
      this.beliefMesh.renderOrder=5;this.scene.add(this.beliefMesh);
      this.beliefMarks=new THREE.Group();this.beliefMarks.renderOrder=6;this.scene.add(this.beliefMarks);
    }else{this.beliefTexture.image.data.set(rgba);}
    this.beliefTexture.needsUpdate=true;
    const pos=this.beliefMesh.geometry.attributes.position;
    beliefCorners(b).forEach((c,i)=>pos.setXYZ(i,...toScene(c)));pos.needsUpdate=true;this.beliefMesh.geometry.computeBoundingSphere();
    for(const c of [...this.beliefMarks.children]){c.traverse(o=>{o.geometry?.dispose();o.material?.dispose();});this.beliefMarks.remove(c);}
    for(const p of b.seen){const m=new THREE.Mesh(new THREE.CircleGeometry(.05,16),new THREE.MeshBasicMaterial({color:0xffffff,transparent:true,opacity:.85,depthWrite:false}));m.rotation.x=-Math.PI/2;m.position.set(...toScene(p,BELIEF_Y+.01));this.beliefMarks.add(m);}
    if(b.next){
      const mark=new THREE.Group(),mat=()=>new THREE.MeshBasicMaterial({color:0xff2d2d,transparent:true,opacity:1,side:THREE.DoubleSide,depthWrite:false});
      const ring=new THREE.Mesh(new THREE.RingGeometry(.15,.21,40),mat());ring.rotation.x=-Math.PI/2;mark.add(ring);
      const tip=new THREE.Shape();tip.moveTo(.42,0);tip.lineTo(.24,.09);tip.lineTo(.24,-.09);tip.closePath();
      const arrow=new THREE.Mesh(new THREE.ShapeGeometry(tip),mat());arrow.rotation.x=-Math.PI/2;mark.add(arrow);
      mark.position.set(...toScene(b.next,BELIEF_Y+.02));mark.rotation.y=b.next[2];this.beliefMarks.add(mark);
    }
    this.beliefPhase=b.phase;this.beliefOpacity(1);
  }
  beliefOpacity(k){
    if(!this.beliefMesh)return;
    this.beliefMesh.material.opacity=k;
    this.beliefMarks.traverse(o=>{if(o.material){o.material.userData.base??=o.material.opacity;o.material.opacity=o.material.userData.base*k;}});
    this.needsRender=true;
  }
  fadeBelief(duration){if(!this.beliefMesh)return;this.beliefFade={start:performance.now(),duration:this.reducedMotion.matches?1:duration};this.needsRender=true;}
  clearBelief(){
    this.beliefFade=null;this.beliefPhase=null;
    if(!this.beliefMesh)return;
    this.scene.remove(this.beliefMesh);this.beliefMesh.geometry.dispose();this.beliefMesh.material.dispose();this.beliefTexture.dispose();
    this.beliefMarks.traverse(o=>{o.geometry?.dispose();o.material?.dispose();});this.scene.remove(this.beliefMarks);
    this.beliefMesh=this.beliefMarks=this.beliefTexture=null;this.needsRender=true;
  }
  clear(){this.clearBelief();this.needsRender=true;this.renderer.shadowMap.needsUpdate=true;this.robotTween=null;this.cameraTween=null;this.snapshot=null;this.objectsKey='';this.labelRoot.replaceChildren();this.labels=[];this.robot.visible=false;this.cloud.geometry.dispose();this.cloud.geometry=new THREE.BufferGeometry();this.pathLine.geometry.dispose();this.pathLine.geometry=new THREE.BufferGeometry();this.markers.visible=false;this.environment.visible=false;}
  updateLabels(){
    const w=this.canvas.clientWidth,h=this.canvas.clientHeight,occupied=[];
    const labels=[...(this.labels||[])].sort((a,b)=>Number(b.object.id===this.selected)-Number(a.object.id===this.selected));
    for(const {el,object:o,marker} of labels){
      const v=world(o.position).add(new THREE.Vector3(0,.4,0)).project(this.camera);
      const near=this.mode!=='local'||(this.snapshot?.robot&&Math.hypot(o.position[0]-this.snapshot.robot.position[0],o.position[1]-this.snapshot.robot.position[1])<3.2);
      marker.visible=this.showObjects&&near;
      el.hidden=!this.showObjects||!near||v.z>1||v.z< -1;
      el.classList.toggle('selected',o.id===this.selected);
      let x=(v.x*.5+.5)*w,y=(-v.y*.5+.5)*h;
      if(this.declutterLabels&&!el.hidden){
        const width=el.offsetWidth,height=el.offsetHeight;
        const box={left:x-width/2,top:y-height,right:x+width/2,bottom:y};
        el.hidden=box.left<4||box.right>w-4||box.top<48||box.bottom>h-38||occupied.some(r=>box.left<r.right+8&&box.right>r.left-8&&box.top<r.bottom+8&&box.bottom>r.top-8);
        if(!el.hidden)occupied.push(box);
      }
      el.style.left=`${x}px`;el.style.top=`${y}px`;
    }
    const right=new THREE.Vector3().setFromMatrixColumn(this.camera.matrixWorld,0);
    const a=this.controls.target.clone().project(this.camera),b=this.controls.target.clone().add(right).project(this.camera);
    const scale=document.getElementById('scale-line');if(scale)scale.style.width=`${Math.abs(b.x-a.x)*w/2}px`;
  }
  focus(id){const o=this.snapshot?.objects.find(o=>o.id===id);if(!o)return;this.selected=id;const target=world(o.position);this.moveCamera(this.camera.position.clone().add(target.clone().sub(this.controls.target)),target);}
  zoom(factor){const offset=this.camera.position.clone().sub(this.controls.target);offset.setLength(THREE.MathUtils.clamp(offset.length()*factor,this.controls.minDistance,this.controls.maxDistance));this.moveCamera(offset.add(this.controls.target),this.controls.target);}
}
