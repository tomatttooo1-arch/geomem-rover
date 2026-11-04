import * as THREE from 'three';
import { OrbitControls } from '../vendor/orbitcontrols.js';
import { buildCarModel } from './car_model.js';
const world = p => new THREE.Vector3(p[0],p[2],-p[1]);
export class SpaceView {
  constructor(canvas, labelRoot, onSelect, nameOf = o => o.name || o.className) {
    this.nameOf=nameOf;
    this.canvas=canvas;this.labelRoot=labelRoot;this.onSelect=onSelect;this.mode='global';this.source='demo';this.selected=null;this.showObjects=true;this.showGeometry=true;this.showPath=true;
    this.renderer=new THREE.WebGLRenderer({canvas,antialias:true,alpha:true});this.renderer.setPixelRatio(Math.min(devicePixelRatio,2));
    this.renderer.setClearColor(0xe7f1f8,0);this.renderer.outputColorSpace=THREE.SRGBColorSpace;this.renderer.toneMapping=THREE.ACESFilmicToneMapping;this.renderer.toneMappingExposure=.85;this.renderer.shadowMap.enabled=true;this.renderer.shadowMap.type=THREE.PCFSoftShadowMap;
    this.renderer.shadowMap.autoUpdate=false;this.renderer.shadowMap.needsUpdate=true;
    this.needsRender=true;this.reducedMotion=matchMedia('(prefers-reduced-motion: reduce)');this.cameraTween=null;this.robotTween=null;this.scene=new THREE.Scene();this.camera=new THREE.PerspectiveCamera(36,1,.1,150);
    this.controls=new OrbitControls(this.camera,canvas);this.controls.enableDamping=true;this.controls.dampingFactor=.09;this.controls.minDistance=3;this.controls.maxDistance=30;this.controls.maxPolarAngle=Math.PI/2.1;this.controls.enablePan=true;this.controls.addEventListener('change',()=>{this.needsRender=true;});this.controls.addEventListener('start',()=>{this.cameraTween=null;});
    this.scene.add(new THREE.HemisphereLight(0xf6fbff,0x718ba1,.9));
    const sun=this.sun=new THREE.DirectionalLight(0xfff8ed,2);sun.position.set(-3,8,5);this.scene.add(sun.target);sun.castShadow=true;sun.shadow.mapSize.set(1024,1024);sun.shadow.camera.left=-7;sun.shadow.camera.right=7;sun.shadow.camera.top=7;sun.shadow.camera.bottom=-7;sun.shadow.bias=-.0005;sun.shadow.normalBias=.025;sun.shadow.radius=3;this.scene.add(sun);
    // Local studio reflections give curved surfaces readable highlights without external assets.
    const studio=new THREE.Scene();studio.background=new THREE.Color(0xc8dce9);
    for(const [x,y,z,w,h,intensity] of [[-4,6,2,5,5,4],[5,3,-4,3,6,2],[0,7,0,6,3,3]]){const panel=new THREE.Mesh(new THREE.PlaneGeometry(w,h),new THREE.MeshBasicMaterial({color:new THREE.Color().setScalar(intensity),side:THREE.DoubleSide}));panel.position.set(x,y,z);panel.lookAt(0,0,0);studio.add(panel);}
    const pmrem=new THREE.PMREMGenerator(this.renderer);this.studioEnvironment=pmrem.fromScene(studio,.05);this.scene.environment=this.studioEnvironment.texture;pmrem.dispose();studio.traverse(o=>{o.geometry?.dispose();o.material?.dispose();});
    this.environment=new THREE.Group();this.scene.add(this.environment);this.makeRoom();
    // 点云顶点保持 map/odom 系原始坐标（x,y,z），由对象自身旋转 −90° 绕 X 轴得到显示坐标 (x,z,−y)：类型数组零拷贝直接上 GPU。
    this.cloudUniforms={uCenter:{value:new THREE.Vector2()},uRadius:{value:0}};
    const cloudMaterial=new THREE.PointsMaterial({size:.022,vertexColors:true,transparent:true,opacity:.16,sizeAttenuation:true});
    cloudMaterial.onBeforeCompile=shader=>{Object.assign(shader.uniforms,this.cloudUniforms);shader.vertexShader='uniform vec2 uCenter;\nuniform float uRadius;\n'+shader.vertexShader.replace('#include <project_vertex>','#include <project_vertex>\n  if(uRadius>0.0&&distance(position.xy,uCenter)>uRadius)gl_Position=vec4(2.0,2.0,2.0,1.0);');};
    this.cloud=new THREE.Points(new THREE.BufferGeometry(),cloudMaterial);this.cloud.rotation.x=-Math.PI/2;this.cloud.frustumCulled=false;this.scene.add(this.cloud);this.cloudBox=null;
    this.highlights=new THREE.Group();this.scene.add(this.highlights);
    this.pathLine=new THREE.Line(new THREE.BufferGeometry(),new THREE.LineDashedMaterial({color:0x087da3,dashSize:.13,gapSize:.08}));this.scene.add(this.pathLine);
    this.markers=new THREE.Group();this.scene.add(this.markers);
    this.robot=new THREE.Group();this.scene.add(this.robot);
    {const car=buildCarModel({scale:1.6});car.rotation.y=Math.PI/2;this.robot.add(car);} // 实车外观（10-21，照实车照片；放大 1.6 倍便于全局视角看清；car_model.js）
    const halo=this.halo=new THREE.Mesh(new THREE.RingGeometry(.44,.47,48),new THREE.MeshBasicMaterial({color:0x66edff,transparent:true,opacity:.7,side:THREE.DoubleSide}));halo.rotation.x=-Math.PI/2;halo.position.y=.014;this.robot.add(halo);
    // 里程推算档：虚线光环（12 段弧）。
    this.dashedHalo=new THREE.Group();for(let i=0;i<12;i++){const arc=new THREE.Mesh(new THREE.RingGeometry(.44,.48,8,1,i*Math.PI/6,Math.PI/10),new THREE.MeshBasicMaterial({color:0xf2b84b,transparent:true,opacity:.95,side:THREE.DoubleSide}));arc.rotation.x=-Math.PI/2;arc.position.y=.015;this.dashedHalo.add(arc);}this.dashedHalo.visible=false;this.robot.add(this.dashedHalo);
    const arrowShape=new THREE.Shape();arrowShape.moveTo(0,-.78);arrowShape.lineTo(-.1,-.59);arrowShape.lineTo(.1,-.59);arrowShape.closePath();const arrow=new THREE.Mesh(new THREE.ShapeGeometry(arrowShape),new THREE.MeshBasicMaterial({color:0x9afcff,side:THREE.DoubleSide}));arrow.rotation.x=Math.PI/2;arrow.position.y=.03;this.robot.add(arrow);
    this.robot.traverse(o=>{if(o.material){o.material.userData.baseColor=o.material.color.clone();o.material.userData.baseOpacity=o.material.opacity;o.material.userData.baseTransparent=o.material.transparent;}});this.robotStyle='';this.userMoved=false;this.controls.addEventListener('start',()=>{this.userMoved=true;});
    this.resizeObserver=new ResizeObserver(()=>this.resize());this.resizeObserver.observe(canvas.parentElement);this.resize();this.reset(false);
    this.lastRender=0;
    this.animate = time => {
      this.frame=requestAnimationFrame(this.animate);
      if(document.hidden || !this.canvas.getClientRects().length || time-this.lastRender<15)return;
      this.lastRender=time;
      this.advanceMotion(time);if(this.mode==='local'){this.cloudUniforms.uCenter.value.set(this.robot.position.x,-this.robot.position.z);}
      const moving=this.controls.update();
      if(!moving&&!this.needsRender)return;
      this.needsRender=false;this.updateLabels();this.renderer.render(this.scene,this.camera);
    };
    // 预编译着色器（含点云的改写版），避免第一份点云到达时现场编译造成上屏延迟。
    try{this.renderer.compile(this.scene,this.camera);}catch{}
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
    if(this.cameraTween){const t=this.cameraTween,p=this.reducedMotion.matches?1:Math.max(0,Math.min(1,(time-t.start)/t.duration)),e=1-(1-p)**4;this.camera.position.lerpVectors(t.from,t.to,e);this.controls.target.lerpVectors(t.targetFrom,t.targetTo,e);this.needsRender=true;if(p===1)this.cameraTween=null;}
    if(this.robotTween){this.renderer.shadowMap.needsUpdate=true;const t=this.robotTween,p=this.reducedMotion.matches?1:Math.max(0,Math.min(1,(time-t.start)/t.duration));const old=this.robot.position.clone();this.robot.position.lerpVectors(t.from,t.to,p);this.robot.rotation.y=t.yawFrom+t.yawDelta*p;if(this.mode==='local'&&!this.cameraTween){const delta=this.robot.position.clone().sub(old);this.camera.position.add(delta);this.controls.target.add(delta);}this.needsRender=true;if(p===1)this.robotTween=null;}
  }
  // 视角按真实范围自动框：点云＋物体＋小车＋路径的包围盒（演示另并上演示房间）。
  dataBox(){
    const box=new THREE.Box3();
    if(this.cloudBox&&this.showGeometry&&this.cloud.visible)box.union(this.cloudBox);
    const s=this.snapshot;
    if(s){for(const o of s.objects)box.expandByPoint(world(o.position));if(s.robot)box.expandByPoint(world(s.robot.position));for(const p of s.path)box.expandByPoint(world(p));for(const c of s.task?.candidates||[])box.expandByPoint(world(c.position));}
    if(this.source==='demo')box.union(new THREE.Box3(new THREE.Vector3(-4.2,-.2,-3.2),new THREE.Vector3(4.2,1.5,3.2)));
    if(box.isEmpty())box.set(new THREE.Vector3(-4,-.2,-3),new THREE.Vector3(4,1.5,3));
    box.min.y=Math.min(box.min.y,-.2);box.max.y=Math.max(box.max.y,1.2);
    const size=box.getSize(new THREE.Vector3());const pad=new THREE.Vector3(Math.max(0,1.5-size.x)/2,0,Math.max(0,1.5-size.z)/2);box.min.sub(pad);box.max.add(pad);
    return box;
  }
  fitShadow(box){
    const c=box.getCenter(new THREE.Vector3()),r=Math.max(4,box.getSize(new THREE.Vector3()).length()/2+.5),cam=this.sun.shadow.camera;
    this.sun.target.position.copy(c);this.sun.position.copy(c).add(new THREE.Vector3(-3,8,5).setLength(r*1.5));
    cam.left=-r;cam.right=r;cam.top=r;cam.bottom=-r;cam.near=.1;cam.far=r*4;cam.updateProjectionMatrix();this.renderer.shadowMap.needsUpdate=true;
  }
  reset(smooth=true){
    this.userMoved=false;
    if(this.mode==='local'&&this.snapshot?.robot){this.localCamera(smooth);return;}
    const box=this.dataBox();this.fitShadow(box);
    const target=box.getCenter(new THREE.Vector3());target.y=Math.max(.2,Math.min(target.y,.6));
    const direction=new THREE.Vector3(7,7.4,9).normalize();
    const right=new THREE.Vector3().crossVectors(new THREE.Vector3(0,1,0),direction).normalize(),up=new THREE.Vector3().crossVectors(direction,right);
    const tangent=Math.tan(THREE.MathUtils.degToRad(this.camera.fov/2));let distance=0;
    for(const x of [box.min.x,box.max.x])for(const y of [box.min.y,box.max.y])for(const z of [box.min.z,box.max.z]){const p=new THREE.Vector3(x,y,z).sub(target);distance=Math.max(distance,p.dot(direction)+Math.abs(p.dot(up))/tangent*1.12,p.dot(direction)+Math.abs(p.dot(right))/(tangent*this.camera.aspect)*1.12);}
    distance=Math.max(distance,this.controls.minDistance);
    this.controls.maxDistance=Math.max(30,distance*2);this.camera.far=Math.max(150,distance*6);this.camera.updateProjectionMatrix();
    this.moveCamera(target.clone().addScaledVector(direction,distance),target,smooth);
  }
  // 数据第一次到、换地图或换点云时调用：用户没手动动过视角才重新框。
  autoFrame(){if(!this.userMoved)this.reset(false);}
  localCamera(smooth=true){const p=this.robot.position.clone();this.moveCamera(p.clone().add(new THREE.Vector3(3,3.6,4)),p.clone().add(new THREE.Vector3(0,.3,0)),smooth);}
  applyCloudLook(){const live=this.source!=='demo';this.cloud.material.size=this.mode==='local'?.045:(live?.03:.022);this.cloud.material.opacity=this.mode==='local'||live?.95:.16;this.cloudUniforms.uRadius.value=this.mode==='local'?3.2:0;}
  setMode(mode){this.needsRender=true;this.renderer.shadowMap.needsUpdate=true;this.mode=mode;this.environment.visible=this.showGeometry && this.source==='demo' && mode==='global';this.applyCloudLook();this.reset();}
  setSource(source){this.needsRender=true;this.renderer.shadowMap.needsUpdate=true;this.source=source;this.applyCloudLook();this.environment.visible=this.showGeometry && source==='demo' && this.mode==='global';}
  setLayer(name,value){this.needsRender=true;this.renderer.shadowMap.needsUpdate=true;if(name==='geometry'){this.showGeometry=value;this.cloud.visible=value;this.environment.visible=value&&this.source==='demo'&&this.mode==='global';}if(name==='objects'){this.showObjects=value;this.markers.visible=value;this.highlights.visible=value;this.updateLabels();}if(name==='path'){this.showPath=value;this.pathLine.visible=value;}}
  // 新云到达时整体替换几何（类型数组直接做属性，不逐帧重建）；cloud 为 null 时清空。
  setCloud(cloud){
    this.needsRender=true;this.cloud.geometry.dispose();this.cloud.geometry=new THREE.BufferGeometry();this.cloudBox=null;this.cloudCount=0;
    if(cloud&&cloud.header.count>0){
      this.cloud.geometry.setAttribute('position',new THREE.BufferAttribute(cloud.positions,3));
      this.cloud.geometry.setAttribute('color',new THREE.BufferAttribute(cloud.colors,3,true));
      this.cloud.geometry.computeBoundingBox();this.cloud.updateMatrixWorld(true);
      this.cloudBox=this.cloud.geometry.boundingBox.clone().applyMatrix4(this.cloud.matrixWorld);this.cloudCount=cloud.header.count;
    }
    // 只读标注，便于现场排错与自动化核对当前画的是哪份云。
    this.canvas.dataset.cloudPoints=String(this.cloudCount);this.canvas.dataset.cloudRev=cloud?.header.rev||'';
  }
  styleRobot(snapshot){
    const r=snapshot.robot;if(!r)return;
    const stale=Number.isFinite(r.poseAgeMs)&&r.poseAgeMs>1000,grey=r.pose==='none',key=`${r.pose}|${stale}`;
    if(key===this.robotStyle)return;this.robotStyle=key;
    const g=new THREE.Color(0x9aa7b1);
    this.robot.traverse(o=>{const m=o.material,u=m?.userData;if(!u?.baseColor)return;m.color.copy(u.baseColor);if(grey)m.color.lerp(g,.85);m.transparent=stale||u.baseTransparent;m.opacity=(stale?.4:1)*u.baseOpacity;m.needsUpdate=true;});
    this.halo.visible=r.pose==='map';this.dashedHalo.visible=r.pose==='odometry';
  }
  update(snapshot){
    this.needsRender=true;
    this.renderer.shadowMap.needsUpdate=true;
    const oldPosition=this.snapshot?.robot?.position;this.snapshot=snapshot;this.markers.visible=this.showObjects;this.highlights.visible=this.showObjects;
    this.robot.visible=!!snapshot.robot;
    if(snapshot.robot){
      this.styleRobot(snapshot);
      const target=world(snapshot.robot.position),yaw=snapshot.robot.yaw-Math.PI/2;
      if(!oldPosition||this.reducedMotion.matches){this.robot.position.copy(target);this.robot.rotation.y=yaw;this.robotTween=null;}
      else {const delta=Math.atan2(Math.sin(yaw-this.robot.rotation.y),Math.cos(yaw-this.robot.rotation.y));if(target.distanceToSquared(this.robot.position)>.000001||Math.abs(delta)>.0001)this.robotTween={from:this.robot.position.clone(),to:target,yawFrom:this.robot.rotation.y,yawDelta:delta,start:performance.now(),duration:this.source==='demo'?950:200};}
    }
    const pathKey=JSON.stringify(snapshot.path);
    if(pathKey!==this.pathKey){this.pathKey=pathKey;this.pathLine.geometry.dispose();this.pathLine.geometry=new THREE.BufferGeometry().setFromPoints(snapshot.path.map(p=>world(p).add(new THREE.Vector3(0,.035,0))));this.pathLine.computeLineDistances();}
    const key=JSON.stringify(snapshot.objects.map(o=>[o.id,o.name,o.className,o.color,o.position,o.state]));
    if(key!==this.objectsKey){this.objectsKey=key;for(const c of [...this.markers.children]){c.geometry.dispose();c.material.dispose();this.markers.remove(c);}this.labelRoot.replaceChildren();this.labels=[];
      for(const o of snapshot.objects){const invalid=o.state==='position_invalid',occluded=o.state==='temporarily_occluded';const m=new THREE.Mesh(new THREE.RingGeometry(.15,.19,32),new THREE.MeshBasicMaterial({color:invalid?0x9aa7b1:0x7eebff,transparent:invalid||occluded,opacity:invalid?.6:occluded?.45:1,side:THREE.DoubleSide}));m.rotation.x=-Math.PI/2;m.position.copy(world(o.position)).add(new THREE.Vector3(0,.06,0));this.markers.add(m);const name=this.nameOf(o);const b=document.createElement('button');b.className='object-label';if(o.state)b.dataset.state=o.state;b.textContent=name;b.setAttribute('aria-label',`查看${name} ${o.id}`);b.onclick=()=>this.onSelect(o.id);this.labelRoot.append(b);this.labels.push({el:b,object:o,marker:m});}
    }
    const candidates=snapshot.task?.state==='ambiguous'?snapshot.task.candidates:[];
    const hkey=JSON.stringify(candidates.map(c=>[c.id,c.position]));
    if(hkey!==this.highlightKey){this.highlightKey=hkey;for(const c of [...this.highlights.children]){c.geometry.dispose();c.material.dispose();this.highlights.remove(c);}
      for(const c of candidates){const m=new THREE.Mesh(new THREE.RingGeometry(.24,.3,40),new THREE.MeshBasicMaterial({color:0xf29a3a,side:THREE.DoubleSide}));m.rotation.x=-Math.PI/2;m.position.copy(world(c.position)).add(new THREE.Vector3(0,.07,0));this.highlights.add(m);}}
    for(const l of this.labels||[])l.el.classList.toggle('candidate',candidates.some(c=>c.id===l.object.id));
    this.updateLabels();
  }
  clear(){this.needsRender=true;this.renderer.shadowMap.needsUpdate=true;this.robotTween=null;this.cameraTween=null;this.snapshot=null;this.objectsKey='';this.pathKey='';this.highlightKey='';this.robotStyle='';this.labelRoot.replaceChildren();this.labels=[];this.robot.visible=false;this.setCloud(null);this.pathLine.geometry.dispose();this.pathLine.geometry=new THREE.BufferGeometry();this.markers.visible=false;for(const c of [...this.highlights.children]){c.geometry.dispose();c.material.dispose();this.highlights.remove(c);}this.environment.visible=false;this.userMoved=false;}
  updateLabels(){const w=this.canvas.clientWidth,h=this.canvas.clientHeight;for(const {el,object:o,marker} of this.labels||[]){const v=world(o.position).add(new THREE.Vector3(0,.4,0)).project(this.camera);const near=this.mode!=='local'||(this.snapshot.robot&&Math.hypot(o.position[0]-this.snapshot.robot.position[0],o.position[1]-this.snapshot.robot.position[1])<3.2);marker.visible=this.showObjects && near;el.hidden=!this.showObjects||!near||v.z>1||v.z< -1;el.style.left=`${(v.x*.5+.5)*w}px`;el.style.top=`${(-v.y*.5+.5)*h}px`;el.classList.toggle('selected',o.id===this.selected);}const right=new THREE.Vector3().setFromMatrixColumn(this.camera.matrixWorld,0);const a=this.controls.target.clone().project(this.camera),b=this.controls.target.clone().add(right).project(this.camera);const scale=document.getElementById('scale-line');if(scale)scale.style.width=`${Math.abs(b.x-a.x)*w/2}px`;}
  focus(id){const o=this.snapshot?.objects.find(o=>o.id===id);if(!o)return;this.selected=id;const target=world(o.position);this.moveCamera(this.camera.position.clone().add(target.clone().sub(this.controls.target)),target);}
  zoom(factor){const offset=this.camera.position.clone().sub(this.controls.target);offset.setLength(THREE.MathUtils.clamp(offset.length()*factor,this.controls.minDistance,this.controls.maxDistance));this.moveCamera(offset.add(this.controls.target),this.controls.target);}
}
