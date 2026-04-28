
// ── Splash Screen ─────────────────────────────────────────────
function splashProgress(pct, msg) {
  var bar = document.getElementById('splash-bar');
  var txt = document.getElementById('splash-msg');
  if (bar) bar.style.width = pct + '%';
  if (txt && msg) txt.textContent = msg;
}

function splashHide() {
  var el = document.getElementById('splash-screen');
  if (!el) return;
  el.style.transition = 'opacity .5s ease';
  el.style.opacity = '0';
  setTimeout(function(){ el.style.display = 'none'; }, 520);
}

// Schrittweise Progress während Initialisierung
splashProgress(10, '3D Szene wird aufgebaut…');
'use strict';
// ═══════════════════════════════════════════════════
// KUKA KR8 R1420 HW — KINEMATIK
// Robot CS: X=vorwärts, Y=links, Z=oben
// Three.js scene: Z-up (camera up=(0,0,1)) → gleich wie Robot CS!
// ═══════════════════════════════════════════════════
const JOINTS_DEF = [
  {name:'A1',off:[150,0,450],  min:-170,max:170, axis:'Rz'},
  {name:'A2',off:[610,0,0],   min:-185,max:65,  axis:'Ry'},
  {name:'A3',off:[0,0,200],   min:-120,max:180, axis:'Ry'},
  {name:'A4',off:[630,0,0],   min:-165,max:165, axis:'Rx'},
  {name:'A5',off:[80,0,0],    min:-115,max:140, axis:'Ry'},
  {name:'A6',off:[0,0,0],     min:-180,max:180, axis:'Rx'}
];

const TCP_DEF = {x:364.5, y:0, z:46.5, a:0, b:90, c:0};
const FK_SIGNS = [-1,1,1,-1,1,-1];

// Current joint angles (degrees)
let jointAngles = [0,-90,90,0,0,0];

// ─── 3×3 matrix math ───
function mRz(a){const c=Math.cos(a),s=Math.sin(a);return[[c,-s,0],[s,c,0],[0,0,1]];}
function mRy(a){const c=Math.cos(a),s=Math.sin(a);return[[c,0,s],[0,1,0],[-s,0,c]];}
function mRx(a){const c=Math.cos(a),s=Math.sin(a);return[[1,0,0],[0,c,-s],[0,s,c]];}
function mMul(A,B){const R=[[0,0,0],[0,0,0],[0,0,0]];for(let i=0;i<3;i++)for(let j=0;j<3;j++)for(let k=0;k<3;k++)R[i][j]+=A[i][k]*B[k][j];return R;}
function mVec(M,v){return[M[0][0]*v[0]+M[0][1]*v[1]+M[0][2]*v[2],M[1][0]*v[0]+M[1][1]*v[1]+M[1][2]*v[2],M[2][0]*v[0]+M[2][1]*v[1]+M[2][2]*v[2]];}
function mT(M){return[[M[0][0],M[1][0],M[2][0]],[M[0][1],M[1][1],M[2][1]],[M[0][2],M[1][2],M[2][2]]];}
function rotZYX(a,b,c){const r=d=>d*Math.PI/180;return mMul(mRz(r(a)),mMul(mRy(r(b)),mRx(r(c))));}

// ─── FK ───────────────────────────────────────────
// Returns: { pts: [[x,y,z]×7, tcp], rot_final, rot_all }
function fkAll(angles_deg) {
  const r = a => a * Math.PI / 180;
  const axFns = [mRz,mRy,mRy,mRx,mRy,mRx];
  let pos = [0,0,0], rot = [[1,0,0],[0,1,0],[0,0,1]];
  const pts = [[0,0,0]];   // base origin
  const rots = [rot];

  for (let i = 0; i < 6; i++) {
    rot = mMul(rot, axFns[i](r(angles_deg[i] * FK_SIGNS[i])));
    const off = mVec(rot, JOINTS_DEF[i].off);
    pos = [pos[0]+off[0], pos[1]+off[1], pos[2]+off[2]];
    pts.push([...pos]);
    rots.push(rot);
  }

  // TCP: R_fc=Ry(90°) = KUKA Flansch-KS (Z vorwärts, X unten)
  const R_fc  = mRy(Math.PI/2);
  const R_usr = rotZYX(TCP_DEF.a, TCP_DEF.b, TCP_DEF.c);
  const R_tcp = mMul(R_fc, R_usr);
  const p_tcp_local = mVec(R_fc, [TCP_DEF.x, TCP_DEF.y, TCP_DEF.z]);
  const p_tcp_world = mVec(rot, p_tcp_local);
  const tcp = [pos[0]+p_tcp_world[0], pos[1]+p_tcp_world[1], pos[2]+p_tcp_world[2]];
  pts.push(tcp);

  const R_tcp_world = mMul(rot, R_tcp);
  return { pts, rot_final: rot, rots, tcp_rot: R_tcp_world };
}

// ─── IK (numerisch, DLS) ─────────────────────────
const ORI_SCALE = 100.0;

function logOriErr(Rt, R_cur) {
  const Re = mMul(Rt, mT(R_cur));
  const trace = Re[0][0]+Re[1][1]+Re[2][2];
  const cosT  = Math.max(-1, Math.min(1, (trace-1)*0.5));
  const theta = Math.acos(cosT);
  const sinT  = Math.sin(theta);
  if (sinT < 1e-7) return [0,0,0];
  const k = theta * ORI_SCALE / (2*sinT);
  return [(Re[2][1]-Re[1][2])*k, (Re[0][2]-Re[2][0])*k, (Re[1][0]-Re[0][1])*k];
}

function fkTCP_pos(angles_deg) {
  return fkAll(angles_deg).pts[7];
}

function fkTCP_full(angles_deg) {
  const r = fkAll(angles_deg);
  return { pos: r.pts[7], rot: r.tcp_rot };
}

function err6(angs, tp, Rt) {
  const fk = fkTCP_full(angs);
  const ep = [tp[0]-fk.pos[0], tp[1]-fk.pos[1], tp[2]-fk.pos[2]];
  const eo = logOriErr(Rt, fk.rot);
  return [...ep, ...eo];
}

function solve6x6(A, b) {
  const M = A.map((row,i) => [...row, b[i]]);
  for (let c=0; c<6; c++) {
    let mx=c;
    for (let r=c+1;r<6;r++) if (Math.abs(M[r][c])>Math.abs(M[mx][c])) mx=r;
    [M[c],M[mx]]=[M[mx],M[c]];
    if (Math.abs(M[c][c])<1e-14) continue;
    for (let r=c+1;r<6;r++) {
      const f=M[r][c]/M[c][c];
      for (let j=c;j<=6;j++) M[r][j]-=f*M[c][j];
    }
  }
  const x=Array(6).fill(0);
  for (let i=5;i>=0;i--) {
    if (Math.abs(M[i][i])<1e-14) continue;
    x[i]=M[i][6];
    for (let j=i+1;j<6;j++) x[i]-=M[i][j]*x[j];
    x[i]/=M[i][i];
  }
  return x;
}

function solveIK(tx, ty, tz, ta, tb, tc, initAngles) {
  const clamp=(v,lo,hi)=>Math.max(lo,Math.min(hi,v));
  const tp = [tx, ty, tz];
  const Rt = rotZYX(ta, tb, tc);
  const dt=0.3, lam=0.5, tolP=0.5, tolO=0.5;

  const starts = [
    initAngles || jointAngles.slice(),
    [0,-90,90,0,0,0], [0,-90,90,0,-45,0], [0,-90,90,0,-90,0],
    [0,-90,90,-90,-45,0], [0,-90,90,90,-45,0],
    [0,-120,110,0,-45,0], [0,-60,60,0,-45,0],
  ];

  let bestScore=Infinity, bestQ=JOINTS_DEF.map((_,i)=>jointAngles[i]);

  for (const start of starts) {
    let q=[...start];
    for (let iter=0; iter<300; iter++) {
      const e=err6(q, tp, Rt);
      const eP=Math.sqrt(e[0]**2+e[1]**2+e[2]**2);
      const eO=Math.sqrt(e[3]**2+e[4]**2+e[5]**2);
      const score=eP+eO;
      if (score<bestScore){bestScore=score;bestQ=[...q];}
      if (eP<tolP&&eO<tolO) break;

      const J=[];
      for (let i=0;i<6;i++){
        const q1=[...q]; q1[i]+=dt;
        const e1=err6(q1,tp,Rt);
        J.push([(e1[0]-e[0])/dt,(e1[1]-e[1])/dt,(e1[2]-e[2])/dt,
                (e1[3]-e[3])/dt,(e1[4]-e[4])/dt,(e1[5]-e[5])/dt]);
      }
      const JtJ=Array.from({length:6},()=>Array(6).fill(0));
      const Jte=Array(6).fill(0);
      for (let i=0;i<6;i++){
        for (let r=0;r<6;r++){
          Jte[i]+=J[i][r]*e[r];
          for (let j=0;j<6;j++) JtJ[i][j]+=J[i][r]*J[j][r];
        }
        JtJ[i][i]+=lam;
      }
      const dq=solve6x6(JtJ,Jte);
      const step=Math.min(2.0,10.0/Math.max(1,bestScore));
      for (let i=0;i<6;i++){
        if (!isFinite(dq[i])) continue;
        q[i]=clamp(q[i]-step*dq[i], JOINTS_DEF[i].min, JOINTS_DEF[i].max);
      }
    }
    if (bestScore<(tolP+tolO)*1.5) break;
  }
  return {angles:bestQ, score:bestScore, ok: bestScore<20};
}

// ═══════════════════════════════════════════════════
// KRL PARSER (original vom KUKA Simulator)
// ═══════════════════════════════════════════════════
const KW=new Set(['LIN','PTP','SLIN','CIRC','DECL','IF','ELSE','ENDIF','FOR','ENDFOR',
  'WHILE','ENDWHILE','LOOP','ENDLOOP','DEF','END','DEFFCT','ENDFCT','RETURN','WAIT','HALT']);
let parsedData={positions:[],finalState:{variables:{},digitalIn:{},digitalOut:{},analogOut:{}}};

function parsePos(str){
  const p={X:0,Y:0,Z:0,A:0,B:0,C:0,S:null,T:null};
  let m;const rx=/([XYZABCST])\s+([-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)/g;
  while((m=rx.exec(str))!==null)if(m[1]in p)p[m[1]]=parseFloat(m[2]);
  return p;
}
function parseVal(s){
  s=s.trim().replace(/;.*$/,'').trim();
  if(/^TRUE$/i.test(s))return true;if(/^FALSE$/i.test(s))return false;
  const n=Number(s);return(!isNaN(n)&&s!=='')?n:s.replace(/^"|"$/g,'');
}
function parseKRL(code){
  const lines=code.split(/\r?\n/);const steps=[],positions=[],vars={},din={},dout={},anout={};
  function snap(){return{variables:{...vars},digitalIn:{...din},digitalOut:{...dout},analogOut:{...anout}};}
  function pushStep(ln,type,extra){steps.push({lineNum:ln,type,...extra,snapshot:snap()});}
  for(let ln=0;ln<lines.length;ln++){
    let raw=lines[ln];const ci=raw.indexOf(';');if(ci===0)continue;
    let line=(ci>0?raw.slice(0,ci):raw).trim();if(!line)continue;
    const circM=line.match(/^CIRC\s+(\{[^}]+\})\s*,\s*(\{[^}]+\})/i);
    if(circM){const s=snap();const pi=positions.length;
      positions.push({type:'CIRC_AUX',...parsePos(circM[1].slice(1,-1)),lineNum:ln,snapshot:s});
      positions.push({type:'CIRC',...parsePos(circM[2].slice(1,-1)),lineNum:ln,snapshot:s});
      pushStep(ln,'move',{posIdx:pi+1,label:'CIRC'});continue;}
    const circP=line.match(/^CIRC\s+(\{[^}]+\})\s*,?\s*$/i);
    if(circP){let next='';
      for(let j=ln+1;j<Math.min(ln+3,lines.length);j++){next=lines[j].trim().replace(/^;.*/,'');if(next)break;}
      const endM=next.match(/^\{([^}]+)\}/);
      if(endM){const s=snap(),pi=positions.length;
        positions.push({type:'CIRC_AUX',...parsePos(circP[1].slice(1,-1)),lineNum:ln,snapshot:s});
        positions.push({type:'CIRC',...parsePos(endM[1]),lineNum:ln,snapshot:s});
        pushStep(ln,'move',{posIdx:pi+1,label:'CIRC'});}continue;}
    const moveM=line.match(/^(LIN|PTP|SLIN)\s+\{([^}]+)\}/i);
    if(moveM){const pi=positions.length;
      const moveType=moveM[1].toUpperCase();
      const parsedMove=parsePos(moveM[2]);
      // PTP mit Achswinkeln (A1..A6) → als ptpAngles speichern
      const axisM=moveM[2].match(/A1\s*([-\d.]+)/i);
      if(moveType==='PTP' && axisM) {
        const aMatch=moveM[2].match(/A([1-6])\s*([-\d.]+)/gi)||[];
        const ptpQ=[0,-90,90,0,0,0];
        aMatch.forEach(function(s){const m2=s.match(/A([1-6])\s*([-\d.]+)/i);if(m2)ptpQ[+m2[1]-1]=parseFloat(m2[2]);});
        pushStep(ln,'ptpAxis',{angles:ptpQ.slice()});continue;
      }
      positions.push({type:moveType,...parsedMove,lineNum:ln,snapshot:snap()});
      pushStep(ln,'move',{posIdx:pi,label:moveType});continue;}
    let m;
    if((m=line.match(/^\$IN\s*\[(\d+)\]\s*=\s*(.+)/i))){din[+m[1]]=parseVal(m[2]);pushStep(ln,'signal',{});continue;}
    if((m=line.match(/^\$OUT\s*\[(\d+)\]\s*=\s*(.+)/i))){dout[+m[1]]=parseVal(m[2]);pushStep(ln,'signal',{});continue;}
    if((m=line.match(/^\$ANOUT\s*\[(\d+)\]\s*=\s*(.+)/i))){const v=parseFloat(m[2]);anout[+m[1]]=isNaN(v)?0:Math.max(-10,Math.min(10,v));pushStep(ln,'signal',{});continue;}
    if((m=line.match(/^DECL\s+\S+\s+([A-Za-z_]\w*)\s*=\s*(.+)/i))){vars[m[1]]=parseVal(m[2]);pushStep(ln,'var',{});continue;}
    if((m=line.match(/^([A-Za-z_]\w*)\s*=\s*(.+)/))){if(!KW.has(m[1].toUpperCase())){vars[m[1]]=parseVal(m[2]);pushStep(ln,'var',{});continue;}}
    pushStep(ln,'other',{});
  }
  return{steps,positions,finalState:{variables:{...vars},digitalIn:{...din},digitalOut:{...dout},analogOut:{...anout}}};
}

// ═══════════════════════════════════════════════════
// THREE.JS SETUP (Z-up scene = KUKA Robot CS)
// ═══════════════════════════════════════════════════
const canvas=document.getElementById('c3d');
const renderer=new THREE.WebGLRenderer({canvas,antialias:true});
renderer.setPixelRatio(window.devicePixelRatio);
const scene=new THREE.Scene();
scene.background=new THREE.Color(0x070d1a);

const perspCam=new THREE.PerspectiveCamera(50,1,1,80000);
perspCam.up.set(0,0,1);
const orthoCam=new THREE.OrthographicCamera(-2000,2000,2000,-2000,-50000,50000);
orthoCam.up.set(0,0,1);
let activeCam=perspCam, orthoHalfSize=2000, currentView='iso';

// Lights
scene.add(new THREE.AmbientLight(0xffffff, 0.85));
const sun=new THREE.DirectionalLight(0xffffff,0.8);
sun.position.set(1000,1000,2000);scene.add(sun);
const fill=new THREE.DirectionalLight(0xffffff,0.4);fill.position.set(-1000,-500,1000);scene.add(fill);

// Grid (horizontal, Z=0)
const grid=new THREE.GridHelper(8000,40,0x0e1e30,0x0a1828);
grid.rotation.x=Math.PI/2;
// GridHelper uses LineSegments with array material
function setGridColor(col){
  if(Array.isArray(grid.material)){grid.material.forEach(m=>m.color.set(col));}
  else{grid.material.color.set(col);}
}
scene.add(grid);

// World origin frame
function wArrow(d,col,len,org){scene.add(new THREE.ArrowHelper(new THREE.Vector3(...d).normalize(),new THREE.Vector3(...org),len,col,len*.18,len*.09));}
wArrow([1,0,0],0xff3333,300,[0,0,0]);
wArrow([0,1,0],0x33ff33,300,[0,0,0]);
wArrow([0,0,1],0x3377ff,300,[0,0,0]);

// Groups
const posGrp=new THREE.Group();scene.add(posGrp);
const pathGrp=new THREE.Group();scene.add(pathGrp);
const stlGrp=new THREE.Group();scene.add(stlGrp);
const robotGrp=new THREE.Group();scene.add(robotGrp);
const tcpTraceGrp=new THREE.Group();scene.add(tcpTraceGrp);

// Simulation marker (white sphere + CS arrows, like KUKA sim)

// BASE Koordinatensystem (Weltkoordinaten)
const baseFrameGrp = new THREE.Group();
(function() {
  var sz = 150;
  var dirs = [[1,0,0,0xff3333],[0,1,0,0x33ff33],[0,0,1,0x3388ff]];
  dirs.forEach(function(d) {
    baseFrameGrp.add(new THREE.ArrowHelper(
      new THREE.Vector3(d[0],d[1],d[2]),
      new THREE.Vector3(0,0,0),
      sz, d[3], sz*.2, sz*.1
    ));
  });
})();
scene.add(baseFrameGrp);
const markerGrp=new THREE.Group();scene.add(markerGrp);
const markerVisuals=new THREE.Group();markerGrp.add(markerVisuals);
{
  const sp=new THREE.Mesh(new THREE.SphereGeometry(28,12,12),new THREE.MeshBasicMaterial({color:0xf05500,transparent:true,opacity:.9}));
  markerVisuals.add(sp);
  const ring=new THREE.Mesh(new THREE.TorusGeometry(36,5,6,24),new THREE.MeshBasicMaterial({color:0xff6600}));
  ring.rotation.x=Math.PI/2;markerVisuals.add(ring);
  for(const[d,c]of[[[1,0,0],0xff6666],[[0,1,0],0x66ff66],[[0,0,1],0x6699ff]])
    markerVisuals.add(new THREE.ArrowHelper(new THREE.Vector3(...d),new THREE.Vector3(),190,c,38,19));
  markerGrp.visible=false;
}
const selSphere=new THREE.Mesh(new THREE.SphereGeometry(34,12,12),new THREE.MeshBasicMaterial({color:0x00ccff,transparent:true,opacity:.35,depthTest:false}));
selSphere.visible=false;scene.add(selSphere);

const TYPE_COL={LIN:0xf05500,PTP:0xffaa00,SLIN:0x00aaff,CIRC:0xaa44ff,CIRC_AUX:0x445566};
let frameSize=40,sphereSize=7;
let traceLineWidth=2, pathLineWidth=2;  // Linienstärken
// SVG Icons für Visibility-Zustände
var svgIconSolid  = '<svg viewBox="0 0 24 24" width="16" height="16"><circle cx="12" cy="12" r="10" fill="#f58220"/></svg>';
var svgIconTransp = '<svg viewBox="0 0 24 24" width="16" height="16"><circle cx="12" cy="12" r="9" fill="none" stroke="#f58220" stroke-width="3"/></svg>';
var svgIconHidden = '<svg viewBox="0 0 24 24" width="16" height="16"><circle cx="12" cy="12" r="9" fill="none" stroke="#f58220" stroke-width="3"/><line x1="4" y1="20" x2="20" y2="4" stroke="#f58220" stroke-width="3" stroke-linecap="round"/></svg>';
var sceneSTLMode = { pedestal:'solid', tool:'solid' };
function getSTLVisIcon(m){return m==='solid'?svgIconSolid:m==='transparent'?svgIconTransp:svgIconHidden;}
function applyMeshMode(mesh,mode){if(!mesh)return;if(mode==='hidden'){mesh.visible=false;}else if(mode==='transparent'){mesh.visible=true;mesh.material.transparent=true;mesh.material.opacity=0.3;mesh.material.depthWrite=false;mesh.material.needsUpdate=true;}else{mesh.visible=true;mesh.material.transparent=false;mesh.material.opacity=1;mesh.material.depthWrite=true;mesh.material.needsUpdate=true;}}
function cycleSTLMode(type,idx){var c={solid:'transparent',transparent:'hidden',hidden:'solid'};var btn,mode;if(type==='axis'){axisSTLMode[idx]=c[axisSTLMode[idx]]||'solid';mode=axisSTLMode[idx];btn=document.getElementById('asl-vis'+idx);if(btn)btn.innerHTML=getSTLVisIcon(mode);applyMeshMode(axisSTLMeshes[idx],mode);}else if(type==='pedestal'){sceneSTLMode.pedestal=c[sceneSTLMode.pedestal]||'solid';mode=sceneSTLMode.pedestal;btn=document.getElementById('vis-pedestal');if(btn)btn.innerHTML=getSTLVisIcon(mode);applyMeshMode(pedestalMesh,mode);}else if(type==='tool'){sceneSTLMode.tool=c[sceneSTLMode.tool]||'solid';mode=sceneSTLMode.tool;btn=document.getElementById('vis-tool');if(btn)btn.innerHTML=getSTLVisIcon(mode);applyMeshMode(toolMesh,mode);}}
function setAxisSTLMode(idx,mode){axisSTLMode[idx]=mode;var btn=document.getElementById('asl-vis'+idx);if(btn)btn.innerHTML=getSTLVisIcon(mode);applyMeshMode(axisSTLMeshes[idx],mode);}
var axisSTLMode = ['solid','solid','solid','solid','solid','solid']; // solid|transparent|hidden

function kukaEuler(A,B,C){const r=THREE.MathUtils.degToRad;return new THREE.Euler(r(C),r(B),r(A),'ZYX');}

function makeFrame(pos){
  const grp=new THREE.Group();
  for(const[d,c]of[[[1,0,0],0xff4444],[[0,1,0],0x44ff44],[[0,0,1],0x4488ff]])
    grp.add(new THREE.ArrowHelper(new THREE.Vector3(...d),new THREE.Vector3(),frameSize,c,frameSize*.2,frameSize*.1));
  grp.add(new THREE.Mesh(new THREE.SphereGeometry(sphereSize,8,8),new THREE.MeshBasicMaterial({color:TYPE_COL[pos.type]!==undefined?TYPE_COL[pos.type]:0xffffff})));
  grp.position.set(pos.X,pos.Y,pos.Z);
  grp.setRotationFromEuler(kukaEuler(pos.A,pos.B,pos.C));
  grp.userData.posIdx=-1;return grp;
}

function rebuildFrames(){
  posGrp.clear(); posGrp.visible = showPosFrames;
  parsedData.positions.forEach((pos,i)=>{const g=makeFrame(pos);g.userData.posIdx=i;posGrp.add(g);});
  if(selectedPosIdx!==null){const p=parsedData.positions[selectedPosIdx];if(p)selSphere.position.set(p.X,p.Y,p.Z);}
  markerVisuals.scale.setScalar(frameSize/120);
}

document.getElementById('frame-size-sl').addEventListener('input',function(){
  frameSize=parseInt(this.value);document.getElementById('frame-size-v').textContent=frameSize;rebuildFrames();
});
document.getElementById('sphere-size-sl').addEventListener('input',function(){
  sphereSize=parseInt(this.value);document.getElementById('sphere-size-v').textContent=sphereSize;rebuildFrames();
});

// ═══════════════════════════════════════════════════
// ROBOT 3D MODEL
// ═══════════════════════════════════════════════════
let robotColor=0xcc4400, jointColor=0xe8a020, tcpColor=0xffee00;
let showRobot3D=true, showTCPTrace=true;
const tcpTracePoints=[];

function hexToInt(hex){return parseInt(hex.slice(1),16);}

function buildCylinder(from,to,radius,color){
  const v1=new THREE.Vector3(...from),v2=new THREE.Vector3(...to);
  const dir=new THREE.Vector3().subVectors(v2,v1);
  const len=dir.length();if(len<1)return null;
  const geo=new THREE.CylinderGeometry(radius,radius,len,8);
  const mat=new THREE.MeshPhongMaterial({color, shininess:80, specular:0x444444});
  const mesh=new THREE.Mesh(geo,mat);
  const mid=new THREE.Vector3().addVectors(v1,v2).multiplyScalar(.5);
  mesh.position.copy(mid);
  const up=new THREE.Vector3(0,1,0);
  const dn=dir.clone().normalize();
  const ax=new THREE.Vector3().crossVectors(up,dn).normalize();
  const ang=Math.acos(Math.max(-1,Math.min(1,up.dot(dn))));
  if(ax.length()>0.001)mesh.quaternion.setFromAxisAngle(ax,ang);
  else if(dn.y<0)mesh.rotation.z=Math.PI;
  return mesh;
}

// ── STL axis model state ──────────────────────────────────────
const axisSTLMeshes  = [null,null,null,null,null,null];
const axisSTLBase64  = [null,null,null,null,null,null];
const axisPivots     = [];   // THREE.Group per axis (hierarchical chain)
let pedestalMesh     = null;
let showSkeleton     = true;
let showPosFrames    = true;   // Zielkoordinatensysteme (posGrp)
let showBaseFrame    = true;   // BASE Koordinatensystem
let showTCPMarker    = true;   // TCP Frame (markerVisuals)
let showSTLRobot     = true;
let showToolMesh     = true;
let showPedestalMesh = true;
let stlRefAngles     = [0,-90,90,0,0,0];  // pose at which STL files are modelled

// Build pivot chain once; re-build when JOINTS_DEF changes
function buildPivotChain() {
  axisPivots.forEach(function(g){ if(g.parent) g.parent.remove(g); });
  axisPivots.length = 0;
  let parent = robotGrp;
  for (let i = 0; i < 6; i++) {
    const g = new THREE.Group();
    // Each pivot is offset by its joint's link offset IN the parent's local frame
    if (i === 0) g.position.set(0, 0, 0);
    else { const o = JOINTS_DEF[i-1].off; g.position.set(o[0],o[1],o[2]); }
    parent.add(g);
    axisPivots.push(g);
    parent = g;
  }
  // Reattach any loaded STL meshes with corrected local offsets
  attachSTLMeshesToPivots();
}

// STL axis meshes are in scene directly (like pedestal)
// Their world transform is updated in buildRobotModel via updateAxisSTLTransforms
function attachSTLMeshesToPivots() {
  // no-op: meshes are in scene, not in pivot hierarchy
}

// Update pivot rotations from current joint angles
function updatePivotRotations(angles) {
  const r = Math.PI / 180;
  for (let i = 0; i < 6; i++) {
    if (!axisPivots[i]) continue;
    const a = angles[i] * FK_SIGNS[i] * r;
    axisPivots[i].rotation.set(0,0,0);
    switch(JOINTS_DEF[i].axis) {
      case 'Rz': axisPivots[i].rotation.z = a; break;
      case 'Ry': axisPivots[i].rotation.y = a; break;
      case 'Rx': axisPivots[i].rotation.x = a; break;
    }
  }
}

function buildRobotModel(angles) {
  // Ensure pivot chain exists
  if (!axisPivots.length) buildPivotChain();

  // Clear non-pivot children of robotGrp (skeleton primitives only)
  const _toRemove = robotGrp.children.filter(ch => !axisPivots.includes(ch));
  _toRemove.forEach(ch => robotGrp.remove(ch));

  // Update pivot rotations (moves STL meshes with joints)
  updatePivotRotations(angles);

  // STL: visibility + world transform from FK
  {
    const ref  = fkAll(stlRefAngles);
    const curr = fkAll(angles);
    const _m4  = new THREE.Matrix4();
    const _q   = new THREE.Quaternion();
    for (let i = 0; i < 6; i++) {
      const mesh = axisSTLMeshes[i];
      if (!mesh) continue;
      mesh.visible = showSTLRobot && showRobot3D;
      if (!mesh.visible) continue;
      // R_delta = curRot[i+1] × refRot[i+1]^T
      const R = mMul(curr.rots[i+1], mT(ref.rots[i+1]));
      _m4.set(R[0][0],R[0][1],R[0][2],0,
              R[1][0],R[1][1],R[1][2],0,
              R[2][0],R[2][1],R[2][2],0,
              0,0,0,1);
      _q.setFromRotationMatrix(_m4);
      mesh.quaternion.copy(_q);
      // translation = curPivot - R × refPivot
      const rp = mVec(R, ref.pts[i]);
      mesh.position.set(
        curr.pts[i][0] - rp[0],
        curr.pts[i][1] - rp[1],
        curr.pts[i][2] - rp[2]
      );
    }
  }


  // Tool STL: origin at A6 flange center — follow A6 pose
  if (toolMesh) {
    toolMesh.visible = showRobot3D && showToolMesh;
    if (toolMesh.visible) {
      const fkC = fkAll(angles);
      const Rt0 = fkC.rots[6];
      // Tool KS: Rz(180°) so X zeigt nach unten
      const Rz180 = [[-1,0,0],[0,-1,0],[0,0,1]];
      const Rt = [
        [Rt0[0][0]*Rz180[0][0]+Rt0[0][1]*Rz180[1][0]+Rt0[0][2]*Rz180[2][0],
         Rt0[0][0]*Rz180[0][1]+Rt0[0][1]*Rz180[1][1]+Rt0[0][2]*Rz180[2][1],
         Rt0[0][0]*Rz180[0][2]+Rt0[0][1]*Rz180[1][2]+Rt0[0][2]*Rz180[2][2]],
        [Rt0[1][0]*Rz180[0][0]+Rt0[1][1]*Rz180[1][0]+Rt0[1][2]*Rz180[2][0],
         Rt0[1][0]*Rz180[0][1]+Rt0[1][1]*Rz180[1][1]+Rt0[1][2]*Rz180[2][1],
         Rt0[1][0]*Rz180[0][2]+Rt0[1][1]*Rz180[1][2]+Rt0[1][2]*Rz180[2][2]],
        [Rt0[2][0]*Rz180[0][0]+Rt0[2][1]*Rz180[1][0]+Rt0[2][2]*Rz180[2][0],
         Rt0[2][0]*Rz180[0][1]+Rt0[2][1]*Rz180[1][1]+Rt0[2][2]*Rz180[2][1],
         Rt0[2][0]*Rz180[0][2]+Rt0[2][1]*Rz180[1][2]+Rt0[2][2]*Rz180[2][2]],
      ];
      const _m4t = new THREE.Matrix4();
      _m4t.set(Rt[0][0],Rt[0][1],Rt[0][2],0,
               Rt[1][0],Rt[1][1],Rt[1][2],0,
               Rt[2][0],Rt[2][1],Rt[2][2],0,
               0,0,0,1);
      const _qt = new THREE.Quaternion();
      _qt.setFromRotationMatrix(_m4t);
      toolMesh.quaternion.copy(_qt);
      toolMesh.position.set(fkC.pts[6][0], fkC.pts[6][1], fkC.pts[6][2]);
    }
  }

  // Pedestal visibility
  if (pedestalMesh) pedestalMesh.visible = showPedestalMesh;

  // ── Skeleton ──
  if (showSkeleton && showRobot3D) {
    const fk = fkAll(angles);
    const pts = fk.pts;
    const radii=[40,38,30,24,20,16], linkRadii=[28,20,16,12,8,6];
    for(let i=0;i<7;i++){const f=pts[i],t=pts[i+1];if(!f||!t)continue;const cyl=buildCylinder(f,t,i<linkRadii.length?linkRadii[i]:5,robotColor);if(cyl)robotGrp.add(cyl);}
    for(let i=1;i<=6;i++){const p=pts[i],r=radii[i-1]||10;const s=new THREE.Mesh(new THREE.SphereGeometry(r,12,8),new THREE.MeshPhongMaterial({color:jointColor,shininess:120,specular:0x666666}));s.position.set(p[0],p[1],p[2]);robotGrp.add(s);}
    const tcp=pts[7];
    if(tcp){const R=fk.tcp_rot,aLen=150;for(const[[cx,cy,cz],col] of [[[1,0,0],0xff4444],[[0,1,0],0x44ff44],[[0,0,1],0x4488ff]]){const wd=mVec(R,[cx,cy,cz]);robotGrp.add(new THREE.ArrowHelper(new THREE.Vector3(...wd).normalize(),new THREE.Vector3(...tcp),aLen,col,aLen*.2,aLen*.1));}}
  }
}

// ── STL loading for axes ──────────────────────────────────────
function buildAxisSTLUI() {
  const el = document.getElementById('axis-stl-ui');
  if (!el) return;
  el.innerHTML = JOINTS_DEF.map((j,i) =>
    `<div class="axis-stl-row">
      <span class="axis-stl-lbl">A${i+1}</span>
      <span class="axis-stl-name" id="asl-name${i}">—</span>
      <button class="stl-vis-btn" id="asl-vis${i}" onclick="cycleSTLMode('axis',${i})" title="Klicken zum Wechseln">${svgIconSolid}</button>
      <button class="axis-stl-btn" onclick="pickAxisSTL(${i})">+ STL</button>
      <button class="axis-stl-btn" id="asl-del${i}" style="display:none;color:var(--err)" onclick="removeAxisSTL(${i})">✕</button>
    </div>`
  ).join('') +
  `<div style="display:flex;align-items:center;gap:5px;margin-top:6px;font-size:.75em;border-top:1px solid var(--bdr);padding-top:5px">
    <span style="color:var(--txt3);flex:1">Referenz-Pose A1..A6:</span>
    <input id="stl-ref-angles" type="text" value="0,-90,90,0,0,0"
      style="background:var(--bg1);color:#9ecfea;border:1px solid var(--bdr);border-radius:3px;padding:1px 4px;font-size:.9em;width:130px;font-family:inherit"
      onchange="applySTLRefAngles(this.value)" title="Winkel in Grad (kommagetrennt) bei denen die STL-Dateien erstellt wurden">
  </div>`;
}

function applySTLRefAngles(val) {
  const parts = val.split(',').map(v=>parseFloat(v.trim()));
  if (parts.length===6 && parts.every(v=>!isNaN(v))) {
    stlRefAngles = parts;
    updateSTLRefFK();
    buildRobotModel(jointAngles);
  }
}

function pickAxisSTL(idx) {
  const inp = document.getElementById('axis-stl-file');
  inp.onchange = e => {
    const file = e.target.files[0]; if (!file) return;
    loadAxisSTL(idx, file); inp.value='';
  };
  inp.click();
}

function loadAxisSTL(idx, file) {
  const reader = new FileReader();
  reader.onload = e => {
    const geo = stlLoader.parse(e.target.result);
    geo.computeVertexNormals();
    if (axisSTLMeshes[idx]) {
      scene.remove(axisSTLMeshes[idx]);
      axisSTLMeshes[idx].geometry.dispose();
    }
    axisSTLMeshes[idx] = new THREE.Mesh(geo,
      new THREE.MeshPhongMaterial({color:0xe8a020, shininess:80}));
    scene.add(axisSTLMeshes[idx]);
    // Store raw base64 for later save
    axisSTLBase64[idx] = btoa(String.fromCharCode.apply(null, new Uint8Array(e.target.result)));
  window['_axisSTLBuffer'+idx] = e.target.result;
  if (!window._axisSTLBuffers) window._axisSTLBuffers = {};
  window._axisSTLBuffers[idx] = e.target.result;
    document.getElementById('asl-name'+idx).textContent = file.name.replace(/\.stl$/i,'');
    document.getElementById('asl-del'+idx).style.display = '';
  };
  reader.readAsArrayBuffer(file);
}

function removeAxisSTL(idx) {
  if (axisSTLMeshes[idx]) {
    scene.remove(axisSTLMeshes[idx]);
    axisSTLMeshes[idx].geometry.dispose();
    axisSTLMeshes[idx] = null;
  }
  document.getElementById('asl-name'+idx).textContent = '—';
  document.getElementById('asl-del'+idx).style.display = 'none';
  buildRobotModel(jointAngles);
}

function loadAxisSTLFromBase64(idx, b64, filename, rawBuffer) {
  const buffer = rawBuffer || (() => {
    const bin = atob(b64);
    const buf = new ArrayBuffer(bin.length);
    const view = new Uint8Array(buf);
    for (let i = 0; i < bin.length; i++) view[i] = bin.charCodeAt(i);
    return buf;
  })();
  try {
    const geo  = stlLoader.parse(buffer);
    geo.computeVertexNormals();
    if (axisSTLMeshes[idx]) { scene.remove(axisSTLMeshes[idx]); axisSTLMeshes[idx].geometry.dispose(); }
    axisSTLMeshes[idx] = new THREE.Mesh(geo, new THREE.MeshPhongMaterial({color:0xe8a020,shininess:80}));
    scene.add(axisSTLMeshes[idx]);
    if (b64) axisSTLBase64[idx] = b64;
    const nameEl = document.getElementById('asl-name'+idx);
    if (nameEl) nameEl.textContent = filename.replace(/\.stl$/i,'');
    const delEl = document.getElementById('asl-del'+idx);
    if (delEl) delEl.style.display = '';
    buildRobotModel(jointAngles);
  } catch(e) { console.error('STL load error A'+(idx+1), e); }
}

function loadPedestalSTL() {
  const inp = document.getElementById('pedestal-stl-file');
  inp.onchange = e => {
    const file = e.target.files[0]; if (!file) return;
    const reader = new FileReader();
    reader.onload = ev => {
      const geo = stlLoader.parse(ev.target.result);
      geo.computeVertexNormals();
      if (pedestalMesh) { scene.remove(pedestalMesh); pedestalMesh.geometry.dispose(); }
      pedestalMesh = new THREE.Mesh(geo, new THREE.MeshPhongMaterial({color:0x334455,shininess:40}));
      scene.add(pedestalMesh);
      document.getElementById('pedestal-name').textContent = file.name.replace(/\.stl$/i,'');
    };
    reader.readAsArrayBuffer(file);
    inp.value = '';
  };
  inp.click();
}

function toggleSkeleton() {
  showSkeleton = !showSkeleton;
  document.getElementById('btn-show-skeleton').classList.toggle('on', showSkeleton);
  buildRobotModel(jointAngles);
}

function toggleSTLRobot() {
  showSTLRobot = !showSTLRobot;
  document.getElementById('btn-show-stl-robot').classList.toggle('on', showSTLRobot);
  buildRobotModel(jointAngles);
}



















// TCP Trace
function addTCPTracePoint(tcp) {
  if (!showTCPTrace) return;
  tcpTracePoints.push(new THREE.Vector3(...tcp));
  if (tcpTracePoints.length > 2000) tcpTracePoints.shift();
  rebuildTCPTrace();
}

function rebuildTCPTrace() {
  tcpTraceGrp.clear();
  if (tcpTracePoints.length < 2) return;
  const geo = new THREE.BufferGeometry().setFromPoints(tcpTracePoints);
  var _traceColEl = document.getElementById('cfg-trace-col') || document.getElementById('cfg-path-col');
  const col = hexToInt(_traceColEl.value);
  const mat = new THREE.LineBasicMaterial({color: col, opacity:.5, transparent:true, linewidth: traceLineWidth});
  tcpTraceGrp.add(new THREE.Line(geo, mat));
}

// Color update


document.getElementById('cfg-tcp-col').addEventListener('input', function(){
  tcpColor=hexToInt(this.value); buildRobotModel(jointAngles);
});

document.getElementById('btn-robot3d').addEventListener('click', function(){
  showRobot3D=!showRobot3D;this.classList.toggle('on',showRobot3D);
  if(!showRobot3D){ buildRobotModel(jointAngles); } else buildRobotModel(jointAngles);
});
function toggleProjection(){
  var isPersp = activeCam === perspCam;
  if(isPersp){
    // Wechsel zu Ortho — aktuelle View-Richtung beibehalten (ISO bleibt ISO)
    activeCam = orthoCam;
    // currentView unverändert lassen
    document.getElementById('btn-persp').setAttribute('data-i18n','ortho');
    document.getElementById('btn-persp').textContent = t('ortho');
    document.getElementById('btn-persp').classList.remove('on');
  } else {
    activeCam = perspCam;
    // currentView unverändert lassen
    document.getElementById('btn-persp').setAttribute('data-i18n','persp');
    document.getElementById('btn-persp').textContent = t('persp');
    document.getElementById('btn-persp').classList.add('on');
  }
  updateCamera();resize();
}

// ── Steuerungspanel ────────────────────────────────────────
// ══════════════════════════════════════════════
// SAVE / LOAD — Kinematik, TCP, Programm
// ══════════════════════════════════════════════

function downloadFile(content, filename) {
  const a = document.createElement('a');
  a.href = 'data:text/plain;charset=utf-8,' + encodeURIComponent(content);
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
}

// ── Kinematik ──────────────────────────────────
// STL offset/rotation per axis (set via UI, default 0)
var axisSTLOffsets = [
  {px:0,py:0,pz:0,rx:0,ry:0,rz:0},
  {px:0,py:0,pz:0,rx:0,ry:0,rz:0},
  {px:0,py:0,pz:0,rx:0,ry:0,rz:0},
  {px:0,py:0,pz:0,rx:0,ry:0,rz:0},
  {px:0,py:0,pz:0,rx:0,ry:0,rz:0},
  {px:0,py:0,pz:0,rx:0,ry:0,rz:0}
];
// Scene STL offsets (pedestal, tool)
var sceneSTLOffsets = {
  pedestal: {px:0,py:0,pz:0,rx:0,ry:0,rz:0, name:'podest'},
  tool:     {px:0,py:0,pz:0,rx:0,ry:0,rz:0, name:'tool1_tcp'}
};

function getKinematicData() {
  var data = {
    name: (document.getElementById('kin-name')&&document.getElementById('kin-name').value) || 'Kinematik',
    joints: JOINTS_DEF.map(function(j) {
      return {name:j.name, axis:j.axis, offset:{x:j.off[0],y:j.off[1],z:j.off[2]}, min:j.min, max:j.max};
    }),
    stlRefAngles: stlRefAngles,
    stlFiles: {}
  };
  for (var i = 0; i < 6; i++) {
    var name = (document.getElementById('asl-name'+i)&&document.getElementById('asl-name'+i).textContent) || '';
    var off = axisSTLOffsets[i];
    data.stlFiles['A'+(i+1)] = {
      name: name || ('a'+(i+1)),
      posx: off.px, posy: off.py, posz: off.pz,
      posrx: off.rx, posry: off.ry, posrz: off.rz,
      color: (axisSTLMeshes[i] && axisSTLMeshes[i].material)
        ? '#' + axisSTLMeshes[i].material.color.getHexString() : '#e8a020'
    };
  }
  // Scene models
  data.sceneModels = {
    pedestal: Object.assign({}, sceneSTLOffsets.pedestal),
    tool:     Object.assign({}, sceneSTLOffsets.tool)
  };
  return data;
}

function getKinematicJSON() {
  return JSON.stringify(getKinematicData(), null, 2);
}

function saveKinematic() {
  var kinName = ((document.getElementById('kin-name')&&document.getElementById('kin-name').value) || 'kinematik').replace(/\s+/g,'_');
  var data = getKinematicData();

  if (typeof JSZip === 'undefined') {
    // Fallback: JSON only
    downloadFile(JSON.stringify(data, null, 2), kinName + '.json');
    return;
  }

  var zip = new JSZip();
  // JSON config (no STL data embedded)
  zip.file(kinName + '.json', JSON.stringify(data, null, 2));

  // Axis STLs — direkt aus ArrayBuffer (kein Base64-Umweg)
  var axisAdded = 0;
  for (var i = 0; i < 6; i++) {
    var buf = window['_axisSTLBuffer'+i];
    if (!buf) buf = window._axisSTLBuffers && window._axisSTLBuffers[i];
    if (buf) {
      var stlName = (data.stlFiles['A'+(i+1)] && data.stlFiles['A'+(i+1)].name) || ('a'+(i+1));
      if (stlName.toLowerCase().indexOf('.stl') === -1) stlName += '.stl';
      zip.file(stlName, new Uint8Array(buf));
      axisAdded++;
    } else if (axisSTLBase64[i]) {
      // Fallback: base64 → binary
      var stlName2 = (data.stlFiles['A'+(i+1)] && data.stlFiles['A'+(i+1)].name) || ('a'+(i+1));
      if (stlName2.toLowerCase().indexOf('.stl') === -1) stlName2 += '.stl';
      try {
        var bin = atob(axisSTLBase64[i]);
        var arr = new Uint8Array(bin.length);
        for (var k = 0; k < bin.length; k++) arr[k] = bin.charCodeAt(k);
        zip.file(stlName2, arr);
        axisAdded++;
      } catch(ex) { console.warn('ZIP: STL base64 error A'+(i+1), ex); }
    }
  }
  console.log('ZIP: ' + axisAdded + ' Achsen-STLs hinzugefügt');

  // Pedestal STL
  if (window._pedestalSTLBuffer) {
    var pName = (sceneSTLOffsets.pedestal.name || 'podest') + '.stl';
    zip.file(pName, new Uint8Array(window._pedestalSTLBuffer));
  }

  // Tool STL
  if (window._toolSTLBuffer) {
    var tName = (sceneSTLOffsets.tool.name || 'tool1_tcp') + '.stl';
    zip.file(tName, new Uint8Array(window._toolSTLBuffer));
  }

  zip.generateAsync({type:'blob', compression:'DEFLATE', compressionOptions:{level:6}})
    .then(function(blob) {
      var url = URL.createObjectURL(blob);
      var a = document.createElement('a');
      a.href = url; a.download = kinName + '.zip';
      document.body.appendChild(a); a.click();
      document.body.removeChild(a);
      setTimeout(function(){ URL.revokeObjectURL(url); }, 2000);
    });
}

function loadKinematic() {
  var inp = document.getElementById('kin-file-in');
  inp.onchange = function(e) {
    var file = e.target.files[0]; if (!file) return;
    if (file.name.endsWith('.zip')) {
      loadKinematicZIP(file);
    } else {
      var reader = new FileReader();
      reader.onload = function(ev) {
        try {
          var data = file.name.endsWith('.xml')
            ? parseKinematicXML(ev.target.result)
            : JSON.parse(ev.target.result);
          applyKinematicData(data, {});
        } catch(err) { alert('Fehler beim Laden: ' + err.message); }
        inp.value = '';
      };
      reader.readAsText(file);
    }
    inp.value = '';
  };
  inp.click();
}

function loadKinematicZIP(file) {
  if (typeof JSZip === 'undefined') {
    alert('JSZip nicht geladen');
    return;
  }
  JSZip.loadAsync(file).then(function(zip) {
    // Find JSON config
    var jsonFile = null;
    zip.forEach(function(path, entry) {
      if (!entry.dir && path.endsWith('.json')) jsonFile = entry;
    });
    if (!jsonFile) { alert('Keine JSON-Konfiguration in ZIP gefunden'); return; }

    jsonFile.async('string').then(function(jsonStr) {
      var data;
      try { data = JSON.parse(jsonStr); } catch(e) { alert('JSON Fehler: ' + e.message); return; }

      // Collect STL buffers from ZIP
      var stlBuffers = {};
      var stlPromises = [];
      zip.forEach(function(path, entry) {
        if (!entry.dir && path.toLowerCase().endsWith('.stl')) {
          var fname = path.replace(/.*\//, '').replace(/\.stl$/i, '').toLowerCase();
          stlPromises.push(
            entry.async('arraybuffer').then(function(buf) {
              stlBuffers[fname] = buf;
            })
          );
        }
      });

      Promise.all(stlPromises).then(function() {
        applyKinematicData(data, stlBuffers);
      });
    });
  }).catch(function(e) { alert('ZIP Fehler: ' + e.message); });
}

function parseKinematicXML(xml) {
  const parser = new DOMParser();
  const doc = parser.parseFromString(xml, 'text/xml');
  const name = (function(){var _e=doc.querySelector('kinematik');return _e&&_e.getAttribute('name');})() || 'Kinematik';
  const joints = [...doc.querySelectorAll('joint')].map(j => ({
    name: j.getAttribute('name'),
    axis: j.getAttribute('axis'),
    offset: {
      x: parseFloat((function(){var _e=j.querySelector('offset');return _e&&_e.getAttribute('x');})() || 0),
      y: parseFloat((function(){var _e=j.querySelector('offset');return _e&&_e.getAttribute('y');})() || 0),
      z: parseFloat((function(){var _e=j.querySelector('offset');return _e&&_e.getAttribute('z');})() || 0)
    },
    min: parseFloat(j.getAttribute('min') || -180),
    max: parseFloat(j.getAttribute('max') || 180)
  }));
  return {name, joints};
}

function applyKinematicData(data, stlBuffers) {
  stlBuffers = stlBuffers || {};
  if (data.name) {
    const el = document.getElementById('kin-name');
    if (el) el.value = data.name;
    // Update section title
    const secT = document.querySelector('.sec-t span:last-of-type');
  }
  if (data.joints) {
    // Extend or replace JOINTS_DEF
    while (JOINTS_DEF.length < data.joints.length)
      JOINTS_DEF.push({name:'A'+(JOINTS_DEF.length+1), off:[0,0,0], min:-180, max:180, axis:'Ry'});
    data.joints.forEach((j, i) => {
      if (!JOINTS_DEF[i]) return;
      if (j.offset) { JOINTS_DEF[i].off[0]=j.offset.x||0; JOINTS_DEF[i].off[1]=j.offset.y||0; JOINTS_DEF[i].off[2]=j.offset.z||0; }
      if (j.min !== undefined) JOINTS_DEF[i].min = j.min;
      if (j.max !== undefined) JOINTS_DEF[i].max = j.max;
      if (j.axis) JOINTS_DEF[i].axis = j.axis;
      if (j.name) JOINTS_DEF[i].name = j.name;
    });
  }
  buildKinConfig();
  buildSteuerAxes();
  // Restore STL files from embedded data
  if (data.stlRefAngles) {
    stlRefAngles = data.stlRefAngles;
    const inp = document.getElementById('stl-ref-angles');
    if (inp) inp.value = stlRefAngles.join(',');
  }
  if (data.stlFiles) {
    Object.entries(data.stlFiles).forEach(function(entry) {
      var key = entry[0]; var val = entry[1];
      if (!val) return;
      var idx = parseInt(key.replace('A','')) - 1;
      if (idx < 0 || idx > 5) return;
      // Store offset/rotation
      if (axisSTLOffsets[idx]) {
        axisSTLOffsets[idx] = {
          px: val.posx||0, py: val.posy||0, pz: val.posz||0,
          rx: val.posrx||0, ry: val.posry||0, rz: val.posrz||0
        };
      }
      // Try ZIP buffer first, then embedded base64
      var fname = (val.name || key.toLowerCase()).replace(/\.stl$/i,'').toLowerCase();
      if (stlBuffers[fname]) {
        loadAxisSTLFromBase64(idx, null, (val.name||key)+'.stl', stlBuffers[fname]);
      } else if (val.data) {
        loadAxisSTLFromBase64(idx, val.data, val.name || key+'.stl');
      }
    });
  }
  // Scene models from ZIP
  if (data.sceneModels) {
    var sm = data.sceneModels;
    if (sm.pedestal) {
      sceneSTLOffsets.pedestal = Object.assign(sceneSTLOffsets.pedestal, sm.pedestal);
      var pname = (sm.pedestal.name || 'podest').toLowerCase();
      if (stlBuffers[pname]) {
        var buf = stlBuffers[pname];
        window._pedestalSTLBuffer = buf;
        var geo = stlLoader.parse(buf); geo.computeVertexNormals();
        if (pedestalMesh) { scene.remove(pedestalMesh); pedestalMesh.geometry.dispose(); }
        pedestalMesh = new THREE.Mesh(geo, new THREE.MeshPhongMaterial({color:0x334455,shininess:40}));
        scene.add(pedestalMesh);
        var el = document.getElementById('pedestal-name');
        if (el) el.textContent = pname;
      }
    }
    if (sm.tool) {
      sceneSTLOffsets.tool = Object.assign(sceneSTLOffsets.tool, sm.tool);
      var tname = (sm.tool.name || 'tool1_tcp').toLowerCase();
      if (stlBuffers[tname]) {
        var tbuf = stlBuffers[tname];
        window._toolSTLBuffer = tbuf;
        var tgeo = stlLoader.parse(tbuf); tgeo.computeVertexNormals();
        if (toolMesh) { scene.remove(toolMesh); toolMesh.geometry.dispose(); toolMesh.material.dispose(); }
        var mat = new THREE.MeshPhongMaterial({color:0xdd9944,transparent:false,opacity:1.0,side:THREE.DoubleSide,specular:0x666666});
        toolMesh = new THREE.Mesh(tgeo, mat);
        scene.add(toolMesh);
        document.getElementById('tool-filename').textContent = tname;
        document.getElementById('tool-controls').style.display = 'block';
      }
    }
  }
  buildRobotModel(jointAngles);
  setStatus('paused', 'Kinematik geladen');
}

function newKinematic() {
  if (!confirm(t('confirm_new_kin'))) return;
  JOINTS_DEF.forEach(j => { j.off=[0,0,0]; j.min=-180; j.max=180; });
  const el = document.getElementById('kin-name');
  if (el) el.value = 'Neue Kinematik';
  buildKinConfig();
  buildRobotModel(jointAngles);
}

// ── TCP ──────────────────────────────────────
function getTCPJSON() {
  return JSON.stringify({
    name: 'TCP',
    tcp: {x: TCP_DEF.x, y: TCP_DEF.y, z: TCP_DEF.z, a: TCP_DEF.a, b: TCP_DEF.b, c: TCP_DEF.c}
  }, null, 2);
}

function saveTCP() {
  downloadFile(getTCPJSON(), 'tcp.json');
}

function loadTCP() {
  const inp = document.getElementById('tcp-file-in');
  inp.onchange = e => {
    const file = e.target.files[0]; if (!file) return;
    const reader = new FileReader();
    reader.onload = ev => {
      try {
        let data;
        if (file.name.endsWith('.xml')) {
          const doc = new DOMParser().parseFromString(ev.target.result, 'text/xml');
          const t = doc.querySelector('tcp');
          data = {tcp:{x:+(t&&t.getAttribute)('x'),y:+(t&&t.getAttribute)('y'),z:+(t&&t.getAttribute)('z'),a:+(t&&t.getAttribute)('a'),b:+(t&&t.getAttribute)('b'),c:+(t&&t.getAttribute)('c')}};
        } else {
          data = JSON.parse(ev.target.result);
        }
        if (data.tcp) {
          const t = data.tcp;
          ['x','y','z','a','b','c'].forEach(k => {
            const el = document.getElementById('tcp-'+k);
            if (el && t[k] !== undefined) el.value = t[k];
          });
          updateTCPDef();
          setStatus('paused', 'TCP geladen');
        }
      } catch(err) { alert('Fehler: ' + err.message); }
      inp.value = '';
    };
    reader.readAsText(file);
  };
  inp.click();
}

// ── Programm ────────────────────────────────
function saveProgram() {
  const code = document.getElementById('code-input').value;
  downloadFile(code, 'programm.src');
}

function loadProgram() {
  const inp = document.getElementById('prog-file-in');
  inp.onchange = e => {
    const file = e.target.files[0]; if (!file) return;
    const reader = new FileReader();
    reader.onload = ev => {
      document.getElementById('code-input').value = ev.target.result;
      rebuildGutter();
      setStatus('paused', 'Programm geladen: ' + file.name);
    };
    reader.readAsText(file);
    inp.value = '';
  };
  inp.click();
}


// ═══════════════════════════════════════════════════
// ACHSENKARTE — Redundanter Achsenoptimierer
// ═══════════════════════════════════════════════════
let ampAxis = 'A6';          // which axis to show
let ampMap = null;           // Float32Array [cols × rows]: 0=valid,1=limit,2=singular,3=unreachable
let ampCols = 0, ampRows = 180;
let ampUserPath = [];        // user-defined angle per column - persistent
let ampAutoPath = [];        // auto-optimized angle per column
let ampDragging = false;


// ── Achsenkarte auf Viewport ausrichten ───────────────────────
function ampAlignToViewport() {
  var panel = document.getElementById('axis-map-panel');
  if (!panel) return;
  var vp = document.querySelector('.vp');
  if (!vp) return;
  var r = vp.getBoundingClientRect();
  panel.style.left  = r.left + 'px';
  panel.style.width = r.width + 'px';
}
function toggleAxisMap() {
  const p = document.getElementById('axis-map-panel');
  p.classList.toggle('visible');
  if (p.classList.contains('visible')) ampAlignToViewport();
  document.getElementById('btn-axmap').classList.toggle('on', p.classList.contains('visible'));
  if (p.classList.contains('visible') && trajectory.length > 0) {
    setTimeout(ampBuild, 50);  // auf Anforderung wenn Map geöffnet wird
  }
}

function setAmpAxis(ax) {
  ampAxis = ax;
  document.querySelectorAll('.amp-tab').forEach(b => b.classList.toggle('on', b.textContent === ax));
  // Achsenkarte wird nur auf Anforderung gebaut (Performance)
  // if (trajectory.length > 0) ampBuild();
}

// Build the map: for each trajectory step × angle value → classify
// ═══════════════════════════════════════════════════
// ACHSENKARTE A6 — Analytisches Modell
//
// Werkzeug-Symmetrieachse: A6 frei wählbar.
// Für jede Zelle (Schritt × A6-Ziel):
//   1. A6 außerhalb Achsgrenzen → PINK (Endschalter)
//   2. A4 nach Kompensation außerhalb → PINK
//   3. |A5| < Schwelle → ORANGE (Singularität)
//   4. Sonst VALID (schwarz)
//
// Kompensation: A4_neu = A4_alt - (A6_ziel - A6_alt)
// Hält Handgelenks-Orientierung annähernd konstant.
// ═══════════════════════════════════════════════════
const SING_THRESH = 6;
// A6_MIN/MAX dynamisch aus JOINTS_DEF[5]
var A6_MIN = JOINTS_DEF[5].min, A6_MAX = JOINTS_DEF[5].max;
function ampBuild() {
  // A6_MIN/MAX immer aktuell aus JOINTS_DEF laden
  A6_MIN = JOINTS_DEF[5].min;
  A6_MAX = JOINTS_DEF[5].max;
  if (!trajectory.length) return;
  const canvas = document.getElementById('amp-canvas');
  const wrap   = document.getElementById('amp-canvas-wrap');
  const W = Math.max(1, wrap.clientWidth  - 42);
  const H = Math.max(1, wrap.clientHeight - 22);
  canvas.width = W; canvas.height = H;
  canvas.style.position = 'absolute'; canvas.style.left = '42px';

  // Grid: 300 cols × 180 rows (2° per row from -360 to +360)
  const COLS = Math.min(trajectory.length, 300);
  const ROWS = 180;
  ampCols = COLS; ampRows = ROWS;
  ampMap  = new Uint8Array(COLS * ROWS);

  let totalDist = 0;
  for (let i=1; i<trajectory.length; i++) {
    const a=trajectory[i-1].pos, b=trajectory[i].pos;
    totalDist += Math.sqrt((b.X-a.X)**2+(b.Y-a.Y)**2+(b.Z-a.Z)**2);
  }

  const a4L = JOINTS_DEF[3];  // A4 limits [-185,+185]
  const a6L = JOINTS_DEF[5];  // A6 limits dynamisch

  // ── Analytical map: no IK per cell ──────────────────────
  // For a rotationally symmetric tool:
  //   A6 is freely choosable.
  //   Compensation: A4_new = A4_cur - (A6_target - A6_cur)
  //   to keep wrist orientation approximately constant.
  //
  // Status per cell:
  //   0 = valid (black)
  //   1 = joint limit: A6 outside limits OR A4 after compensation outside limits (pink)
  //   2 = singularity: |A5| < SING_THRESH (orange)
  //   3 = limit + singularity (red)

  // Bogenlängen-Synchronisation: col → tidx über gleichmäßige Bogenlänge
  window._ampArcLen = [0];
  var arcLen = window._ampArcLen;
  arcLen.length = 0; arcLen.push(0);
  var _refT = trajectoryRef.length ? trajectoryRef : trajectory;
  for (var _ti = 1; _ti < _refT.length; _ti++) {
    var _p0 = _refT[_ti-1].pos;
    var _p1 = _refT[_ti].pos;
    var _dx = _p1[0]-_p0[0], _dy = _p1[1]-_p0[1], _dz = _p1[2]-_p0[2];
    arcLen.push(arcLen[_ti-1] + Math.sqrt(_dx*_dx+_dy*_dy+_dz*_dz));
  }
  var totalArc = arcLen[arcLen.length-1] || 1;
  var TLEN = _refT.length;

  function arcToTidx(col) {
    var s = (col / Math.max(1, COLS-1)) * totalArc;
    var lo = 0, hi = arcLen.length - 1;
    while (lo < hi) {
      var mid = (lo + hi) >> 1;
      if (arcLen[mid] < s) lo = mid + 1; else hi = mid;
    }
    var refLen = (trajectoryRef.length ? trajectoryRef : trajectory).length;
    return Math.min(lo, refLen - 1);
  }

  // ── Vollständige IK pro Zelle (Spec Kap. 9) ─────────────────
  // Async Chunk-Processing: Browser friert nicht ein
  var _col = 0;
  var _canvas = canvas, _W = W, _H = H;
  var _totalDist = totalDist;

  document.getElementById('amp-info').textContent = 'Berechne Map… 0%';

  function processChunk() {
    var chunkEnd = Math.min(_col + 10, COLS);
    for (var col = _col; col < chunkEnd; col++) {
    var tidx = arcToTidx(col);
    var refTraj = trajectoryRef.length ? trajectoryRef : trajectory;
    var entry = refTraj[Math.min(tidx, refTraj.length-1)];
    var ang = entry.angles;
    var pos = entry.pos;
    var posX = pos.X !== undefined ? pos.X : (pos[0]||0);
    var posY = pos.Y !== undefined ? pos.Y : (pos[1]||0);
    var posZ = pos.Z !== undefined ? pos.Z : (pos[2]||0);
    var posA = pos.A !== undefined ? pos.A : (pos[3]||0);
    var posB = pos.B !== undefined ? pos.B : (pos[4]||0);
    var posC = pos.C !== undefined ? pos.C : (pos[5]||0);
    var a6Ref = ang[5];

    // Referenz-Zeile (Startpunkt für den Scan)
    var refRow = Math.round((a6Ref - A6_MIN) / (A6_MAX - A6_MIN) * (ROWS-1));
    refRow = Math.max(0, Math.min(ROWS-1, refRow));

    function computeCell(row, warmQ) {
      var a6t = A6_MIN + (row / (ROWS-1)) * (A6_MAX - A6_MIN);
      if (a6t < a6L.min || a6t > a6L.max) {
        ampMap[col * ROWS + row] = 1; return null;
      }
      // Neue Orientierung: nur Rz ändert sich
      var Anew = posA + (a6t - a6Ref);
      while (Anew >  180) Anew -= 360;
      while (Anew <= -180) Anew += 360;
      // Vollständige IK — alle Achsen frei
      var res = solveIKFast(posX, posY, posZ, Anew, posB, posC, warmQ);
      if (!res.ok) { ampMap[col * ROWS + row] = 1; return null; }
      var q = res.angles;
      // Alle Achslimits prüfen
      for (var k=0; k<6; k++) {
        if (q[k] < JOINTS_DEF[k].min || q[k] > JOINTS_DEF[k].max) {
          ampMap[col * ROWS + row] = 1; return q;
        }
      }
      // Singularität: A5 nahe 0°
      if (Math.abs(q[4]) < SING_THRESH) {
        ampMap[col * ROWS + row] = 2; return q;
      }
      ampMap[col * ROWS + row] = 0; return q;
    }

    // Aufwärts von refRow → ROWS-1
    var prevQ = ang.slice();
    for (var row=refRow; row<ROWS; row++) {
      var q2 = computeCell(row, prevQ);
      if (q2) prevQ = q2;
    }
    // Abwärts von refRow-1 → 0
    prevQ = ang.slice();
    for (var row=refRow-1; row>=0; row--) {
      var q2 = computeCell(row, prevQ);
      if (q2) prevQ = q2;
    }
  }

    _col = chunkEnd;
    var pct = Math.round(_col / COLS * 100);
    document.getElementById('amp-info').textContent = 'Berechne Map… ' + pct + '%';
    if (_col < COLS) {
      setTimeout(processChunk, 0);
    } else {
      // Fertig — zeichnen
      _finishBuild(_canvas, _W, _H, _totalDist);
    }
  }
  setTimeout(processChunk, 0);
}

function _finishBuild(canvas, W, H, totalDist) {
  var COLS = ampCols, ROWS = ampRows;
  // Auto path aus aktuellen Trajektorie-Winkeln
  ampAutoPath = [];
  for (var col=0; col<COLS; col++) {
    var tidx=Math.round(col/Math.max(1,COLS-1)*(trajectory.length-1));
    ampAutoPath.push(trajectory[tidx].angles[5]);
  }
  if (!ampUserPath.length || ampUserPath.length !== COLS) {
    ampUserPath = ampAutoPath.slice();
  } else {
    var oldPath = ampUserPath.slice();
    var oldLen  = oldPath.length;
    var newPath = [];
    for (var ci = 0; ci < COLS; ci++) {
      var f = ci / Math.max(1, COLS-1) * (oldLen-1);
      var i0 = Math.floor(f), i1 = Math.min(i0+1, oldLen-1);
      newPath.push(oldPath[i0] + (oldPath[i1]-oldPath[i0])*(f-i0));
    }
    ampUserPath = newPath;
  }

  ampDraw(canvas, W, H);
  ampBuildXAxis(COLS, totalDist);

  var sing=0, lim=0, valid=0;
  for (var i=0; i<COLS*ROWS; i++) {
    var s=ampMap[i];
    if(s===2||s===3)sing++;else if(s===1)lim++;else valid++;
  }
  var pct=function(v){return Math.round(v/(COLS*ROWS)*100);};
  document.getElementById('amp-info').textContent =
    COLS+'×'+ROWS+' Zellen · Gültig: '+pct(valid)+'% · Limit: '+pct(lim)+'% · Sing.: '+pct(sing)+'%';
}

function ampDraw(canvas, W, H) {
  const ctx = canvas.getContext('2d');
  const img = ctx.createImageData(W, H);
  const d   = img.data;

  // Colors: valid, limit(pink), singularity(orange), both(red)
  const COLORS = [
    [12, 28, 52],      // 0 valid
    [170, 40, 130],    // 1 limit (pink)
    [200, 90, 0],      // 2 singularity (orange)
    [150, 15, 15],     // 3 both (red)
  ];

  for (let px = 0; px < W; px++) {
    const col = Math.round(px / Math.max(1,W-1) * (ampCols-1));
    for (let py = 0; py < H; py++) {
      // py=0 = top = A6_MAX, py=H-1 = bottom = A6_MIN
      const deg = A6_MAX - (py / (H-1)) * (A6_MAX - A6_MIN);
      const row = Math.round((deg - A6_MIN) / (A6_MAX - A6_MIN) * (ampRows-1));
      const rowC = Math.max(0, Math.min(ampRows-1, row));
      const status = (ampMap[col * ampRows + rowC]||0);
      const [r,g,b] = COLORS[status];
      const i = (py * W + px) * 4;
      d[i]=r; d[i+1]=g; d[i+2]=b; d[i+3]=255;
    }
  }
  ctx.putImageData(img, 0, 0);

  // Grid lines at -180, 0, +180
  ctx.strokeStyle='rgba(255,255,255,0.12)'; ctx.lineWidth=1; ctx.setLineDash([4,4]);
  for (const deg of [-180, 0, 180]) {
    const py = H - ((deg - A6_MIN) / (A6_MAX - A6_MIN)) * H;
    ctx.beginPath(); ctx.moveTo(0,py); ctx.lineTo(W,py); ctx.stroke();
  }
  ctx.setLineDash([]);

  // A6 limit bands (hatched)
  const limMin = JOINTS_DEF[5].min, limMax = JOINTS_DEF[5].max;
  const pyLimBot = H - ((limMin - A6_MIN) / (A6_MAX - A6_MIN)) * H;
  const pyLimTop = H - ((limMax - A6_MIN) / (A6_MAX - A6_MIN)) * H;
  ctx.fillStyle = 'rgba(180,40,130,0.15)';
  ctx.fillRect(0, pyLimBot, W, H - pyLimBot);  // below min
  ctx.fillRect(0, 0, W, pyLimTop);              // above max

  // User path (yellow, thick)
  ctx.strokeStyle='#ffee00'; ctx.lineWidth=2.5; ctx.setLineDash([]);
  ctx.beginPath();
  for (let px = 0; px < W; px++) {
    const col = Math.round(px / Math.max(1,W-1) * (ampCols-1));
    const deg = ampUserPath[col] || 0;
    const py  = H - ((deg - A6_MIN) / (A6_MAX - A6_MIN)) * H;
    px === 0 ? ctx.moveTo(px, py) : ctx.lineTo(px, py);
  }
  ctx.stroke();

  // Y-Achse Labels
  var yaxisEl = document.getElementById('amp-yaxis');
  if (yaxisEl) {
    yaxisEl.height = H;
    var yctx = yaxisEl.getContext('2d');
    yctx.clearRect(0, 0, 42, H);
    yctx.fillStyle = '#3a6080';
    yctx.font = '10px monospace';
    yctx.textAlign = 'right';
    var range = A6_MAX - A6_MIN;
    var step  = range <= 90 ? 15 : range <= 180 ? 30 : 60;
    for (var yd = A6_MIN; yd <= A6_MAX + 0.1; yd += step) {
      var frac = (yd - A6_MIN) / range;
      var py4  = Math.round((1 - frac) * (H - 1));  // +180 oben, -180 unten
      yctx.fillStyle = yd === 0 ? '#9ecfea' : '#3a6080';
      yctx.fillText(yd.toFixed(0) + '°', 38, py4 + 4);
      // Tick
      yctx.fillStyle = '#1a3050';
      yctx.fillRect(38, py4, 4, 1);
    }
  }

  // Cursor drawn on amp-cursor overlay canvas

  // First and last control points (draggable diamonds)
  // Zone shading for drag areas
  const zone = Math.max(15, Math.round(ampCols * 0.15));
  const zoneW1 = zone / (ampCols-1) * (W-1);
  const zoneW2 = (ampCols-1-zone) / (ampCols-1) * (W-1);
  ctx.fillStyle='rgba(0,255,136,0.06)'; ctx.fillRect(0,0,zoneW1,H);
  ctx.fillStyle='rgba(0,170,255,0.06)'; ctx.fillRect(zoneW2,0,W-zoneW2,H);
  ctx.strokeStyle='rgba(0,255,136,0.2)'; ctx.lineWidth=1; ctx.setLineDash([3,3]);
  ctx.beginPath(); ctx.moveTo(zoneW1,0); ctx.lineTo(zoneW1,H); ctx.stroke();
  ctx.strokeStyle='rgba(0,170,255,0.2)';
  ctx.beginPath(); ctx.moveTo(zoneW2,0); ctx.lineTo(zoneW2,H); ctx.stroke();
  ctx.setLineDash([]);

  function drawCtrlPt(col, color, label) {
    const deg = ampUserPath[Math.max(0,Math.min(ampCols-1,col))] || 0;
    const px2 = col / Math.max(1,ampCols-1) * (W-1);
    const py2 = H - ((deg - A6_MIN) / (A6_MAX - A6_MIN)) * H;
    ctx.fillStyle=color; ctx.strokeStyle='#fff'; ctx.lineWidth=2;
    ctx.beginPath();
    ctx.moveTo(px2,py2-11); ctx.lineTo(px2+11,py2); ctx.lineTo(px2,py2+11); ctx.lineTo(px2-11,py2); ctx.closePath();
    ctx.fill(); ctx.stroke();
    ctx.fillStyle='#fff'; ctx.font='bold 11px monospace';
    ctx.fillText(label, px2+13, py2+4);
  }
  // Alle Zielpunkte als Punkte auf der Planlinie
  if (ampUserPath.length > 0 && parsedData.positions.length > 1) {
    var N_pts = parsedData.positions.length;
    for (var pi = 0; pi < N_pts; pi++) {
      var pxP = (pi / (N_pts - 1)) * (W - 1);
      // A6 von der Planlinie (nicht vom IK-Solver) — Punkt liegt IMMER auf der Linie
      var col_idx = Math.round(pi / Math.max(1, N_pts-1) * (ampCols-1));
      col_idx = Math.max(0, Math.min(ampCols-1, col_idx));
      var a6 = ampUserPath[col_idx] !== undefined ? ampUserPath[col_idx] : 0;
      var pyP = H - ((a6 - A6_MIN) / (A6_MAX - A6_MIN)) * H;
      pyP = Math.max(3, Math.min(H-3, pyP));
      // Farbe je nach Bereichsstatus an dieser Stelle
      var row_idx = Math.round((a6 - A6_MIN) / (A6_MAX - A6_MIN) * (ampRows-1));
      row_idx = Math.max(0, Math.min(ampRows-1, row_idx));
      var status = ampMap ? (ampMap[col_idx * ampRows + row_idx] || 0) : 0;
      var dotColor = status === 0 ? '#00ddaa' : status === 1 ? '#ff60c0' : status === 2 ? '#ff8800' : '#ff3030';
      ctx.fillStyle = dotColor;
      ctx.strokeStyle = 'rgba(0,0,0,0.7)';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.arc(pxP, pyP, 8, 0, Math.PI * 2);
      ctx.fill(); ctx.stroke();
      // Positionsnummer
      ctx.fillStyle = 'rgba(255,255,255,0.8)';
      ctx.font = 'bold 8px monospace';
      ctx.fillText(pi + 1, pxP + 6, pyP - 3);
    }
  }
  drawCtrlPt(0,           '#00ff88', 'P1 ↕');
  drawCtrlPt(ampCols-1,   '#00aaff', 'Pn ↕');
  ampUpdateCursor();
}

function ampBuildXAxis(cols, totalDistMm) {
  const el = document.getElementById('amp-xaxis');
  el.innerHTML = '';
  for (let i = 0; i <= 6; i++) {
    const mm = Math.round(totalDistMm * i / 6);
    const span = document.createElement('span');
    span.textContent = mm >= 1000 ? `${(mm/1000).toFixed(1)}m` : `${mm}mm`;
    el.appendChild(span);
  }
}

function ampAutoOptimize() {
  if (!ampMap || !ampCols) return;
  const path = [];
  let current = ampUserPath[0] || 0;

  for (let col = 0; col < ampCols; col++) {
    // Find valid A6 nearest to current, prefer no singularity
    let bestDeg = current, bestScore = Infinity;
    for (let row = 0; row < ampRows; row++) {
      const deg = A6_MIN + (row / (ampRows-1)) * (A6_MAX - A6_MIN);
      const status = ampMap[col * ampRows + row];
      if (status === 0) {  // only valid zones
        const score = Math.abs(deg - current);
        if (score < bestScore) { bestScore = score; bestDeg = deg; }
      }
    }
    // If no valid zone, stay as close to limits as possible
    if (bestScore === Infinity) {
      bestDeg = Math.max(JOINTS_DEF[5].min, Math.min(JOINTS_DEF[5].max, current));
    }
    path.push(bestDeg);
    current = bestDeg;
  }

  // Smooth pass
  for (let pass = 0; pass < 5; pass++) {
    for (let col = 1; col < ampCols-1; col++) {
      const avg = (path[col-1] + path[col+1]) / 2;
      const row = Math.round((avg - A6_MIN) / (A6_MAX - A6_MIN) * (ampRows-1));
      if (row >= 0 && row < ampRows && ampMap[col * ampRows + row] === 0) {
        path[col] = avg;
      }
    }
  }

  ampUserPath = path;
  const canvas = document.getElementById('amp-canvas');
  ampDraw(canvas, canvas.width, canvas.height);
  document.getElementById('amp-info').textContent = 'Plan optimiert. "Weg übernehmen" zum Anwenden.';
}

function ampApplyPath() {
  if (!ampUserPath.length || !trajectory.length) return;

  document.getElementById('amp-info').textContent = 'IK wird neu berechnet…';

  // Für jeden Trajektorie-Schritt: A6-Zielwert aus Map → IK neu lösen
  {
    let errCount = 0;
    for (let tidx = 0; tidx < trajectory.length; tidx++) {
      if (!trajectory[tidx]) continue;

      // A6-Zielwert aus Map-Pfad (bogenlaengen-interpoliert)
      const colF = tidx / Math.max(1,trajectory.length-1) * (ampCols-1);
      const col0 = Math.floor(colF), col1 = Math.min(col0+1, ampCols-1);
      const frac = colF - col0;
      const a6target = ampUserPath[col0] + (ampUserPath[col1]-ampUserPath[col0]) * frac;
      const a6clamped = Math.max(JOINTS_DEF[5].min, Math.min(JOINTS_DEF[5].max, a6target));

      // Strategie: A6 direkt setzen, A4 kompensiert die Differenz
      // (haelt Wrist-Orientierung konstant: delta_A4 = -delta_A6)
      const curAngles = trajectory[tidx].angles.slice();
      const deltaA6 = a6clamped - curAngles[5];
      const newAngles = curAngles.slice();
      newAngles[5] = a6clamped;
      // A4-Kompensation (Wrist-Redundanz)
      const a4new = curAngles[3] - deltaA6;
      if (a4new >= JOINTS_DEF[3].min && a4new <= JOINTS_DEF[3].max) {
        newAngles[3] = a4new;
      }
      // Feinkorrektur: IK mit festem A6 startend
      const pos = trajectory[tidx].pos;
      const res = solveIK(pos.X, pos.Y, pos.Z, pos.A, pos.B, pos.C, newAngles);
      if (res.ok && Math.abs(res.angles[5] - a6clamped) < 15) {
        // IK-Lösung nahe am gewünschten A6 → übernehmen
        trajectory[tidx].angles = res.angles;
      } else {
        // IK weicht zu stark ab → direkte Winkel verwenden
        trajectory[tidx].angles = newAngles;
        errCount++;
      }
    }

    // KRL-Programm aktualisieren: A-Winkel neu berechnen und eintragen
    const ta = document.getElementById('code-input');
    const lines = ta.value.split(/\r?\n/);

    // Normierungs-Hilfsfunktion
    const nd = v => { let d=v*180/Math.PI; while(d>180)d-=360; while(d<=-180)d+=360; return d; };

    parsedData.positions.forEach((pos, posIdx) => {
      const trajIdx = Math.round(posIdx / Math.max(1,parsedData.positions.length-1) * (trajectory.length-1));
      const tStep = trajectory[Math.min(trajIdx, trajectory.length-1)];
      if (!tStep || pos.lineNum === undefined) return;

      // FK mit neuen Gelenkwinkeln → neue Euler-Winkel A,B,C
      const fk = fkAll(tStep.angles);
      const R = fk.tcp_rot;
      const B2 = -Math.asin(Math.max(-1, Math.min(1, R[2][0])));
      const cb2 = Math.cos(B2);
      let A2, C2;
      if (Math.abs(cb2) > 1e-6) {
        A2 = Math.atan2(R[1][0]/cb2, R[0][0]/cb2);
        C2 = Math.atan2(R[2][1]/cb2, R[2][2]/cb2);
      } else {
        A2 = 0;
        C2 = Math.atan2(-R[1][2], R[1][1]);
      }
      const newA = nd(A2).toFixed(3);
      const newB = nd(B2).toFixed(3);
      const newC = nd(C2).toFixed(3);
      console.log('  → FK A6='+tStep.angles[5].toFixed(1)+' → A='+newA+' B='+newB+' C='+newC);
      console.log('[ampApply] pos', posIdx, 'lineNum:', pos.lineNum,
        'A6_traj:', _t2 ? _t2.angles[5].toFixed(1) : '?',
        'line:', lines[pos.lineNum] ? lines[pos.lineNum].substring(0,60) : 'UNDEFINED');
      if (lines[pos.lineNum]) {
        lines[pos.lineNum] = lines[pos.lineNum].replace(/(\{[^}]+\})/, function(block) {
          // Alle drei Euler-Winkel A, B, C ersetzen
          return block
            .replace(/A\s+([-+]?[0-9]+(?:\.[0-9]+)?)/, 'A ' + newA)
            .replace(/B\s+([-+]?[0-9]+(?:\.[0-9]+)?)/, 'B ' + newB)
            .replace(/C\s+([-+]?[0-9]+(?:\.[0-9]+)?)/, 'C ' + newC);
        });
      }
    });

    var newCode = lines.join('\n');
    ta.value = newCode;
    rebuildGutter();

    // Positionen neu parsen — aber IK NICHT neu berechnen
    parsedData = parseKRL(ta.value);
    // Nur Positions-Karten updaten, keine 3D-Neuberechnung
    renderPositions(parsedData.positions);

    console.log('[ampApply] Written, ta.value now starts:', ta.value.substring(0, 200));
    const msg = errCount > 0
      ? '✓ Pfad übernommen (' + errCount + ' Schritte ohne IK-Konvergenz)'
      : '✓ Pfad übernommen · X/Y/Z-Endpunkte exakt erhalten';
    setStatus('paused', 'A6-Pfad übernommen');
    document.getElementById('amp-info').textContent = msg;
  }
}

// Mouse interaction on canvas — Gummiband: nur Zielpunkte ziehbar
(function() {
  const canvas = document.getElementById('amp-canvas');
  if (!canvas) return;
  const tooltip = document.getElementById('amp-tooltip');

  var dragWptIdx = -1;  // Index des gezogenen Zielpunkts (-1 = keiner)

  function getDeg(e) {
    var r = canvas.getBoundingClientRect();
    var py = e.clientY - r.top;
    var deg = A6_MIN + (1 - py / canvas.height) * (A6_MAX - A6_MIN);
    return Math.max(A6_MIN, Math.min(A6_MAX, deg));
  }

  // Welcher Zielpunkt liegt am nächsten zur Mausposition?
  function nearestWaypoint(e) {
    var r = canvas.getBoundingClientRect();
    var mx = e.clientX - r.left;
    var my = e.clientY - r.top;
    var N = parsedData.positions.length;
    if (N < 1 || !ampUserPath.length) return -1;
    var bestIdx = -1, bestDist = 20; // max 20px Fangradius
    for (var pi = 0; pi < N; pi++) {
      var pxP = (pi / Math.max(1, N-1)) * (canvas.width - 1);
      var col = Math.round(pi / Math.max(1, N-1) * (ampCols-1));
      col = Math.max(0, Math.min(ampCols-1, col));
      var a6 = ampUserPath[col] !== undefined ? ampUserPath[col] : 0;
      var pyP = canvas.height - ((a6 - A6_MIN) / (A6_MAX - A6_MIN)) * canvas.height;
      var dist = Math.sqrt((mx-pxP)*(mx-pxP) + (my-pyP)*(my-pyP));
      if (dist < bestDist) { bestDist = dist; bestIdx = pi; }
    }
    return bestIdx;
  }

  // Gummiband-Interpolation: bewege Punkt pi auf Wert newDeg,
  // interpoliere linear zu den Nachbarn
  function rubberBand(pi, newDeg) {
    var N = parsedData.positions.length;
    if (N < 1) return;
    // Setze diesen Punkt
    var col = Math.round(pi / Math.max(1, N-1) * (ampCols-1));
    col = Math.max(0, Math.min(ampCols-1, col));
    ampUserPath[col] = newDeg;

    // Interpoliere von vorherigem Zielpunkt bis zu diesem
    if (pi > 0) {
      var colPrev = Math.round((pi-1) / Math.max(1, N-1) * (ampCols-1));
      var valPrev = ampUserPath[colPrev];
      var colCurr = col;
      for (var c = colPrev+1; c < colCurr; c++) {
        var f = (c - colPrev) / Math.max(1, colCurr - colPrev);
        ampUserPath[c] = valPrev + (newDeg - valPrev) * f;
      }
    }
    // Interpoliere von diesem bis zum nächsten Zielpunkt
    if (pi < N-1) {
      var colNext = Math.round((pi+1) / Math.max(1, N-1) * (ampCols-1));
      var valNext = ampUserPath[colNext];
      for (var c = col+1; c < colNext; c++) {
        var f = (c - col) / Math.max(1, colNext - col);
        ampUserPath[c] = newDeg + (valNext - newDeg) * f;
      }
    }
  }

  function onDragStart(e) {
    if (!ampCols) return;
    var ee = e.touches ? e.touches[0] : e;
    var idx = nearestWaypoint(ee);
    if (idx >= 0) {
      dragWptIdx = idx;
      ampDragging = true;
      rubberBand(idx, getDeg(ee));
      ampDraw(canvas, canvas.width, canvas.height);
      e.preventDefault();
    }
  }
  canvas.addEventListener('mousedown', onDragStart);
  canvas.addEventListener('touchstart', onDragStart, {passive:false});

  function onDragMove(e) {
    if (!ampCols) return;
    var ee = e.touches ? e.touches[0] : e;
    // Tooltip
    var r = canvas.getBoundingClientRect();
    var px = ee.clientX - r.left;
    var col = Math.round(px / (canvas.width-1) * (ampCols-1));
    col = Math.max(0, Math.min(ampCols-1, col));
    var deg = getDeg(e);
    var tidx = Math.round(col / (ampCols-1) * (trajectory.length-1));
    var sample = trajectory[tidx];
    var curVal = (sample&&sample.angles&&sample.angles[5]!==undefined) ? sample.angles[5].toFixed(1) : '—';
    var rowIdx = Math.round((deg - A6_MIN) / (A6_MAX - A6_MIN) * (ampRows-1));
    rowIdx = Math.max(0, Math.min(ampRows-1, rowIdx));
    var status = ampMap ? (ampMap[col * ampRows + rowIdx]||0) : 0;
    var statusLbl = ['✓ Gültig','⚠ Endschalter','⚠ Singularität','✗ Nicht erreichbar'][status] || '';
    tooltip.style.display = 'block';
    tooltip.style.left = (e.offsetX+10)+'px';
    tooltip.style.top  = (e.offsetY-28)+'px';

    // Cursor: Zeiger wenn nahe an Zielpunkt
    var near = nearestWaypoint(ee);
    canvas.style.cursor = near >= 0 ? 'ns-resize' : 'default';

    if (near >= 0 && !ampDragging) {
      tooltip.textContent = '#' + (near+1) + ' ziehen · A6 aktuell: ' + curVal + '° · ' + statusLbl;
    } else {
      tooltip.textContent = 'Schritt ' + (col+1) + '/' + ampCols + ' · A6=' + deg.toFixed(1) + '° · ' + statusLbl;
    }

    if (!ampDragging || dragWptIdx < 0) return;
    var newDeg = getDeg(ee);
    e.preventDefault();
    rubberBand(dragWptIdx, newDeg);
    ampDraw(canvas, canvas.width, canvas.height);

    // Roboter live bewegen: vollständige IK mit neuer Orientierung
    // TCP-Position bleibt exakt auf dem Pfad — nur Rz (A6) ändert sich
    var pi2 = dragWptIdx;
    var N2  = parsedData.positions.length;
    var refTraj2 = trajectoryRef.length ? trajectoryRef : trajectory;
    var tidxD2 = Math.round(pi2 / Math.max(1,N2-1) * (refTraj2.length-1));
    tidxD2 = Math.max(0, Math.min(refTraj2.length-1, tidxD2));
    var entryD2 = refTraj2[tidxD2];
    if (entryD2) {
      var angD2 = entryD2.angles;
      var posD2 = entryD2.pos;
      var pX2 = posD2.X !== undefined ? posD2.X : (posD2[0]||0);
      var pY2 = posD2.Y !== undefined ? posD2.Y : (posD2[1]||0);
      var pZ2 = posD2.Z !== undefined ? posD2.Z : (posD2[2]||0);
      var pA2 = posD2.A !== undefined ? posD2.A : (posD2[3]||0);
      var pB2 = posD2.B !== undefined ? posD2.B : (posD2[4]||0);
      var pC2 = posD2.C !== undefined ? posD2.C : (posD2[5]||0);
      // Rz_new = Rz_ref + (A6_new - A6_ref) — nur erster Euler-Winkel ändert sich
      var a6RefD = angD2[5];
      var AnewD2 = pA2 + (newDeg - a6RefD);
      while(AnewD2 >  180) AnewD2 -= 360;
      while(AnewD2 <= -180) AnewD2 += 360;
      // Vollständige IK: TCP-Position bleibt, nur Orientierung ändert sich
      // Warm-Start: aktuelle Winkel des Trajektorie-Punkts
      var resD = solveIKFast(pX2, pY2, pZ2, AnewD2, pB2, pC2, angD2);
      if (resD.ok) {
        applyAngles(resD.angles);
      } else {
        // Fallback: Trajektorie-Winkel mit A6 gesetzt
        var fallback = angD2.slice();
        fallback[5] = newDeg;
        applyAngles(fallback);
      }
    }
  }
  canvas.addEventListener('mousemove', onDragMove);
  canvas.addEventListener('touchmove', onDragMove, {passive:false});

  function onDragEnd() {
    ampDragging = false;
    dragWptIdx = -1;
    canvas.style.cursor = 'default';
  }
  canvas.addEventListener('mouseup', onDragEnd);
  canvas.addEventListener('touchend', onDragEnd);
  canvas.addEventListener('mouseleave', function() {
    onDragEnd();
    if (tooltip) tooltip.style.display = 'none';
  });
})();

// Rebuild map when trajectory is rebuilt

function toggleSteuerPanel(){
  const p=document.getElementById('steuer-panel');
  p.classList.toggle('visible');
  document.getElementById('btn-steuer').classList.toggle('on', p.classList.contains('visible'));
  if(p.classList.contains('visible')) updateSteuerPanel();
}

function buildSteuerAxes(){
  const el=document.getElementById('sp-axes');
  if(!el) return;
  el.innerHTML=JOINTS_DEF.map((j,i)=>`
    <div class="sp-axis-row">
      <span class="sp-axis-lbl">A${i+1}:</span>
      <span class="sp-axis-min">${j.min}</span>
      <div class="sp-slider-wrap" id="sp-wrap${i}" style="cursor:ew-resize">
        <div class="sp-slider-fill" id="sp-fill${i}"></div>
        <div class="sp-slider-thumb" id="sp-thumb${i}"></div>
      </div>
      <span class="sp-axis-max">${j.max}</span>
      <input class="sp-axis-inp" id="sp-av${i}" type="number"
        value="0" step="0.1" min="${j.min}" max="${j.max}"
        onchange="spAxisChanged(${i},this.value)"
        onkeydown="if(event.key==='Enter')spAxisChanged(${i},this.value)">
    </div>`).join('');

  // Make sliders draggable
  JOINTS_DEF.forEach((j,i)=>{
    const wrap=document.getElementById('sp-wrap'+i);
    if(!wrap) return;
    let dragging=false;
    wrap.addEventListener('mousedown',e=>{
      e.preventDefault(); dragging=true;
      document.addEventListener('mousemove',onDrag);
      document.addEventListener('mouseup',()=>{dragging=false;document.removeEventListener('mousemove',onDrag);},{once:true});
      function onDrag(ev){
        if(!dragging) return;
        const r=wrap.getBoundingClientRect();
        const pct=Math.max(0,Math.min(1,(ev.clientX-r.left)/r.width));
        const val=j.min+(j.max-j.min)*pct;
        spAxisChanged(i, val.toFixed(2));
      }
      onDrag(e);
    });
    // Also make TCP values editable on click
  });

  // Make TCP display values editable (click to edit) - no arguments.callee
  function makeTCPEditable(id){
    const el=document.getElementById(id);
    if(!el) return;
    el.style.cursor='text';
    el.title='Klicken zum Bearbeiten';
    el.onclick=function(){
      const cur=this.textContent;
      const parent=this.parentNode;
      const inp=document.createElement('input');
      inp.type='number'; inp.value=parseFloat(cur)||0; inp.step='0.1';
      inp.style.cssText='width:100%;background:#070d1a;color:#9ecfea;border:1px solid var(--acc);border-radius:2px;font-family:inherit;font-size:inherit;padding:0 2px;outline:none;text-align:right';
      this.replaceWith(inp); inp.focus(); inp.select();
      function commit(){
        const span=document.createElement('span');
        span.id=id; span.className='sp-tcp-val';
        span.textContent=parseFloat(inp.value||0).toFixed(3);
        inp.replaceWith(span);
        makeTCPEditable(id);
        spTCPChanged();
      }
      inp.addEventListener('blur',commit);
      inp.addEventListener('keydown',e=>{
        if(e.key==='Enter'){e.preventDefault();inp.blur();}
        if(e.key==='Escape'){inp.value=parseFloat(cur)||0;inp.blur();}
      });
    };
  }
  ['sp-x','sp-y','sp-z','sp-a','sp-b','sp-c'].forEach(makeTCPEditable);
}

function spAxisChanged(i, val){
  const v=Math.max(JOINTS_DEF[i].min, Math.min(JOINTS_DEF[i].max, parseFloat(val)||0));
  const newAngles=jointAngles.slice();
  newAngles[i]=v;
  applyAngles(newAngles);
  document.getElementById('jog-status').textContent='A'+(i+1)+': '+v.toFixed(2)+'°';
}

function spTCPChanged(){
  const g=id=>{const el=document.getElementById(id);return parseFloat((el&&el.textContent)||(el&&el.value))||0;};
  const x=g('sp-x'),y=g('sp-y'),z=g('sp-z'),a=g('sp-a'),b=g('sp-b'),cc=g('sp-c');
  const res=solveIK(x,y,z,a,b,cc);
  if(res.ok){
    applyAngles(res.angles);
    document.getElementById('jog-status').textContent='IK OK  Δ'+res.score.toFixed(2)+'mm';
  } else {
    document.getElementById('jog-status').textContent='IK fehlgeschlagen  Δ'+res.score.toFixed(1)+'mm';
  }
}

function updateSteuerPanel(){
  const p=document.getElementById('steuer-panel');
  if(!(p&&p.classList).contains('visible')) return;
  // Axes display
  JOINTS_DEF.forEach((j,i)=>{
    const ang=jointAngles[i];
    const pct=Math.max(0,Math.min(100,(ang-j.min)/(j.max-j.min)*100));
    const fill=document.getElementById('sp-fill'+i);
    const thumb=document.getElementById('sp-thumb'+i);
    const inp=document.getElementById('sp-av'+i);
    if(fill) fill.style.width=pct+'%';
    if(thumb) thumb.style.left=pct+'%';
    if(inp&&document.activeElement!==inp) inp.value=ang.toFixed(2);
  });
  // TCP display + jog inputs
  const fk=fkAll(jointAngles);
  const tcp=fk.pts[7];
  if(tcp){
    const f=v=>v.toFixed(3);
    const setV=(id,v)=>{const el=document.getElementById(id);if(el)el.textContent=v;};
    setV('sp-x',f(tcp[0]));setV('sp-y',f(tcp[1]));setV('sp-z',f(tcp[2]));
    const R=fk.tcp_rot;
    const B=-Math.asin(Math.max(-1,Math.min(1,R[2][0])));
    const cb=Math.cos(B);
    let A,C;
    if(Math.abs(cb)>1e-6){A=Math.atan2(R[1][0]/cb,R[0][0]/cb);C=Math.atan2(R[2][1]/cb,R[2][2]/cb);}
    else{A=0;C=Math.atan2(-R[1][2],R[1][1]);}
    const nd=v=>{let d=v*180/Math.PI;while(d>180)d-=360;while(d<=-180)d+=360;return d;}
    const ndS=v=>nd(v).toFixed(3);
    setV('sp-a',ndS(A));setV('sp-b',ndS(B));setV('sp-c',ndS(C));

  }
}

// ── Insert PTP (axis angles) into program ──
function insertPTP(){
  const a=jointAngles;
  const velMmMin=parseFloat((document.getElementById('jog-feed')&&document.getElementById('jog-feed').value))||250;
  const krl=`PTP {A1 ${a[0].toFixed(3)}, A2 ${a[1].toFixed(3)}, A3 ${a[2].toFixed(3)}, A4 ${a[3].toFixed(3)}, A5 ${a[4].toFixed(3)}, A6 ${a[5].toFixed(3)}}`;
  appendToProgram(krl, velMmMin);
  document.getElementById('jog-status').textContent='✓ PTP eingefügt';
}

function insertLIN(){
  const fk=fkAll(jointAngles); const tcp=fk.pts[7];
  const R=fk.tcp_rot;
  const B=-Math.asin(Math.max(-1,Math.min(1,R[2][0])));
  const cb=Math.cos(B);
  let A=0,C=0;
  if(Math.abs(cb)>1e-6){A=Math.atan2(R[1][0]/cb,R[0][0]/cb);C=Math.atan2(R[2][1]/cb,R[2][2]/cb);}
  else{C=Math.atan2(-R[1][2],R[1][1]);}
  const nd=v=>{let d=v*180/Math.PI;while(d>180)d-=360;while(d<=-180)d+=360;return d;}
  const vel=parseFloat((document.getElementById('jog-feed')&&document.getElementById('jog-feed').value))||250; // mm/min
  const krl=`LIN {X ${tcp[0].toFixed(3)}, Y ${tcp[1].toFixed(3)}, Z ${tcp[2].toFixed(3)}, A ${nd(A).toFixed(3)}, B ${nd(B).toFixed(3)}, C ${nd(C).toFixed(3)}}`;
  appendToProgram(krl, vel);
  document.getElementById('jog-status').textContent='✓ LIN eingefügt';
}

// ── Append line to KRL editor (before END) ──
// vel: mm/s for LIN ($VEL.CP), null for PTP
// velPct: % for PTP (BAS #VEL_PTP), undefined for LIN
function appendToProgram(krlLine, vel){
  // vel in mm/min → m/s = vel / 60000
  const ta=document.getElementById('code-input');
  let code=ta.value;
  const lines=code.split(/\r?\n/);
  let endIdx=lines.length-1;
  for(let i=lines.length-1;i>=0;i--){
    if(/^\s*END\s*$/i.test(lines[i])){endIdx=i;break;}
  }
  const insLines=[];
  if(vel!==null && vel!==undefined){
    const mPerSec = vel / 60000;
    insLines.push(`$VEL.CP=${mPerSec.toFixed(4)}`);
  }
  insLines.push(krlLine);
  lines.splice(endIdx,0,...insLines);
  ta.value=lines.join('\n');
  rebuildGutter();
}

// Drag steuerungspanel
(function(){
  let dragging=false,ox=0,oy=0;
  const hdr=document.getElementById('steuer-hdr');
  const panel=document.getElementById('steuer-panel');
  if(!hdr||!panel) return;
  hdr.addEventListener('mousedown',e=>{if(e.button!==0)return;dragging=true;ox=e.clientX-panel.offsetLeft;oy=e.clientY-panel.offsetTop;e.preventDefault();});
  window.addEventListener('mousemove',e=>{if(!dragging)return;panel.style.left=(e.clientX-ox)+'px';panel.style.top=(e.clientY-oy)+'px';});
  window.addEventListener('mouseup',()=>dragging=false);
})();

function toggleGrid(){
  grid.visible = !grid.visible;
  document.getElementById('btn-grid').classList.toggle('on', grid.visible);
}

document.getElementById('btn-bg').addEventListener('click', function(){
  document.body.classList.toggle('bg-white');
  const isWhite = document.body.classList.contains('bg-white');
  scene.background=new THREE.Color(isWhite?0xf0f0eb:0x070d1a);
  if(isWhite){
    setGridColor(0xbbbbaa);
  } else {
    setGridColor(0x0e1e30);
  }
});
document.getElementById('btn-tcp-trace').addEventListener('click', function(){
  showTCPTrace=!showTCPTrace;this.classList.toggle('on',showTCPTrace);
  document.getElementById('cfg-show-trace').checked=showTCPTrace;
  if(!showTCPTrace)tcpTraceGrp.clear();
});

// ═══════════════════════════════════════════════════
// IK TABLE — pre-computed for each program position
// ═══════════════════════════════════════════════════
let ikTable = []; // [{angles, ok, score}] per position


// Schneller IK für Kandidaten-Generierung (weniger Iterationen)
function solveIKFast(tx, ty, tz, ta, tb, tc, initAngles) {
  var clamp = function(v,lo,hi){ return Math.max(lo,Math.min(hi,v)); };
  var tp = [tx, ty, tz];
  var Rt = rotZYX(ta, tb, tc);
  var dt = 0.4, lam = 0.8, tolP = 1.0, tolO = 1.0;
  var q = initAngles ? initAngles.slice() : jointAngles.slice();
  var bestScore = Infinity, bestQ = q.slice();
  for (var iter = 0; iter < 80; iter++) {
    var e = err6(q, tp, Rt);
    var eP = Math.sqrt(e[0]*e[0]+e[1]*e[1]+e[2]*e[2]);
    var eO = Math.sqrt(e[3]*e[3]+e[4]*e[4]+e[5]*e[5]);
    var score = eP + eO;
    if (score < bestScore) { bestScore = score; bestQ = q.slice(); }
    if (eP < tolP && eO < tolO) break;
    var J = [];
    for (var i = 0; i < 6; i++) {
      var q1 = q.slice(); q1[i] += dt;
      var e1 = err6(q1, tp, Rt);
      J.push([(e1[0]-e[0])/dt,(e1[1]-e[1])/dt,(e1[2]-e[2])/dt,
               (e1[3]-e[3])/dt,(e1[4]-e[4])/dt,(e1[5]-e[5])/dt]);
    }
    var JtJ = Array.from({length:6}, function(){ return Array(6).fill(0); });
    var Jte = Array(6).fill(0);
    for (var i = 0; i < 6; i++) {
      for (var r = 0; r < 6; r++) {
        Jte[i] += J[i][r] * e[r];
        for (var j = 0; j < 6; j++) JtJ[i][j] += J[i][r] * J[j][r];
      }
      JtJ[i][i] += lam;
    }
    var dq = solve6x6(JtJ, Jte);
    var step = Math.min(2.0, 8.0 / Math.max(1, bestScore));
    for (var i = 0; i < 6; i++) {
      if (!isFinite(dq[i])) continue;
      q[i] = clamp(q[i] - step*dq[i], JOINTS_DEF[i].min, JOINTS_DEF[i].max);
    }
  }
  return { angles: bestQ, score: bestScore, ok: bestScore < 25 };
}


// ── Jacobi-Singularitätserkennung (PDF Punkt 2) ───────────────
// Berechne numerischen Jacobi J(6x6), dann kleinsten Singulärwert via
// J^T * J Eigenwert-Approximation (Frobenius-Norm Vereinfachung)
function computeManipulability(angles_deg) {
  var dt = 0.5;  // 0.5° Perturbation
  var e0 = fkTCP_full(angles_deg);
  var J = [];
  for (var i = 0; i < 6; i++) {
    var q1 = angles_deg.slice();
    q1[i] += dt;
    var e1 = fkTCP_full(q1);
    // Positions-Spalte
    var col = [
      (e1.pos[0] - e0.pos[0]) / dt,
      (e1.pos[1] - e0.pos[1]) / dt,
      (e1.pos[2] - e0.pos[2]) / dt,
      // Orientierungs-Ableitung (vereinfacht)
      (e1.rot[0][0] - e0.rot[0][0]) / dt,
      (e1.rot[1][0] - e0.rot[1][0]) / dt,
      (e1.rot[2][0] - e0.rot[2][0]) / dt,
    ];
    J.push(col);
  }
  // Kleinsten Singulärwert approximieren via kleinsten Eigenwert von J^T*J
  // Schneller Proxy: det(J^T*J) ≈ Produkt der Diagonale von J^T*J
  // Für Praxis: Konditionszahl = max/min Diagonalelement
  var JtJ_diag = Array(6).fill(0);
  for (var i = 0; i < 6; i++) {
    for (var r = 0; r < 6; r++) {
      JtJ_diag[i] += J[i][r] * J[i][r];
    }
  }
  var minEig = Math.min.apply(null, JtJ_diag);
  var maxEig = Math.max.apply(null, JtJ_diag);
  // Manipulierbarkeit: kleiner Wert = Singularität näher
  return { manipulability: minEig, condition: maxEig / Math.max(minEig, 1e-9) };
}

var SING_MANIP_THRESH = 0.001;   // Unter diesem Wert = Singularität
var SING_COND_THRESH  = 500;     // Konditionszahl > Schwellwert = Singularität

function isSingular(angles_deg) {
  // Schneller A5-Check als Vorfilter (A5 nahe 0° = Wrist-Singularität garantiert)
  if (Math.abs(angles_deg[4]) < 8) return true;
  // Jacobi-basierte Prüfung
  var m = computeManipulability(angles_deg);
  return (m.manipulability < SING_MANIP_THRESH || m.condition > SING_COND_THRESH);
}

// ── Globaler DP-Planer (nach Redundanzoptimierung-PDF) ────────
// Kostenfunktion: A4/A5 teuer, A6 kontinuierlich, Limits/Singularitäten bestrafen
function ikCost(from, to) {
  if (!from || !to) return 1e9;
  var W = [1, 1, 1, 8, 8, 1];  // A4/A5 hoch gewichten
  var cost = 0;
  for (var i = 0; i < 6; i++) {
    var d = shortestAngleDiff(from[i], to[i]);
    cost += W[i] * Math.abs(d);
  }
  // Limit-Penalty: Strafe bei Nähe zu Achsgrenzen
  for (var j = 0; j < 6; j++) {
    var lim = JOINTS_DEF[j];
    var margin = Math.min(to[j] - lim.min, lim.max - to[j]);
    if (margin < 10) cost += (10 - margin) * 5;
  }
  // Singularitäts-Penalty: A5 nahe 0
  var a5 = Math.abs(to[4]);
  if (a5 < 15) cost += (15 - a5) * 10;
  return cost;
}

// Generiert mehrere IK-Kandidaten pro Position (A6 periodisch gekachelt)
function ikCandidates(pos, prevAngles) {
  var cands = [];
  // A6 aus Planlinie als primären Startwert verwenden
  var a6Plan = prevAngles[5];
  if (ampUserPath.length > 0 && parsedData.positions.length > 0) {
    var posIdx_hint = parsedData.positions.indexOf(pos);
    if (posIdx_hint < 0) posIdx_hint = 0;
    var planCol = Math.round(posIdx_hint / Math.max(1, parsedData.positions.length-1) * (ampCols-1));
    planCol = Math.max(0, Math.min(ampCols-1, planCol));
    if (ampUserPath[planCol] !== undefined) a6Plan = ampUserPath[planCol];
  }
  // Nur 2 A6-Kacheln: aktuell + benachbarte 360°-Kopie
  var planStart = prevAngles.slice(); planStart[5] = a6Plan;
  var planStart360 = prevAngles.slice(); planStart360[5] = a6Plan + 360;
  var planStartN360 = prevAngles.slice(); planStartN360[5] = a6Plan - 360;
  var a6Offsets = [0];  // Kachelung nur via Starts, nicht als Loop
  var baseStarts = [
    planStart,      // Plan-A6: wichtigster Kandidat
    prevAngles,     // Kontinuität vom Vorgänger
    planStart360,   // A6 + 360
    planStartN360,  // A6 - 360
  ];
  var seen = [];
  for (var oi = 0; oi < a6Offsets.length; oi++) {
    for (var si = 0; si < baseStarts.length; si++) {
      var start = baseStarts[si].slice();
      start[5] += a6Offsets[oi];  // A6 kacheln
      var res = solveIKFast(pos.X, pos.Y, pos.Z, pos.A, pos.B, pos.C, start);
      if (!res.ok) continue;
      // Normalisiere auf kürzesten Weg vom Vorgänger
      var ang = res.angles.map(function(v, i) {
        return prevAngles[i] + shortestAngleDiff(prevAngles[i], v);
      });
      // Duplikat-Check (0.5° Toleranz pro Achse)
      var isDup = seen.some(function(s) {
        return s.every(function(v, i) { return Math.abs(v - ang[i]) < 0.5; });
      });
      if (isDup) continue;
      // Harte Grenzen prüfen (mit 5° Sicherheitsabstand)
      var inLimits = ang.every(function(v, i) {
        return v >= JOINTS_DEF[i].min + 5 && v <= JOINTS_DEF[i].max - 5;
      });
      if (!inLimits) continue;
      seen.push(ang);
      cands.push({ angles: ang, score: res.score });
    }
  }
  return cands;
}

function computeIKTable(positions) {
  ikTable = [];
  var N = positions.length;
  if (!N) { buildTrajectory(positions, ikTable); return; }

  // ── DPSolver ─────────────────────────────────────────────
  // Schwellwert: bei > 150 Punkten → direkte IK mit Warm-Start (Performance)
  var DP_MAX_POINTS = 150;
  if (N > DP_MAX_POINTS) {
    console.log('[Perf] ' + N + ' Punkte → DPSolver deaktiviert, verwende Warm-Start IK');
    splashProgress && splashProgress(50, N + ' Punkte — Schnell-IK wird berechnet…');
    var prevQ = jointAngles.slice();
    if (parsedData.steps) {
      for (var _si=0; _si<parsedData.steps.length; _si++) {
        if (parsedData.steps[_si].type==='ptpAxis'&&parsedData.steps[_si].angles) { prevQ=parsedData.steps[_si].angles.slice(); break; }
        if (parsedData.steps[_si].type==='move') break;
      }
    }
    for (var pi=0; pi<N; pi++) {
      var res = solveIKFast(positions[pi].X, positions[pi].Y, positions[pi].Z,
                            positions[pi].A, positions[pi].B, positions[pi].C, prevQ);
      var angles = res.ok ? res.angles : prevQ;
      // Normalize to shortest path
      angles = angles.map(function(v,j){ return prevQ[j] + shortestAngleDiff(prevQ[j], v); });
      ikTable.push({ angles: angles, score: res.score, ok: res.ok });
      prevQ = angles.slice();
    }
    buildTrajectory(positions, ikTable);
    return;
  }
  try {
    // Arc-length s for each position
    var arcS = [0];
    for (var pi = 1; pi < N; pi++) {
      var p0=positions[pi-1], p1=positions[pi];
      var dx=p1.X-p0.X, dy=p1.Y-p0.Y, dz=p1.Z-p0.Z;
      arcS.push(arcS[pi-1]+Math.sqrt(dx*dx+dy*dy+dz*dz));
    }
    var targetPts = positions.map(function(p,i){ return {s:arcS[i],X:p.X,Y:p.Y,Z:p.Z,A:p.A,B:p.B,C:p.C}; });
    // qStart: letzter PTP-Achswinkel vor den LIN-Positionen
    var qStart = jointAngles.slice();
    if (parsedData.steps) {
      for (var si = 0; si < parsedData.steps.length; si++) {
        var step = parsedData.steps[si];
        if (step.type === 'ptpAxis' && step.angles) {
          qStart = step.angles.slice();  // letzten PTP merken
        }
        if (step.type === 'move') break; // Erster LIN → stopp
      }
    }
    var result = DPSolver.plan(targetPts, qStart);

    // ikTable aus DP raw path (Konfigurationswahl)
    for (var pi2 = 0; pi2 < N; pi2++) {
      var rp = result.rawPath[pi2] || result.rawPath[result.rawPath.length-1];
      ikTable.push({ angles: rp.q.slice(), ok: true, score: 0 });
    }

    // Trajektorie: lineare Interpolation im kartesischen Raum (LIN-Semantik!)
    // Gelenkwinkel werden zwischen den DP-Konfigurationen interpoliert
    buildTrajectory(positions, ikTable);
    return;

  } catch(e) {
    console.warn('DPSolver Fehler, Fallback:', e.message);
  }

  // ── Fallback: alter DP-Planer ─────────────────────────────

  // Schritt 1: Kandidaten für jede Position generieren
  var allCands = [];
  var prevAngles = jointAngles.slice();
  for (var i = 0; i < N; i++) {
    var cands = ikCandidates(positions[i], prevAngles);
    if (!cands.length) {
      // Fallback: normaler Solver
      var res = solveIK(positions[i].X, positions[i].Y, positions[i].Z,
                        positions[i].A, positions[i].B, positions[i].C, prevAngles);
      cands = [{ angles: res.angles, score: res.score }];
    }
    allCands.push(cands);
    if (cands.length) prevAngles = cands[0].angles.slice();
  }

  // Schritt 2: Dynamic Programming — globaler Pfad
  var INF = 1e12;
  // dp[i][j] = { cost, prevJ }
  var dp = allCands.map(function(c) {
    return c.map(function() { return { cost: INF, prevJ: -1 }; });
  });

  // Init: erste Position kostet 0
  for (var j0 = 0; j0 < allCands[0].length; j0++) {
    dp[0][j0].cost = ikCost(jointAngles, allCands[0][j0].angles);
  }

  // Forward pass
  for (var i = 1; i < N; i++) {
    for (var j = 0; j < allCands[i].length; j++) {
      for (var k = 0; k < allCands[i-1].length; k++) {
        var c = dp[i-1][k].cost + ikCost(allCands[i-1][k].angles, allCands[i][j].angles);
        if (c < dp[i][j].cost) {
          dp[i][j].cost = c;
          dp[i][j].prevJ = k;
        }
      }
    }
  }

  // Backward pass: besten Endpunkt finden
  var bestJ = 0, bestCost = INF;
  for (var j = 0; j < allCands[N-1].length; j++) {
    if (dp[N-1][j].cost < bestCost) {
      bestCost = dp[N-1][j].cost;
      bestJ = j;
    }
  }

  // Pfad zurückverfolgen
  var path = new Array(N);
  var cur = bestJ;
  for (var i = N-1; i >= 0; i--) {
    path[i] = cur;
    cur = dp[i][cur].prevJ;
    if (cur < 0) cur = 0;
  }

  // IK-Tabelle aus DP-Pfad aufbauen
  for (var i = 0; i < N; i++) {
    var cand = allCands[i][path[i]];
    ikTable.push({ angles: cand.angles, score: cand.score, ok: cand.score < 20 });
  }

  // Build trajectory after IK table is ready
  buildTrajectory(positions, ikTable);
  // Rebuild axis map if open
  const amp = document.getElementById('axis-map-panel');
  if ((amp&&amp.classList).contains('visible')) setTimeout(ampBuild, 100);
}

// ═══════════════════════════════════════════════════
// CAMERA / VIEW MANAGEMENT
// ═══════════════════════════════════════════════════
const orbitTarget=new THREE.Vector3(500,0,600);
const orbitState={theta:-0.7,phi:1.05,radius:3500};

function updateCamera(){
  if(activeCam===perspCam){
    const{theta,phi,radius}=orbitState;
    perspCam.position.set(orbitTarget.x+radius*Math.sin(phi)*Math.cos(theta),orbitTarget.y+radius*Math.sin(phi)*Math.sin(theta),orbitTarget.z+radius*Math.cos(phi));
    perspCam.lookAt(orbitTarget);
  }else{
    const vp=canvas.parentElement;const asp=vp.clientWidth/vp.clientHeight;
    orthoCam.left=-orthoHalfSize*asp;orthoCam.right=orthoHalfSize*asp;
    orthoCam.top=orthoHalfSize;orthoCam.bottom=-orthoHalfSize;orthoCam.updateProjectionMatrix();
    const D=40000,t=orbitTarget;
    switch(currentView){
      case'top':orthoCam.position.set(t.x,t.y,t.z+D);orthoCam.up.set(0,1,0);break;
      case'bottom':orthoCam.position.set(t.x,t.y,t.z-D);orthoCam.up.set(0,-1,0);break;
      case'front':orthoCam.position.set(t.x,t.y-D,t.z);orthoCam.up.set(0,0,1);break;
      case'back':orthoCam.position.set(t.x,t.y+D,t.z);orthoCam.up.set(0,0,1);break;
      case'left':orthoCam.position.set(t.x-D,t.y,t.z);orthoCam.up.set(0,0,1);break;
      case'right':orthoCam.position.set(t.x+D,t.y,t.z);orthoCam.up.set(0,0,1);break;
    }
    orthoCam.lookAt(t);
  }
}
updateCamera();

function setView(view){
  currentView=view;
  document.querySelectorAll('.vbtn').forEach(b=>b.classList.remove('on'));
  (function(){var _e=document.getElementById('vb-'+view);if(_e)_e.classList.add('on');})();
  activeCam=(view==='iso')?perspCam:orthoCam;
  // sync projection button label
  const btn=document.getElementById('btn-persp');
  if(btn){
    if(activeCam===perspCam){btn.textContent='⊡ Perspektiv';btn.classList.add('on');}
    else{btn.textContent='⊞ Ortho';btn.classList.remove('on');}
  }
  updateCamera();resize();
}

function resize(){
  ampAlignToViewport();
  const vp=canvas.parentElement;const w=vp.clientWidth,h=vp.clientHeight;
  renderer.setSize(w,h);perspCam.aspect=w/h;perspCam.updateProjectionMatrix();
  const asp=w/h;orthoCam.left=-orthoHalfSize*asp;orthoCam.right=orthoHalfSize*asp;
  orthoCam.top=orthoHalfSize;orthoCam.bottom=-orthoHalfSize;orthoCam.updateProjectionMatrix();
}
window.addEventListener('resize',resize);resize();

// ═══════════════════════════════════════════════════
// MOUSE / ORBIT / DRAG
// ═══════════════════════════════════════════════════
const raycaster=new THREE.Raycaster();
let drag=null,dragPos=null,dragMoved=false;

function getNDC(e){const r=canvas.getBoundingClientRect();return new THREE.Vector2((e.clientX-r.left)/r.width*2-1,-((e.clientY-r.top)/r.height)*2+1);}
function getWorldOnZPlane(e,z){const ndc=getNDC(e);raycaster.setFromCamera(ndc,activeCam);const plane=new THREE.Plane(new THREE.Vector3(0,0,1),-z);const hit=new THREE.Vector3();raycaster.ray.intersectPlane(plane,hit);return hit;}

function raycastPositions(e){
  const ndc=getNDC(e);raycaster.setFromCamera(ndc,activeCam);
  const hits=raycaster.intersectObjects(posGrp.children,true);
  if(!hits.length)return null;
  let grp=hits[0].object;
  while(grp.parent&&grp.parent!==posGrp)grp=grp.parent;
  return grp.userData.posIdx!==undefined?grp.userData.posIdx:null;
}

canvas.addEventListener('mousedown',e=>{
  if(e.button!==0&&e.button!==1&&e.button!==2)return;
  dragMoved=false;e.preventDefault();
  if(e.button===0){
    const hit=raycastPositions(e);
    if(hit!==null&&!sim.playing){
      selectPosition(hit);
      const pos=parsedData.positions[hit];
      const sw=getWorldOnZPlane(e,pos.Z);
      dragPos={idx:hit,origPos:{...pos},startWorld:sw,mode:currentDragMode};
      canvas.classList.add('dragging');return;
    }
    drag={btn:activeCam===perspCam?2:1,lx:e.clientX,ly:e.clientY};
  }else if(e.button===1){drag={btn:1,lx:e.clientX,ly:e.clientY};}
  else{drag={btn:2,lx:e.clientX,ly:e.clientY};}
  canvas.classList.add('dragging');
});

window.addEventListener('mousemove',e=>{
  if(dragPos){
    dragMoved=true;
    const pos=dragPos.origPos;
    if(dragPos.mode==='tz'){
      const ndc=getNDC(e);raycaster.setFromCamera(ndc,activeCam);
      const cRight=new THREE.Vector3().crossVectors(activeCam.getWorldDirection(new THREE.Vector3()),activeCam.up).normalize();
      const plane=new THREE.Plane().setFromNormalAndCoplanarPoint(cRight,dragPos.startWorld);
      const hit=new THREE.Vector3();raycaster.ray.intersectPlane(plane,hit);
      if(hit){const dz=hit.z-dragPos.startWorld.z;applyDraggedPos(dragPos.idx,{...parsedData.positions[dragPos.idx],X:pos.X,Y:pos.Y,Z:pos.Z+dz},false);}
    }else{
      const hit=getWorldOnZPlane(e,pos.Z);
      if(hit){const dx=hit.x-dragPos.startWorld.x,dy=hit.y-dragPos.startWorld.y;applyDraggedPos(dragPos.idx,{...parsedData.positions[dragPos.idx],X:pos.X+dx,Y:pos.Y+dy,Z:pos.Z},false);}
    }
    return;
  }
  if(!drag)return;
  const dx=e.clientX-drag.lx,dy=e.clientY-drag.ly;
  drag.lx=e.clientX;drag.ly=e.clientY;
  if(Math.abs(dx)+Math.abs(dy)>2)dragMoved=true;
  if(drag.btn===2&&activeCam===perspCam){orbitState.theta-=dx*.007;orbitState.phi=Math.max(.04,Math.min(Math.PI-.04,orbitState.phi+dy*.007));updateCamera();}
  else if(drag.btn===1||drag.btn===2){
    if(activeCam===perspCam){const sc=orbitState.radius*.0008;const right=new THREE.Vector3().crossVectors(perspCam.getWorldDirection(new THREE.Vector3()),perspCam.up).normalize();orbitTarget.addScaledVector(right,-dx*sc);orbitTarget.z+=dy*sc;}
    else{const vp=canvas.parentElement;const sc=orthoHalfSize*2/vp.clientHeight;const right=new THREE.Vector3().crossVectors(orthoCam.getWorldDirection(new THREE.Vector3()),orthoCam.up).normalize();orbitTarget.addScaledVector(right,-dx*sc);orbitTarget.addScaledVector(orthoCam.up,dy*sc);}
    updateCamera();
  }
});

window.addEventListener('mouseup',e=>{
  if(dragPos){if(dragMoved)syncPositionToCode(dragPos.idx);dragPos=null;canvas.classList.remove('dragging');return;}
  if(drag){drag=null;canvas.classList.remove('dragging');if(!dragMoved&&e.button===0){const hit=raycastPositions(e);if(hit===null)deselectPosition();}}
});
canvas.addEventListener('contextmenu',e=>e.preventDefault());
canvas.addEventListener('wheel',e=>{
  if(activeCam===perspCam)orbitState.radius=Math.max(80,Math.min(40000,orbitState.radius*(1+e.deltaY*.001)));
  else orthoHalfSize=Math.max(50,Math.min(20000,orthoHalfSize*(1+e.deltaY*.001)));
  updateCamera();
},{passive:true});

// ═══════════════════════════════════════════════════
// POSITION SELECTION & EDITING
// ═══════════════════════════════════════════════════
let selectedPosIdx=null,currentDragMode='translate';

function setDragMode(mode){currentDragMode=mode;document.getElementById('ep-translate').classList.toggle('active',mode==='translate');document.getElementById('ep-tz').classList.toggle('active',mode==='tz');}

function selectPosition(idx){
  selectedPosIdx=idx;const pos=parsedData.positions[idx];
  selSphere.position.set(pos.X,pos.Y,pos.Z);selSphere.visible=true;
  document.getElementById('ep-title').textContent=`#${idx+1}  ${pos.type}  (Z.${pos.lineNum+1})`;
  ['x','y','z','a','b','c'].forEach(k=>document.getElementById('ep-'+k).value=pos[k.toUpperCase()].toFixed(3));
  document.getElementById('edit-panel').style.display='block';
  document.querySelectorAll('.pc').forEach((el,i)=>el.classList.toggle('selected',i===idx));
  // Hochpräzisions-IK für exakte Zielposition
  if(ikTable[idx]&&ikTable[idx].ok){
    var initAngles = ikTable[idx].angles.slice();
    var precise = solveIKPrecise(pos.X, pos.Y, pos.Z, pos.A, pos.B, pos.C, initAngles);
    if (precise.ok) {
      ikTable[idx].angles = precise.angles;
    }
    tweenToAngles(ikTable[idx].angles, 500);
  }
  // Show all IK variants for this position
  showEpIKSolutions(pos.X, pos.Y, pos.Z, pos.A, pos.B, pos.C);

  // FK-Δ berechnen und anzeigen (Abweichung IK→Ziel)
  if (ikTable[idx] && ikTable[idx].ok) {
    var fkChk = fkAll(ikTable[idx].angles);
    var tcpChk = fkChk.pts[7];
    if (tcpChk) {
      var dx = tcpChk[0]-pos.X, dy = tcpChk[1]-pos.Y, dz = tcpChk[2]-pos.Z;
      var err = Math.sqrt(dx*dx+dy*dy+dz*dz);
      document.getElementById('rb-fk').textContent = err.toFixed(2);
    }
  }
}

function deselectPosition(){
  selectedPosIdx=null;selSphere.visible=false;
  document.getElementById('edit-panel').style.display='none';
  document.querySelectorAll('.pc').forEach(el=>el.classList.remove('selected'));
}

document.getElementById('ep-close').onclick=deselectPosition;

function applyEditPanel(){
  if(selectedPosIdx===null)return;
  const pos=parsedData.positions[selectedPosIdx];
  const newPos={...pos,X:parseFloat(document.getElementById('ep-x').value)||0,Y:parseFloat(document.getElementById('ep-y').value)||0,Z:parseFloat(document.getElementById('ep-z').value)||0,A:parseFloat(document.getElementById('ep-a').value)||0,B:parseFloat(document.getElementById('ep-b').value)||0,C:parseFloat(document.getElementById('ep-c').value)||0};
  applyDraggedPos(selectedPosIdx,newPos,true);
}

['ep-x','ep-y','ep-z','ep-a','ep-b','ep-c'].forEach(id=>document.getElementById(id).addEventListener('keydown',e=>{if(e.key==='Enter')applyEditPanel();}));

function applyDraggedPos(idx,newPos,syncCode){
  parsedData.positions[idx]=newPos;
  const grp=posGrp.children[idx];
  if(grp){grp.position.set(newPos.X,newPos.Y,newPos.Z);grp.setRotationFromEuler(kukaEuler(newPos.A,newPos.B,newPos.C));}
  if(idx===selectedPosIdx){selSphere.position.set(newPos.X,newPos.Y,newPos.Z);['x','y','z','a','b','c'].forEach(k=>document.getElementById('ep-'+k).value=newPos[k.toUpperCase()].toFixed(3));}
  const card=document.getElementById('pcard-'+idx);
  if(card)card.querySelectorAll('.pf').forEach((el,i)=>{const vals=[newPos.X,newPos.Y,newPos.Z,newPos.A,newPos.B,newPos.C];const labels=['X','Y','Z','A(Z)','B(Y)','C(X)'];const units=['mm','mm','mm','°','°','°'];if(i<6)el.innerHTML=`<span>${labels[i]}</span> ${vals[i].toFixed(2)} ${units[i]}`;});
  updateVisitedPath(sim.t);
  if(syncCode)syncPositionToCode(idx);
  // recompute IK for this position
  const res=solveIK(newPos.X,newPos.Y,newPos.Z,newPos.A,newPos.B,newPos.C);
  ikTable[idx]=res;
  updateIKBadge(idx,res);
}

function syncPositionToCode(idx){
  const pos=parsedData.positions[idx];if(pos.lineNum===undefined)return;
  const ta=document.getElementById('code-input');const lines=ta.value.split(/\r?\n/);const old=lines[pos.lineNum];
  const str=`{X ${pos.X.toFixed(3)},Y ${pos.Y.toFixed(3)},Z ${pos.Z.toFixed(3)},A ${pos.A.toFixed(3)},B ${pos.B.toFixed(3)},C ${pos.C.toFixed(3)}${pos.S!==null&&pos.S!==undefined?',S '+pos.S:''}${pos.T!==null&&pos.T!==undefined?',T '+pos.T:''}}`;
  lines[pos.lineNum]=old.replace(/\{[^}]+\}/,str);ta.value=lines.join('\n');
}

document.getElementById('code-input').addEventListener('input',()=>rebuildGutter());

// ═══════════════════════════════════════════════════
// APPLY ANGLES — update robot model + status bar
// ═══════════════════════════════════════════════════
function applyAngles(angles) {
  for (let i=0;i<6;i++) jointAngles[i] = angles[i];
  buildRobotModel(angles);
  // status bar
  for (let i=0;i<6;i++) {
    const el = document.getElementById('rb-a'+(i+1));
    if (el) el.textContent = angles[i].toFixed(1);
  }
  // FK
  const fk = fkAll(angles);
  const tcp = fk.pts[7];
  document.getElementById('rb-fk').textContent = '—';
  addTCPTracePoint(tcp);
  // Update Steuerungspanel live
  updateSteuerPanel();
}

// ═══════════════════════════════════════════════════
// TOOL STL
// ═══════════════════════════════════════════════════
let toolMesh=null;
// STLLoader inline (r128 compatible)
class STLLoader {
  parse(buffer) {
    // Reliable binary check: 80 header + 4 count + n*50 = filesize
    const view = new DataView(buffer);
    const n = view.getUint32(80, true);
    if (n > 0 && 84 + n * 50 === buffer.byteLength) {
      return this._parseBinary(buffer);
    }
    const text = new TextDecoder().decode(new Uint8Array(buffer));
    if (text.startsWith('solid') && text.includes('facet')) return this._parseASCII(text);
    return this._parseBinary(buffer);
  }
  _parseASCII(text) {
    const geo = new THREE.BufferGeometry();
    const verts = [], norms = [];
    const lines = text.split("\n");
    let nx=0,ny=0,nz=0;
    for (const line of lines) {
      const l = line.trim();
      if (l.startsWith("facet normal")) { const p=l.split(/\s+/); nx=+p[2];ny=+p[3];nz=+p[4]; }
      else if (l.startsWith("vertex")) { const p=l.split(/\s+/); verts.push(+p[1],+p[2],+p[3]); norms.push(nx,ny,nz); }
    }
    geo.setAttribute("position",new THREE.Float32BufferAttribute(verts,3));
    geo.setAttribute("normal",new THREE.Float32BufferAttribute(norms,3));
    return geo;
  }
  _parseBinary(buffer) {
    const view = new DataView(buffer);
    const geo = new THREE.BufferGeometry();
    const n = view.getUint32(80, true);
    const verts = new Float32Array(n*9), norms = new Float32Array(n*9);
    let off = 84;
    for (let i=0;i<n;i++) {
      const nx=view.getFloat32(off,true),ny=view.getFloat32(off+4,true),nz=view.getFloat32(off+8,true); off+=12;
      for (let v=0;v<3;v++) { const j=i*9+v*3; verts[j]=view.getFloat32(off,true);verts[j+1]=view.getFloat32(off+4,true);verts[j+2]=view.getFloat32(off+8,true); norms[j]=nx;norms[j+1]=ny;norms[j+2]=nz; off+=12; } off+=2;
    }
    geo.setAttribute("position",new THREE.Float32BufferAttribute(verts,3));
    geo.setAttribute("normal",new THREE.Float32BufferAttribute(norms,3));
    return geo;
  }
}
const stlLoader = new STLLoader();

function loadToolFile(){document.getElementById('tool-file').click();}
document.getElementById('tool-file').addEventListener('change',function(){if(this.files[0])loadToolSTL(this.files[0]);this.value='';});

function loadToolSTL(file){
  const reader=new FileReader();
  reader.onload=e=>{
    if(toolMesh){markerGrp.remove(toolMesh);toolMesh.geometry.dispose();toolMesh.material.dispose();toolMesh=null;}
    const geo=stlLoader.parse(e.target.result);geo.computeVertexNormals();
    const mat=new THREE.MeshPhongMaterial({color:0xdd9944,transparent:false,opacity:1.0,side:THREE.DoubleSide,specular:0x666666});
    toolMesh=new THREE.Mesh(geo,mat);scene.add(toolMesh);
    buildRobotModel(jointAngles);
    document.getElementById('tool-controls').style.display='block';
    document.getElementById('tool-filename').textContent=file.name.replace(/\.stl$/i,'');
  };
  reader.readAsArrayBuffer(file);
}

function removeToolSTL(){
  if(toolMesh){markerGrp.remove(toolMesh);toolMesh.geometry.dispose();toolMesh.material.dispose();toolMesh=null;}
  document.getElementById('tool-controls').style.display='none';
}

// ═══════════════════════════════════════════════════
// STL SCENE MODELS
// ═══════════════════════════════════════════════════
const stlObjects=[];
document.getElementById('stl-file').addEventListener('change',function(){for(const file of this.files)loadSTL(file);this.value='';});

function loadSTL(file){
  const reader=new FileReader();
  reader.onload=e=>{
    const geo=stlLoader.parse(e.target.result);geo.computeVertexNormals();
    geo.computeBoundingBox();const ctr=new THREE.Vector3();geo.boundingBox.getCenter(ctr);geo.translate(-ctr.x,-ctr.y,-ctr.z);
    const mat=new THREE.MeshPhongMaterial({color:0x7799bb,transparent:true,opacity:.55,side:THREE.DoubleSide,specular:0x444444});
    const mesh=new THREE.Mesh(geo,mat);stlGrp.add(mesh);
    const idx=stlObjects.length;stlObjects.push({mesh,name:file.name});renderSTLEntry(idx);
  };
  reader.readAsArrayBuffer(file);
}

function renderSTLEntry(idx){
  const{mesh,name}=stlObjects[idx];const list=document.getElementById('stl-list');
  if(list.querySelector('.empty'))list.innerHTML='';
  const el=document.createElement('div');el.className='stl-entry';el.id='stl-'+idx;
  el.innerHTML=`<div class="stl-name">${name.replace(/\.stl$/i,'')}<span class="del" onclick="removeSTL(${idx})">✕</span></div>
    <div class="stl-grid">
      <span class="stl-lbl">X</span><input class="stl-inp" id="sx${idx}-x" type="number" value="0" step="1"><span class="stl-unit">mm</span>
      <span class="stl-lbl">Y</span><input class="stl-inp" id="sx${idx}-y" type="number" value="0" step="1"><span class="stl-unit">mm</span>
      <span class="stl-lbl">Z</span><input class="stl-inp" id="sx${idx}-z" type="number" value="0" step="1"><span class="stl-unit">mm</span>
      <span class="stl-lbl">A(Z)</span><input class="stl-inp" id="sx${idx}-a" type="number" value="0" step="1"><span class="stl-unit">°</span>
      <span class="stl-lbl">B(Y)</span><input class="stl-inp" id="sx${idx}-b" type="number" value="0" step="1"><span class="stl-unit">°</span>
      <span class="stl-lbl">C(X)</span><input class="stl-inp" id="sx${idx}-c" type="number" value="0" step="1"><span class="stl-unit">°</span>
    </div>`;
  list.appendChild(el);stlObjects[idx].el=el;
  for(const ax of['x','y','z','a','b','c'])document.getElementById(`sx${idx}-${ax}`).addEventListener('input',()=>updateSTL(idx));
}

function updateSTL(idx){const mesh=(stlObjects[idx]&&stlObjects[idx].mesh);if(!mesh)return;const g=ax=>parseFloat(document.getElementById(`sx${idx}-${ax}`).value)||0;mesh.position.set(g('x'),g('y'),g('z'));mesh.setRotationFromEuler(kukaEuler(g('a'),g('b'),g('c')));}
function removeSTL(idx){const obj=stlObjects[idx];if(!obj)return;stlGrp.remove(obj.mesh);obj.mesh.geometry.dispose();obj.mesh.material.dispose();((obj&&obj.el)&&obj.el.remove());stlObjects[idx]=null;if(document.getElementById('stl-list').children.length===0)document.getElementById('stl-list').innerHTML='<div class="empty">' + t('no_stl') + '</div>';}

// ═══════════════════════════════════════════════════
// SIMULATION ENGINE
// ═══════════════════════════════════════════════════
const sim={t:0,playing:false,dir:1,stepTarget:null,stepIdx:0};
const breakpoints=new Set();

function simSpeed(){return(parseInt(document.getElementById('spd-s').value)/100)*3.0;}
function lerpPos(a,b,f){return{X:a.X+(b.X-a.X)*f,Y:a.Y+(b.Y-a.Y)*f,Z:a.Z+(b.Z-a.Z)*f,A:a.A+(b.A-a.A)*f,B:a.B+(b.B-a.B)*f,C:a.C+(b.C-a.C)*f};}
function shortestAngleDiff(from, to) {
  // Returns smallest signed difference (always <= 180°)
  var d = ((to - from) % 360 + 540) % 360 - 180;
  return d;
}
function lerpAngles(a, b, f) {
  return a.map(function(v, i) {
    return v + shortestAngleDiff(v, b[i]) * f;
  });
}

// ── Trajectory: pre-computed fine-grained path ────────────
// Each entry: {pos:{X,Y,Z,A,B,C}, angles:[6], segIdx:int}
// segIdx = index in parsedData.positions of destination point
let trajectory = [];   // built once on parseAndLoad

// ── DPSolver initialisieren ───────────────────────────────────
(function() {
  DPSolver.fkFn = function(q) { return fkAll(q); };
  DPSolver.solveIKFn = function(x,y,z,a,b,c,init) { return solveIK(x,y,z,a,b,c,init); };
  DPSolver.singFn = function(q) {
    var m = computeManipulability(q);
    return Math.max(0, 1.0 - m.condition / 1000.0);
  };
  DPSolver.limits = JOINTS_DEF.map(function(j){ return {min:j.min, max:j.max}; });
  DPSolver.settings.wAxes = [1,1,1,4,5,0.25];
  DPSolver.settings.a6Copies = [-1,0,1];
  DPSolver.settings.smoothSamples = 150;
})();
let trajectoryRef = [];  // Referenz-Trajektorie für Map (unveränderlich)
let trajMax = 0;       // trajectory.length - 1

// ── Geometric helpers ─────────────────────────────────────
function vec3(p){return[p.X,p.Y,p.Z];}
function v3len(a){return Math.sqrt(a[0]**2+a[1]**2+a[2]**2);}
function v3sub(a,b){return[a[0]-b[0],a[1]-b[1],a[2]-b[2]];}
function v3add(a,b){return[a[0]+b[0],a[1]+b[1],a[2]+b[2]];}
function v3scale(a,s){return[a[0]*s,a[1]*s,a[2]*s];}
function v3dot(a,b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
function v3cross(a,b){return[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];}
function v3norm(a){const l=v3len(a)||1;return[a[0]/l,a[1]/l,a[2]/l];}

// Slerp orientation (simple linear for Euler — close enough for simulation)
function slerpOri(a,b,f){
  return{A:a.A+(b.A-a.A)*f,B:a.B+(b.B-a.B)*f,C:a.C+(b.C-a.C)*f};
}

// Circle through 3 points: returns {center, radius, normal, startAngle, totalAngle, arcLen}
function circleFrom3(p0,pAux,p1){
  const a=vec3(p0),m=vec3(pAux),b=vec3(p1);
  const ab=v3sub(b,a),am=v3sub(m,a),mb=v3sub(b,m);
  const n=v3norm(v3cross(ab,am));
  // Circumcenter in the plane
  const ac=v3sub(b,a),ad=v3sub(m,a);
  const d=2*(ac[0]*(ad[1]*n[2]-ad[2]*n[1])-ac[1]*(ad[0]*n[2]-ad[2]*n[0])+ac[2]*(ad[0]*n[1]-ad[1]*n[0]));
  if(Math.abs(d)<1e-9) return null;
  const s=((v3dot(ac,ac)*(ad[1]*n[2]-ad[2]*n[1])-v3dot(ad,ad)*(ac[1]*n[2]-ac[2]*n[1]))/d);
  // Center via parametric circumcenter formula
  const cx=a[0]+0.5*(b[0]-a[0]),cy=a[1]+0.5*(b[1]-a[1]),cz=a[2]+0.5*(b[2]-a[2]);
  // xax = component of (p0-pAux) projected onto circle plane, normalized
  const pa=v3sub(a,m);
  const pa_proj=v3sub(pa,v3scale(n,v3dot(pa,n)));
  if(v3len(pa_proj)<1e-9) return null;
  const xax=v3norm(pa_proj);
  const yax=v3cross(n,xax);
  function proj2(p){const rel=v3sub(p,a);return[v3dot(rel,xax),v3dot(rel,yax)];}
  const A2=proj2(a),M2=proj2(m),B2=proj2(b);
  // circumcenter in 2D
  const ax=A2[0],ay=A2[1],bx=B2[0],by=B2[1],mx=M2[0],my=M2[1];
  const D2=2*(ax*(by-my)+bx*(my-ay)+mx*(ay-by));
  if(Math.abs(D2)<1e-9)return null;
  const ux=((ax**2+ay**2)*(by-my)+(bx**2+by**2)*(my-ay)+(mx**2+my**2)*(ay-by))/D2;
  const uy=((ax**2+ay**2)*(mx-bx)+(bx**2+by**2)*(ax-mx)+(mx**2+my**2)*(bx-ax))/D2;
  const center=v3add(a,v3add(v3scale(xax,ux),v3scale(yax,uy)));
  const radius=v3len(v3sub(a,center));
  const toA=v3norm(v3sub(a,center));
  const toB=v3norm(v3sub(b,center));
  const toM=v3norm(v3sub(m,center));
  const angA=Math.atan2(v3dot(toA,yax),v3dot(toA,xax));
  const angB=Math.atan2(v3dot(toB,yax),v3dot(toB,xax));
  const angM=Math.atan2(v3dot(toM,yax),v3dot(toM,xax));
  // Determine direction (CCW or CW) via auxiliary point
  let totalAngle=angB-angA;
  const mAngle=angM-angA;
  // normalize
  const norm2=(v)=>{let x=v;while(x>Math.PI)x-=2*Math.PI;while(x<-Math.PI)x+=2*Math.PI;return x;}
  totalAngle=norm2(totalAngle);
  const mN=norm2(mAngle);
  // if aux is not between a and b, flip direction
  if(totalAngle>0&&mN<0){totalAngle-=2*Math.PI;}
  else if(totalAngle<0&&mN>0){totalAngle+=2*Math.PI;}
  const arcLen=Math.abs(radius*totalAngle);
  return{center,radius,n,xax,yax,startAngle:angA,totalAngle,arcLen};
}

function sampleArc(arc,f){
  const angle=arc.startAngle+arc.totalAngle*f;
  return[
    arc.center[0]+arc.radius*(Math.cos(angle)*arc.xax[0]+Math.sin(angle)*arc.yax[0]),
    arc.center[1]+arc.radius*(Math.cos(angle)*arc.xax[1]+Math.sin(angle)*arc.yax[1]),
    arc.center[2]+arc.radius*(Math.cos(angle)*arc.xax[2]+Math.sin(angle)*arc.yax[2]),
  ];
}

// Catmull-Rom spline for SLIN
function catmullRomSegment(p0,p1,p2,p3,t){
  const v=([a,b,c])=>[a,b,c];
  const s=0.5;
  return[
    s*((-p0[0]+3*p1[0]-3*p2[0]+p3[0])*t**3+(2*p0[0]-5*p1[0]+4*p2[0]-p3[0])*t**2+(-p0[0]+p2[0])*t+2*p1[0])/1,
    s*((-p0[1]+3*p1[1]-3*p2[1]+p3[1])*t**3+(2*p0[1]-5*p1[1]+4*p2[1]-p3[1])*t**2+(-p0[1]+p2[1])*t+2*p1[1])/1,
    s*((-p0[2]+3*p1[2]-3*p2[2]+p3[2])*t**3+(2*p0[2]-5*p1[2]+4*p2[2]-p3[2])*t**2+(-p0[2]+p2[2])*t+2*p1[2])/1,
  ];
}

// ── Build trajectory from parsed positions ────────────────
function buildTrajectory(positions, ikTab) {
  trajectory = [];
  const N = positions.length;
  if (!N) { trajMax = 0; return; }

  // Adaptives Sampling: mehr Punkte = größerer Schritt
  const STEP_MM = N > 500 ? 25 : N > 200 ? 15 : N > 50 ? 10 : 8;

  // IK with warm-start from previous angles
  // Used for LIN/SLIN/CIRC to guarantee straight Cartesian path
  function ikWarm(pos, prevAngles) {
    const res = solveIK(pos.X, pos.Y, pos.Z, pos.A, pos.B, pos.C, prevAngles);
    return res.ok ? res.angles : prevAngles;
  }

  function pushSample(pos, angles, segIdx) {
    trajectory.push({pos, angles, segIdx});
  }

  // Startwinkel: letzter PTP-Achsbefehl vor den LIN-Positionen
  var _buildStart = (ikTab[0]&&ikTab[0].angles) || [0,-90,90,0,0,0];
  if (parsedData && parsedData.steps) {
    for (var _si=0; _si<parsedData.steps.length; _si++) {
      var _st=parsedData.steps[_si];
      if (_st.type==='ptpAxis'&&_st.angles) _buildStart=_st.angles.slice();
      if (_st.type==='move') break;
    }
  }
  pushSample(positions[0], _buildStart, 0);

  for (let i = 1; i < N; i++) {
    const prev = positions[i-1];
    const curr = positions[i];
    const angPrev = (ikTab[i-1]&&ikTab[i-1].angles) || [0,-90,90,0,0,0];
    const angCurr = (ikTab[i]&&ikTab[i].angles)   || [0,-90,90,0,0,0];
    const type = curr.type;

    if (type === 'CIRC_AUX') {
      pushSample(curr, angCurr, i);
      continue;
    }

    if (type === 'CIRC' && i > 0 && (positions[i-1]&&positions[i-1].type) === 'CIRC_AUX') {
      const pStart = i >= 2 ? positions[i-2] : prev;
      const pAux   = prev;
      const pEnd   = curr;
      const arc    = circleFrom3(pStart, pAux, pEnd);
      if (!arc || arc.arcLen < 1) { pushSample(curr, angCurr, i); continue; }
      const steps = Math.max(4, Math.ceil(arc.arcLen / STEP_MM));
      const angStart = (ikTab[Math.max(0,i-2)]&&ikTab[Math.max(0,i-2)].angles) || angPrev;
      let warmAng = [...angStart];
      for (let s = 1; s <= steps; s++) {
        const f = s / steps;
        const pt = sampleArc(arc, f);
        const ori = slerpOri(pStart, pEnd, f);
        const pos = {X:pt[0], Y:pt[1], Z:pt[2], ...ori};
        // IK at each arc point → true circular path
        warmAng = ikWarm(pos, warmAng);
        pushSample(pos, warmAng, i);
      }
      continue;
    }

    if (type === 'SLIN') {
      const p0 = i >= 2  ? vec3(positions[i-2]) : vec3(prev);
      const p1 = vec3(prev);
      const p2 = vec3(curr);
      const p3 = i+1 < N ? vec3(positions[i+1]) : vec3(curr);
      const dist = v3len(v3sub(p2, p1));
      const steps = Math.max(4, Math.ceil(dist / STEP_MM));
      let warmAng = [...angPrev];
      for (let s = 1; s <= steps; s++) {
        const f = s / steps;
        const pt = catmullRomSegment(p0,p1,p2,p3,f);
        const ori = slerpOri(prev, curr, f);
        const pos = {X:pt[0], Y:pt[1], Z:pt[2], ...ori};
        // IK at each spline point → smooth curve in Cartesian space
        warmAng = ikWarm(pos, warmAng);
        pushSample(pos, warmAng, i);
      }
      continue;
    }

    if (type === 'LIN') {
      const dist = v3len(v3sub(vec3(curr), vec3(prev)));
      const steps = Math.max(4, Math.ceil(dist / STEP_MM));
      // Normalize angCurr to closest path from angPrev (keine 360°-Sprünge)
      const angCurrLin = angCurr.map(function(v, j) {
        return angPrev[j] + shortestAngleDiff(angPrev[j], v);
      });
      let warmAng = [...angPrev];
      for (let s = 1; s <= steps; s++) {
        const f = s / steps;
        const pos = lerpPos(prev, curr, f);
        if (dist < 20) {
          // Sehr kurze LIN: direkt zwischen ikTable-Winkeln interpolieren
          // (IK würde falsche Konfiguration finden)
          warmAng = lerpAngles(angPrev, angCurrLin, f);
        } else {
          const res = solveIK(pos.X, pos.Y, pos.Z, pos.A, pos.B, pos.C, warmAng);
          // Nur übernehmen wenn nah an der erwarteten Konfiguration
          if (res.ok) {
            const resN = res.angles.map(function(v, j) {
              return warmAng[j] + shortestAngleDiff(warmAng[j], v);
            });
            const totalDelta = resN.reduce(function(s, v, j) { return s + Math.abs(v - warmAng[j]); }, 0);
            warmAng = (totalDelta < 90) ? resN : lerpAngles(angPrev, angCurrLin, f);
          } else {
            warmAng = lerpAngles(angPrev, angCurrLin, f);
          }
        }
        pushSample(pos, warmAng, i);
      }
      continue;
    }

    // PTP: joint-space interpolation mit kürzestem Winkelweg
    const dist = v3len(v3sub(vec3(curr), vec3(prev)));
    const steps = Math.max(2, Math.ceil(dist / (STEP_MM*3)));
    // Normalize angCurr to be closest to angPrev (no 360° flips)
    const angCurrN = angCurr.map(function(v, j) {
      var diff = shortestAngleDiff(angPrev[j], v);
      return angPrev[j] + diff;
    });
    for (let s = 1; s <= steps; s++) {
      const f = s / steps;
      const pos = lerpPos(prev, curr, f);
      const angles = lerpAngles(angPrev, angCurrN, f);
      pushSample(pos, angles, i);
    }
  }

  trajMax = Math.max(0, trajectory.length - 1);
  // Referenz-Trajektorie für Map (tief kopieren)
  trajectoryRef = trajectory.map(function(t) {
    return { pos: t.pos, angles: t.angles ? t.angles.slice() : [] };
  });

  // Rebuild path line from actual Cartesian trajectory
  pathGrp.clear();
  // Full programmed path from parsedData.positions (always correct)
  if (parsedData.positions.length > 1) {
    const progPts = parsedData.positions.map(p => new THREE.Vector3(p.X, p.Y, p.Z));
    pathGrp.add(new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(progPts),
      new THREE.LineBasicMaterial({color:hexToInt((document.getElementById('cfg-planned-col')||{value:'#1a3050'}).value)})
    ));
  }
}

function getTrajSample(t) {
  if (!trajectory.length) return null;
  const f = Math.max(0, Math.min(1, t));
  const idx = Math.floor(f * trajMax);
  const next = Math.min(idx+1, trajMax);
  const frac = f * trajMax - idx;
  const a = trajectory[idx], b = trajectory[next];
  if (!a) return null;
  const pos = lerpPos(a.pos, b.pos, frac);
  const angles = lerpAngles(a.angles, b.angles, frac);
  return {pos, angles, segIdx: a.segIdx};
}

// Legacy: map t (0..N-1) to trajectory fraction
function simTToTrajT(t) {
  const N = parsedData.positions.length;
  return N > 1 ? t / (N-1) : 0;
}

function getIPos(t){
  const s = getTrajSample(simTToTrajT(t));
  return (s&&s.pos) || null;
}

function getIKAngles(t) {
  const s = getTrajSample(simTToTrajT(t));
  return (s&&s.angles) || null;
}

function updateMarkerPose(ipos){markerGrp.position.set(ipos.X,ipos.Y,ipos.Z);markerGrp.setRotationFromEuler(kukaEuler(ipos.A,ipos.B,ipos.C));markerGrp.visible=true;}

// Visited path
let visitedLine=null;
function updateVisitedPath(t){
  if(visitedLine){pathGrp.remove(visitedLine);visitedLine=null;}
  if(!trajectory.length||t<=0)return;
  const pathCol=hexToInt(document.getElementById('cfg-path-col').value);
  const tf=simTToTrajT(t);
  const cutIdx=Math.floor(tf*trajMax);
  if(cutIdx<1)return;
  const pts=trajectory.slice(0,cutIdx+1).map(s=>new THREE.Vector3(s.pos.X,s.pos.Y,s.pos.Z));
  if(pts.length>1){visitedLine=new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts),new THREE.LineBasicMaterial({color:pathCol,linewidth:pathLineWidth}));pathGrp.add(visitedLine);}
}

function setStatus(cls,txt){const el=document.getElementById('sim-status');el.className='sstatus '+cls;el.textContent=txt;}

function checkBP(prevT,newT,dir){
  const pos=parsedData.positions;
  if(dir>0){const fr=Math.floor(prevT)+(Number.isInteger(prevT)?1:0);for(let i=fr;i<=Math.floor(newT);i++)if(i<pos.length&&breakpoints.has(pos[i].lineNum))return i;}
  else{const fr=Math.ceil(prevT)-(Number.isInteger(prevT)?1:0);for(let i=fr;i>=Math.ceil(newT);i--)if(i>=0&&breakpoints.has(pos[i].lineNum))return i;}
  return null;
}

function applySimT(t){ _tween = null;
  const pos=parsedData.positions,N=pos.length;if(!N)return;
  sim.t=Math.max(0,Math.min(N-1,t));
  document.getElementById('pos-s').value=sim.t;
  const idx=Math.min(Math.round(sim.t),N-1);
  // Show trajectory sample count
  const trajSamples = trajectory.length;
  document.getElementById('pos-v').textContent=`${idx+1} / ${N}` + (trajSamples>N?` (${trajSamples} Schritte)`:'');
  updateVisitedPath(sim.t);updateGutterActive(pos[idx].lineNum);updatePosCards(idx);updateSignalsForStep(pos[idx]);
  if(parsedData.steps){const si=parsedData.steps.findIndex(s=>s.type==='move'&&s.posIdx===idx);if(si>=0)sim.stepIdx=si;}
  // Bei exaktem Positionsindex: präzise ikTable-Winkel verwenden
  let angles;
  if(Number.isInteger(sim.t) && ikTable[idx] && ikTable[idx].ok) {
    angles = ikTable[idx].angles;
  } else {
    angles = getIKAngles(sim.t);
  }
  if(angles){
    applyAngles(angles);
    // Place simulation marker at actual FK TCP (not stored pos) → always consistent
    const fkRes=fkAll(angles);
    const fkTCP=fkRes.pts[7];
    // Marker at trajectory.pos (programmed target) — robot at FK(angles)
    // This keeps the marker on the programmed path even if IK has small residual
    const ip_traj = getTrajSample(simTToTrajT(sim.t));
    if (ip_traj) {
      updateMarkerPose(ip_traj.pos);
    } else {
      const R=fkRes.tcp_rot;
      const Beu=-Math.asin(Math.max(-1,Math.min(1,R[2][0])));
      const cb=Math.cos(Beu);let Aeu,Ceu;
      if(Math.abs(cb)>1e-6){Aeu=Math.atan2(R[1][0]/cb,R[0][0]/cb);Ceu=Math.atan2(R[2][1]/cb,R[2][2]/cb);}
      else{Aeu=0;Ceu=Math.atan2(-R[1][2],R[1][1]);}
      const nd=v=>{let d=v*180/Math.PI;while(d>180)d-=360;while(d<=-180)d+=360;return d;}
      updateMarkerPose({X:fkTCP[0],Y:fkTCP[1],Z:fkTCP[2],A:nd(Aeu),B:nd(Beu),C:nd(Ceu)});
    }
    // Status bar: show FK error vs stored trajectory target
    const ip=getIPos(sim.t);
    if(ip){
      const err=Math.sqrt((fkTCP[0]-ip.X)**2+(fkTCP[1]-ip.Y)**2+(fkTCP[2]-ip.Z)**2);
      document.getElementById('rb-fk').textContent=err.toFixed(2);
      const mi=document.getElementById('marker-info');
      mi.style.display='block';
      mi.textContent=`#${idx+1} ${pos[idx].type}  X${fkTCP[0].toFixed(1)} Y${fkTCP[1].toFixed(1)} Z${fkTCP[2].toFixed(1)}  FK-Δ:${err.toFixed(1)}mm`;
    }
  } else {
    const ip=getIPos(sim.t);if(ip)updateMarkerPose(ip);
    const mi=document.getElementById('marker-info');
    if(ip){mi.style.display='block';mi.textContent=`#${idx+1} ${pos[idx].type}  X${ip.X.toFixed(1)} Y${ip.Y.toFixed(1)} Z${ip.Z.toFixed(1)}`;}
  }
}

function updateSignalsForStep(pos){
  if(!(pos&&pos.snapshot))return;const s=pos.snapshot;
  renderVariables(s.variables);renderDigital(s.digitalIn,'$IN','din-list');renderDigital(s.digitalOut,'$OUT','dout-list');renderAnalog(s.analogOut);
}

function applyStep(idx){
  const steps=parsedData.steps;if(!(steps&&steps.length))return;
  idx=Math.max(0,Math.min(steps.length-1,idx));sim.stepIdx=idx;const step=steps[idx];
  updateGutterActive(step.lineNum);
  if(step.snapshot){renderVariables(step.snapshot.variables);renderDigital(step.snapshot.digitalIn,'$IN','din-list');renderDigital(step.snapshot.digitalOut,'$OUT','dout-list');renderAnalog(step.snapshot.analogOut);}
  let posIdx=-1;for(let i=idx;i>=0;i--){if(steps[i].type==='move'){posIdx=steps[i].posIdx;break;}}
  const N=parsedData.positions.length;
  if(posIdx>=0){
    const pos=parsedData.positions[posIdx];updateMarkerPose(pos);markerGrp.visible=true;updatePosCards(posIdx);updateVisitedPath(posIdx);
    document.getElementById('pos-s').value=posIdx;document.getElementById('pos-v').textContent=`${posIdx+1} / ${N}`;
    document.getElementById('marker-info').style.display='block';document.getElementById('marker-info').textContent=`Z.${step.lineNum+1}  #${posIdx+1} ${pos.type}  X${pos.X.toFixed(1)} Y${pos.Y.toFixed(1)} Z${pos.Z.toFixed(1)}`;
    if((ikTable[posIdx]&&ikTable[posIdx].ok))applyAngles(ikTable[posIdx].angles);
  }else{document.getElementById('marker-info').style.display='none';if(N>0)document.getElementById('pos-v').textContent=`— / ${N}`;}
  const typeLabel={move:'MOVE',signal:'SIGNAL',var:'VARIABLE',other:'STATEMENT'};
  setStatus('paused',`L.${step.lineNum+1}  ${typeLabel[step.type]||''}`);
}

function pauseSim(){sim.playing=false;sim.stepTarget=null;document.getElementById('b-playfwd').classList.remove('on');document.getElementById('b-playrev').classList.remove('on');}
function stopSim(goTo){pauseSim();if(goTo!==undefined)applySimT(goTo);}


// ── Achsen-Interpolation (Tween) ──────────────────────────────
var _tween = null;  // { from, to, t, duration }

function tweenToAngles(targetAngles, durationMs) {
  durationMs = durationMs || 600;
  _tween = {
    from: jointAngles.slice(),
    to:   targetAngles.slice(),
    t:    0,
    duration: durationMs / 1000
  };
}

function updateTween(dt) {
  if (!_tween) return;
  _tween.t += dt;
  var f = Math.min(_tween.t / _tween.duration, 1);
  // Ease in-out
  f = f < 0.5 ? 2*f*f : -1 + (4 - 2*f)*f;
  var angles = _tween.from.map(function(v, i) {
    return v + (_tween.to[i] - v) * f;
  });
  applyAngles(angles);
  if (_tween.t >= _tween.duration) _tween = null;
}


// ── Achsenkarte Cursor (Simulations-Position) ─────────────────
function ampUpdateCursor() {
  var cc = document.getElementById('amp-cursor');
  var mc = document.getElementById('amp-canvas');
  if (!cc || !mc || !ampCols) return;
  var W = mc.width, H = mc.height;
  if (!W || !H) return;
  cc.width  = W;
  cc.height = H;
  cc.style.width  = mc.style.width  || W + 'px';
  cc.style.height = mc.style.height || H + 'px';

  var ctx = cc.getContext('2d');
  ctx.clearRect(0, 0, W, H);

  var N = parsedData.positions.length;
  if (N < 2) return;

  var tFrac = sim.t / (N - 1);
  var px = tFrac * (W - 1);

  // Aktuelle A6-Position aus ikTable
  var idx = Math.min(Math.round(sim.t), N-1);
  var curA6 = (ikTable[idx] && ikTable[idx].ok) ? ikTable[idx].angles[5] : null;

  // Vertikale Linie
  ctx.strokeStyle = 'rgba(255,240,0,0.9)';
  ctx.lineWidth = 2;
  ctx.setLineDash([5, 4]);
  ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, H); ctx.stroke();
  ctx.setLineDash([]);

  // Punkt auf der aktuellen A6-Position
  if (curA6 !== null) {
    var py = H - ((curA6 - A6_MIN) / (A6_MAX - A6_MIN)) * H;
    py = Math.max(4, Math.min(H-4, py));
    ctx.fillStyle = '#ffee00';
    ctx.strokeStyle = '#000';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(px, py, 5, 0, Math.PI * 2);
    ctx.fill(); ctx.stroke();
  }

  // Positions-Label oben
  ctx.fillStyle = 'rgba(255,240,0,0.85)';
  ctx.font = 'bold 10px monospace';
  var label = '#' + (idx+1) + '/' + N;
  var tx = Math.min(px + 4, W - ctx.measureText(label).width - 2);
  ctx.fillText(label, tx, 11);

  // Aktuellen Zielpunkt auf Planlinie hervorheben
  if (ampUserPath.length > 0 && N > 1) {
    // Bogenlänge: idx → planCol
    var planCol2 = 0;
    var _arcRef = window._ampArcLen || [];
    if (_arcRef.length > 0 && trajectory.length > 1) {
      var _s2 = _arcRef[Math.min(idx, _arcRef.length-1)] || 0;
      var _tot2 = _arcRef[_arcRef.length-1] || 1;
      planCol2 = Math.round((_s2 / _tot2) * (ampCols-1));
    } else {
      planCol2 = Math.round(idx / Math.max(1,N-1) * (ampCols-1));
    }
    planCol2 = Math.max(0, Math.min(ampCols-1, planCol2));
    var a6plan2 = ampUserPath[planCol2] !== undefined ? ampUserPath[planCol2] : 0;
    var pyCur = H - ((a6plan2 - A6_MIN) / (A6_MAX - A6_MIN)) * H;
    pyCur = Math.max(6, Math.min(H-6, pyCur));
    ctx.fillStyle = 'rgba(255,240,0,0.3)';
    ctx.strokeStyle = '#ffee00';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(px, pyCur, 8, 0, Math.PI * 2);
    ctx.fill(); ctx.stroke();
  }
}

let lastTs=null;
function frame(ts){
  requestAnimationFrame(frame);
  const dt=lastTs!==null?Math.min((ts-lastTs)/1000,.1):0;lastTs=ts;
  updateTween(dt);
  const N=parsedData.positions.length;
  if(N>0&&(sim.playing||sim.stepTarget!==null)){
    const speed=simSpeed(),prevT=sim.t;let newT;
    if(sim.stepTarget!==null){
      const sd=sim.stepTarget>sim.t?1:-1;newT=sim.t+sd*speed*dt;
      if((sd>0&&newT>=sim.stepTarget)||(sd<0&&newT<=sim.stepTarget)){newT=sim.stepTarget;sim.stepTarget=null;applySimT(newT);setStatus('paused','PAUSED');renderer.render(scene,activeCam);return;}
    }else{
      newT=sim.t+sim.dir*speed*dt;
      const bp=checkBP(prevT,newT,sim.dir);
      if(bp!==null){newT=bp;pauseSim();applySimT(newT);setStatus('bp','● BREAKPOINT');renderer.render(scene,activeCam);return;}
      if(newT>=N-1){newT=N-1;pauseSim();setStatus('end','END ✓');}
      else if(newT<=0){newT=0;pauseSim();setStatus('paused','START');}
    }
    applySimT(newT);
  }
  renderer.render(scene,activeCam);
  // Achsenkarte Cursor: nur bei Änderung updaten
  var ampPanel = document.getElementById('axis-map-panel');
  if (ampPanel && ampPanel.classList.contains('visible')) {
    if (typeof _lastCursorT === 'undefined') window._lastCursorT = -1;
    if (Math.abs(sim.t - window._lastCursorT) > 0.01) {
      window._lastCursorT = sim.t;
      ampUpdateCursor();
    }
  }
}
requestAnimationFrame(frame);

// Buttons
document.getElementById('b-start').onclick=()=>{pauseSim();applySimT(0);if((parsedData.steps&&parsedData.steps.length))applyStep(0);else setStatus('stopped','STOPPED');};
document.getElementById('b-end').onclick=()=>{pauseSim();applySimT(Math.max(0,parsedData.positions.length-1));const last=((parsedData.steps&&parsedData.steps.length)||1)-1;if((parsedData.steps&&parsedData.steps.length))applyStep(last);else setStatus('stopped','STOPPED');};
document.getElementById('b-stop').onclick=()=>{pauseSim();setStatus('paused','PAUSED');};
document.getElementById('b-playfwd').onclick=()=>{if(sim.playing&&sim.dir===1){pauseSim();setStatus('paused','PAUSED');}else{if(!parsedData.positions.length)parseAndLoad();if(!parsedData.positions.length)return;if(sim.t>=parsedData.positions.length-1)applySimT(0);sim.playing=true;sim.dir=1;sim.stepTarget=null;document.getElementById('b-playfwd').classList.add('on');document.getElementById('b-playrev').classList.remove('on');setStatus('playing','▶ FORWARD');}};
document.getElementById('b-playrev').onclick=()=>{if(sim.playing&&sim.dir===-1){pauseSim();setStatus('paused','PAUSED');}else{if(!parsedData.positions.length)parseAndLoad();if(!parsedData.positions.length)return;if(sim.t<=0)applySimT(parsedData.positions.length-1);sim.playing=true;sim.dir=-1;sim.stepTarget=null;document.getElementById('b-playrev').classList.add('on');document.getElementById('b-playfwd').classList.remove('on');setStatus('playing','◀ BACKWARD');}};
document.getElementById('b-stepfwd').onclick=()=>{if(!(parsedData.steps&&parsedData.steps.length))return;pauseSim();applyStep(sim.stepIdx+1);};
document.getElementById('b-steprev').onclick=()=>{if(!(parsedData.steps&&parsedData.steps.length))return;pauseSim();applyStep(sim.stepIdx-1);};
document.getElementById('spd-s').addEventListener('input',function(){document.getElementById('spd-v').textContent=this.value+'%';});
document.getElementById('pos-s').addEventListener('mousedown',()=>{if(sim.playing){pauseSim();setStatus('paused','PAUSED');}});
document.getElementById('pos-s').addEventListener('input',function(){const posIdx=Math.round(parseFloat(this.value));applySimT(posIdx);if(parsedData.steps){const si=parsedData.steps.findIndex(s=>s.type==='move'&&s.posIdx===posIdx);if(si>=0)applyStep(si);}});

window.addEventListener('keydown',e=>{
  const inEditor=e.target===document.getElementById('code-input')||e.target.tagName==='INPUT';
  // F-keys: always active
  if(e.key==='F5'){e.preventDefault();document.getElementById('b-playfwd').click();return;}
  if(e.key==='F6'){e.preventDefault();document.getElementById('b-stop').click();return;}
  if(e.key==='F7'){e.preventDefault();document.getElementById('b-stepfwd').click();return;}
  if(e.key==='F8'){e.preventDefault();document.getElementById('b-steprev').click();return;}
  if(e.key==='F9'){e.preventDefault();document.getElementById('parse-btn').click();return;}
  if(e.key==='F10'){e.preventDefault();toggleAxisMap();return;}
  if(e.key==='F11'){e.preventDefault();toggleSteuerPanel();return;}
  if(e.key==='F11'){e.preventDefault();toggleGrid();return;}
  if(inEditor)return;
  if(e.key===' '){e.preventDefault();document.getElementById('b-stop').click();}
  if(e.key==='ArrowRight'){e.preventDefault();document.getElementById('b-stepfwd').click();}
  if(e.key==='ArrowLeft'){e.preventDefault();document.getElementById('b-steprev').click();}
  if(e.key==='Home'){e.preventDefault();document.getElementById('b-start').click();}
  if(e.key==='End'){e.preventDefault();document.getElementById('b-end').click();}
  if(e.key==='Escape'){deselectPosition();}
});

// ═══════════════════════════════════════════════════
// PATH BUILDING
// ═══════════════════════════════════════════════════
function buildScene(positions){
  posGrp.clear(); posGrp.visible = showPosFrames;pathGrp.clear();tcpTraceGrp.clear();tcpTracePoints.length=0;
  visitedLine=null;markerGrp.visible=false;selSphere.visible=false;
  if(!positions.length)return;
  const pts=positions.map(p=>new THREE.Vector3(p.X,p.Y,p.Z));
  if(pts.length>1)pathGrp.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts),new THREE.LineBasicMaterial({color:hexToInt((document.getElementById('cfg-planned-col')||{value:'#1a3050'}).value),linewidth:pathLineWidth})));
  positions.forEach((pos,i)=>{const g=makeFrame(pos);g.userData.posIdx=i;posGrp.add(g);});
  const box=new THREE.Box3();pts.forEach(p=>box.expandByPoint(p));
  const ctr=new THREE.Vector3();box.getCenter(ctr);const span=box.getSize(new THREE.Vector3()).length();
  orbitTarget.copy(ctr);orbitState.radius=Math.max(800,span*1.9);orthoHalfSize=Math.max(800,span*.7);updateCamera();
}

// ═══════════════════════════════════════════════════
// GUTTER
// ═══════════════════════════════════════════════════
let posLineNums=new Set();
function buildGutter(count){
  const gut=document.getElementById('gutter');gut.innerHTML='';
  for(let i=0;i<count;i++){
    const row=document.createElement('div');row.className='gl';row.dataset.line=i;
    if(posLineNums.has(i))row.classList.add('can-bp');if(breakpoints.has(i))row.classList.add('has-bp');
    const ln=document.createElement('span');ln.className='ln';ln.textContent=i+1;
    const dot=document.createElement('span');dot.className='bpdot';
    row.appendChild(ln);row.appendChild(dot);row.addEventListener('click',()=>toggleBP(i,row));gut.appendChild(row);
  }
}
function toggleBP(ln,row){if(!posLineNums.has(ln))return;if(breakpoints.has(ln)){breakpoints.delete(ln);row.classList.remove('has-bp');}else{breakpoints.add(ln);row.classList.add('has-bp');}}
function rebuildGutter(){const lines=document.getElementById('code-input').value.split(/\r?\n/);buildGutter(lines.length);}
document.getElementById('code-input').addEventListener('scroll',function(){document.getElementById('gutter').scrollTop=this.scrollTop;});
function updateGutterActive(lineNum){document.querySelectorAll('#gutter .gl').forEach(r=>r.classList.remove('active'));const t=document.querySelector(`#gutter .gl[data-line="${lineNum}"]`);if(!t)return;t.classList.add('active');const ta=document.getElementById('code-input');const lh=20,pv=10;const top=lineNum*lh+pv;if(top<ta.scrollTop||top>ta.scrollTop+ta.clientHeight-lh)ta.scrollTop=Math.max(0,top-ta.clientHeight/2);}

// ═══════════════════════════════════════════════════
// UI RENDER HELPERS
// ═══════════════════════════════════════════════════
const TYPE_LBL={LIN:'LIN — Linear',PTP:'PTP — Point-to-Point',SLIN:'SLIN — Soft-Linear',CIRC:'CIRC — Endpunkt',CIRC_AUX:'CIRC — Hilfspunkt'};
function ff(v,d=2){return typeof v==='number'?v.toFixed(d):String(v);}

function updateIKBadge(idx, res) {
  const card = document.getElementById('pcard-'+idx);
  if (!card) return;
  const badge = card.querySelector('.ik-reach');
  if (!badge) return;
  badge.className = 'ik-reach' + ((res&&res.ok) ? '' : ' err');
  badge.textContent = (res&&res.ok) ? `IK ✓  Δ${res.score.toFixed(1)}` : 'IK ✗ nicht erreichbar';
}

function renderPositions(positions){
  const el=document.getElementById('pos-list');
  if(!positions.length){el.innerHTML='<div class="empty">' + t('no_pos2') + '</div>';return;}
  el.innerHTML=positions.map((p,i)=>{
    const tc=(p.type||'').toLowerCase();
    const ik=ikTable[i];
    const ikHtml=ik?`<div class="psep"></div><div class="ik-reach${ik.ok?'':' err'}">${ik.ok?`IK ✓  Δ${ik.score.toFixed(1)}`:'IK ✗ nicht erreichbar'}</div>`:'';
    return`<div class="pc ${tc}" id="pcard-${i}">
      <div class="pc-type ${tc}">#${i+1} &nbsp;${TYPE_LBL[p.type]||p.type}${p.lineNum!==undefined?`<span style="color:var(--txt3);font-weight:normal;font-size:.85em"> L.${p.lineNum+1}</span>`:''}
      </div>
      <div class="pc-grid">
        <div class="pf"><span>X</span> ${ff(p.X)} mm</div><div class="pf"><span>Y</span> ${ff(p.Y)} mm</div><div class="pf"><span>Z</span> ${ff(p.Z)} mm</div>
        <div class="psep"></div>
        <div class="pf"><span>A(Z)</span> ${ff(p.A)}°</div><div class="pf"><span>B(Y)</span> ${ff(p.B)}°</div><div class="pf"><span>C(X)</span> ${ff(p.C)}°</div>
        ${ikHtml}
      </div></div>`;
  }).join('');
  positions.forEach((_,i)=>{(function(){var _e=document.getElementById('pcard-'+i);if(_e)_e.addEventListener('click',function(){pauseSim();setStatus('paused','PAUSED');selectPosition(i);});})();;});
}

function updatePosCards(activeIdx){document.querySelectorAll('.pc').forEach((el,i)=>el.classList.toggle('sim-cur',i===activeIdx));(function(){var _e=document.getElementById('pcard-'+activeIdx);if(_e)_e.scrollIntoView({block:'nearest',behavior:'smooth'});})();;}
function renderVariables(vars){const el=document.getElementById('var-list');const entries=Object.entries(vars);if(!entries.length){el.innerHTML='<div class="empty">' + t('no_vars') + '</div>';return;}el.innerHTML=entries.map(([n,v])=>{const d=typeof v==='boolean'?(v?'TRUE':'FALSE'):String(v);return`<div class="vr"><span class="vn">${n}</span><span class="vv">${d}</span></div>`;}).join('');}
function renderDigital(sigs,prefix,elId){const el=document.getElementById(elId);const entries=Object.entries(sigs).sort((a,b)=>+a[0]-+b[0]);if(!entries.length){el.innerHTML=`<div class="empty">Kein ${prefix}</div>`;return;}el.innerHTML=entries.map(([idx,val])=>{const on=val===true||val==='TRUE'||val===1;return`<div class="sr"><div class="led ${on?'on':'off'}"></div><span class="sn">${prefix}[${idx}]</span><span class="sv ${on?'on':'off'}">${on?'TRUE':'FALSE'}</span></div>`;}).join('');}
function renderAnalog(sigs){const el=document.getElementById('anout-list');const entries=Object.entries(sigs).sort((a,b)=>+a[0]-+b[0]);if(!entries.length){el.innerHTML='<div class="empty">Kein $ANOUT</div>';return;}el.innerHTML=entries.map(([idx,v])=>{const pct=Math.abs(v)/10*50;const fill=v>=0?`width:${pct}%;left:50%`:`width:${pct}%;left:${50-pct}%`;return`<div class="ar"><div class="ah"><span class="an">$ANOUT[${idx}]</span><span class="av">${v>=0?'+':''}${v.toFixed(2)} V</span></div><div class="atrack"><div class="amid"></div><div class="afill ${v>=0?'pos':'neg'}" style="${fill}"></div></div></div>`;}).join('');}

// ── Live update when editing position coordinates ──────────
function liveEditUpdate() {
  const x=parseFloat(document.getElementById('ep-x').value)||0;
  const y=parseFloat(document.getElementById('ep-y').value)||0;
  const z=parseFloat(document.getElementById('ep-z').value)||0;
  const a=parseFloat(document.getElementById('ep-a').value)||0;
  const b=parseFloat(document.getElementById('ep-b').value)||0;
  const cv=parseFloat(document.getElementById('ep-c').value)||0;

  // Update 3D marker + coordinate frame live
  if (selectedPosIdx!==null) {
    const grp=posGrp.children[selectedPosIdx];
    if(grp){grp.position.set(x,y,z);grp.setRotationFromEuler(kukaEuler(a,b,cv));}
    selSphere.position.set(x,y,z);
    markerGrp.position.set(x,y,z);markerGrp.setRotationFromEuler(kukaEuler(a,b,cv));markerGrp.visible=true;
    // Update stored position so FK error is computed correctly
    parsedData.positions[selectedPosIdx] = {
      ...parsedData.positions[selectedPosIdx], X:x,Y:y,Z:z,A:a,B:b,C:cv
    };
  }

  // Compute IK + update robot
  const res=solveIK(x,y,z,a,b,cv);
  if(res.ok) applyAngles(res.angles);
  document.getElementById('rb-fk').textContent=res.score.toFixed(2);

  // Compute and show all unique solutions
  showEpIKSolutions(x,y,z,a,b,cv);
}

function showEpIKSolutions(x,y,z,a,b,cv) {
  const configs=[
    {label:'Elbow Up · Vorne',   starts:[[0,-90,90,0,0,0],[0,-90,90,0,-45,0]]},
    {label:'Elbow Up · Hinten',  starts:[[180,-90,90,0,0,0],[180,-90,90,0,-45,0]]},
    {label:'Elbow Up · Flip',    starts:[[0,-90,90,180,-45,180],[0,-90,90,-180,-45,-180]]},
    {label:'Elbow Down · Vorne', starts:[[0,-120,110,0,0,0],[0,-60,60,0,0,0]]},
    {label:'Elbow Down · Hinten',starts:[[180,-120,110,0,0,0],[180,-60,60,0,0,0]]},
    {label:'Elbow Down · Flip',  starts:[[0,-120,110,180,-45,180],[0,-60,60,-180,-45,-180]]},
    {label:'Schulter Alt 1',     starts:[[45,-90,90,0,0,0],[-45,-90,90,0,0,0]]},
    {label:'Schulter Alt 2',     starts:[[90,-90,90,0,0,0],[-90,-90,90,0,0,0]]},
  ];

  const cur=jointAngles.slice();
  const solutions=[];

  for(const cfg of configs){
    let best=null;
    for(const start of cfg.starts){
      const res=solveIK(x,y,z,a,b,cv,start);
      if(!best||res.score<best.score) best={...res,label:cfg.label};
    }
    if(!best||best.score>20) continue;
    // Deduplication: skip if angles within 0.5° of existing solution
    const isDup=solutions.some(s=>s.angles.every((v,i)=>Math.abs(v-best.angles[i])<0.5));
    if(isDup) continue;
    const inLimit=best.angles.every((v,i)=>v>=JOINTS_DEF[i].min&&v<=JOINTS_DEF[i].max);
    const cost=Math.sqrt(best.angles.reduce((s,v,i)=>(s+(v-cur[i])**2),0));
    solutions.push({...best,inLimit,cost});
  }

  // Sort: in-limit first, then by cost
  solutions.sort((a,b)=>(!a.inLimit&&b.inLimit)?1:(!b.inLimit&&a.inLimit)?-1:a.cost-b.cost);
  if(solutions.length>0) solutions[0].isBest=true;

  const listEl=document.getElementById('ep-ik-list');
  const secEl=document.getElementById('ep-ik-solutions');
  if(!listEl){return;}
  secEl.style.display=solutions.length?'block':'none';

  // Store solutions globally for safe index-based access
  window._epSolutions = solutions;

  listEl.innerHTML = solutions.map(function(sol, i) {
    var badge = sol.inLimit
      ? '<span class="ep-sol-ok ok">' + (sol.isBest ? '★ ' : '') + 'OK</span>'
      : '<span class="ep-sol-ok lim">Limit</span>';
    var angleStr = sol.angles.map(function(v, j) {
      return 'A'+(j+1)+':'+v.toFixed(0)+'°';
    }).join(' ');
    return '<div class="ep-sol' + (sol.isBest ? ' best' : '') + '" data-sol-idx="' + i + '">'
      + '<span class="ep-sol-lbl">' + sol.label + '</span>'
      + '<span class="ep-sol-cost">' + sol.cost.toFixed(0) + '°</span>'
      + badge
      + '<div style="font-size:.7em;color:var(--txt3);width:100%;margin-top:2px">' + angleStr + '</div>'
      + '</div>';
  }).join('');

  // Attach click handlers directly (no inline JSON)
  listEl.querySelectorAll('.ep-sol').forEach(function(el) {
    el.addEventListener('click', function() {
      var idx = parseInt(this.getAttribute('data-sol-idx'));
      var sols = window._epSolutions;
      if (!sols || idx < 0 || idx >= sols.length) return;
      // Highlight selected
      listEl.querySelectorAll('.ep-sol').forEach(function(e) { e.style.outline = 'none'; });
      this.style.outline = '2px solid var(--acc)';
      // FK → XYZABC → Editor schreiben
      applyEpSolution(sols[idx].angles);
    });
  });
}

function applyEpSolution(angles) {
  // Roboter auf neue Gelenkwinkel bewegen
  tweenToAngles(angles, 500);

  if (selectedPosIdx === null) return;
  var pos = parsedData.positions[selectedPosIdx];
  if (!pos) return;

  // FK → XYZABC berechnen
  var fkResult = fkAll(angles);
  var R = fkResult.tcp_rot;
  var B2 = -Math.asin(Math.max(-1, Math.min(1, R[2][0])));
  var cb2 = Math.cos(B2);
  var A2, C2;
  if (Math.abs(cb2) > 1e-6) {
    A2 = Math.atan2(R[1][0]/cb2, R[0][0]/cb2);
    C2 = Math.atan2(R[2][1]/cb2, R[2][2]/cb2);
  } else {
    A2 = 0;
    C2 = Math.atan2(-R[1][2], R[1][1]);
  }
  function toDeg(v) {
    var d = v * 180 / Math.PI;
    while (d > 180)  d -= 360;
    while (d <= -180) d += 360;
    return d;
  }
  var newA = toDeg(A2), newB = toDeg(B2), newC = toDeg(C2);
  var tcp = fkResult.pts[7];
  var newX = tcp[0], newY = tcp[1], newZ = tcp[2];

  // ep-Felder aktualisieren
  document.getElementById('ep-x').value = newX.toFixed(3);
  document.getElementById('ep-y').value = newY.toFixed(3);
  document.getElementById('ep-z').value = newZ.toFixed(3);
  document.getElementById('ep-a').value = newA.toFixed(3);
  document.getElementById('ep-b').value = newB.toFixed(3);
  document.getElementById('ep-c').value = newC.toFixed(3);

  // parsedData aktualisieren
  parsedData.positions[selectedPosIdx] = Object.assign({}, pos, {
    X: newX, Y: newY, Z: newZ, A: newA, B: newB, C: newC
  });
  ikTable[selectedPosIdx] = { angles: angles, score: 0, ok: true };
  document.getElementById('rb-fk').textContent = '0.00';

  // Direkt ins KRL schreiben (kein Apply nötig)
  var lineNum = pos.lineNum;
  if (lineNum === undefined) return;
  var ta = document.getElementById('code-input');
  var lines = ta.value.split(/\r?\n/)
  if (!lines[lineNum]) return;

  var newStr = '{X ' + newX.toFixed(3) + ', Y ' + newY.toFixed(3) +
               ', Z ' + newZ.toFixed(3) + ', A ' + newA.toFixed(3) +
               ', B ' + newB.toFixed(3) + ', C ' + newC.toFixed(3) +
               (pos.S !== null && pos.S !== undefined ? ', S ' + pos.S : '') +
               (pos.T !== null && pos.T !== undefined ? ', T ' + pos.T : '') + '}';

  lines[lineNum] = lines[lineNum].replace(/\{[^}]+\}/, newStr);
  ta.value = lines.join('\n');
  rebuildGutter();
}

function writeBackPosition(idx, x, y, z, a, b, c) {
  if (idx === null || idx === undefined) return;
  var pos = parsedData.positions[idx];
  if (!pos) return;
  // Update KRL editor
  var ta = document.getElementById('code-input');
  if (!ta) return;
  var lines = ta.value.split('\n');
  var lineNum = pos.lineNum;  // lineNum ist 0-basiert
  if (lineNum < 0 || lineNum >= lines.length) return;
  var line = lines[lineNum];
  // Replace coordinate values in the line
  var newCoords = 'X ' + x.toFixed(3) + ', Y ' + y.toFixed(3) +
    ', Z ' + z.toFixed(3) + ', A ' + a.toFixed(3) +
    ', B ' + b.toFixed(3) + ', C ' + c.toFixed(3);
  lines[lineNum] = line.replace(
    /X\s*[-\d.]+\s*,\s*Y\s*[-\d.]+\s*,\s*Z\s*[-\d.]+\s*,\s*A\s*[-\d.]+\s*,\s*B\s*[-\d.]+\s*,\s*C\s*[-\d.]+/,
    newCoords
  );
  ta.value = lines.join('\n');
  rebuildGutter();
}

function toggleSec(titleEl){titleEl.closest('.sec').classList.toggle('collapsed');}

// ═══════════════════════════════════════════════════
// KINEMATIK CONFIG UI
// ═══════════════════════════════════════════════════
function buildKinConfig(){
  const limEl=document.getElementById('joint-limits-ui');
  limEl.innerHTML=`
    <div class="joint-cfg" style="font-size:.68em;color:var(--txt3);font-weight:bold;border-bottom:1px solid var(--bdr)">
      <span></span><span style="text-align:center">Min [°]</span><span style="text-align:center">Max [°]</span><span></span><span></span>
    </div>`+
  JOINTS_DEF.map((j,i)=>`
    <div class="joint-cfg">
      <span class="jl">A${i+1}</span>
      <input id="jmin${i}" type="number" value="${j.min}" step="1" style="width:100%">
      <input id="jmax${i}" type="number" value="${j.max}" step="1" style="width:100%">
      <span style="color:var(--txt3);font-size:.75em">${j.axis}</span><span></span>
    </div>`).join('');

  const offEl=document.getElementById('joint-offsets-ui');
  offEl.innerHTML=`
    <div class="joint-cfg" style="font-size:.68em;color:var(--txt3);font-weight:bold;border-bottom:1px solid var(--bdr);margin-top:6px">
      <span></span><span style="text-align:center">X [mm]</span><span style="text-align:center">Y [mm]</span><span style="text-align:center">Z [mm]</span><span></span>
    </div>`+
  JOINTS_DEF.map((j,i)=>`
    <div class="joint-cfg">
      <span class="jl">A${i+1}</span>
      <input id="jox${i}" type="number" value="${j.off[0]}" step="1" style="width:100%">
      <input id="joy${i}" type="number" value="${j.off[1]}" step="1" style="width:100%">
      <input id="joz${i}" type="number" value="${j.off[2]}" step="1" style="width:100%">
      <span style="color:var(--txt3);font-size:.75em">${j.axis}</span>
    </div>`).join('');
}

function applyKinematicConfig(){
  JOINTS_DEF.forEach((j,i)=>{
    j.min=parseFloat(document.getElementById('jmin'+i).value)||j.min;
    j.max=parseFloat(document.getElementById('jmax'+i).value)||j.max;
    const ox=parseFloat((document.getElementById('jox'+i)&&document.getElementById('jox'+i).value));
    const oy=parseFloat((document.getElementById('joy'+i)&&document.getElementById('joy'+i).value));
    const oz=parseFloat((document.getElementById('joz'+i)&&document.getElementById('joz'+i).value));
    if(!isNaN(ox))j.off[0]=ox;
    if(!isNaN(oy))j.off[1]=oy;
    if(!isNaN(oz))j.off[2]=oz;
  });
  buildPivotChain();
  buildRobotModel(jointAngles);
  // Recompute IK + trajectory with new kinematics
  if(parsedData.positions.length>0){
    setStatus('paused','Kinematik geändert — IK wird neu berechnet…');
    setTimeout(()=>{
      computeIKTable(parsedData.positions);
      renderPositions(parsedData.positions);
      if(trajectory.length>0) buildTrajectory(parsedData.positions, ikTable);
      const angles=getIKAngles(simTToTrajT(sim.t));
      if(angles) applyAngles(angles);
      setStatus('paused','Kinematik übernommen · IK neu berechnet');
    },10);
  } else {
    setStatus('paused','Kinematik übernommen');
  }
}

function updateTCPDef(){
  TCP_DEF.x=parseFloat(document.getElementById('tcp-x').value)||0;
  TCP_DEF.y=parseFloat(document.getElementById('tcp-y').value)||0;
  TCP_DEF.z=parseFloat(document.getElementById('tcp-z').value)||0;
  TCP_DEF.a=parseFloat(document.getElementById('tcp-a').value)||0;
  TCP_DEF.b=parseFloat(document.getElementById('tcp-b').value)||0;
  TCP_DEF.c=parseFloat(document.getElementById('tcp-c').value)||0;
  buildRobotModel(jointAngles);
  // Recompute IK for all program positions with new TCP
  // so robot reaches same Cartesian points despite different TCP
  if (parsedData.positions.length > 0) {
    setStatus('paused','TCP geändert — IK wird neu berechnet…');
    setTimeout(() => {
      computeIKTable(parsedData.positions);
      renderPositions(parsedData.positions);
      if (trajectory.length > 0) {
        // Rebuild trajectory with new IK
        buildTrajectory(parsedData.positions, ikTable);
      }
      // Update robot to current simulation position
      const angles = getIKAngles(simTToTrajT(sim.t));
      if (angles) applyAngles(angles);
      setStatus('paused','TCP aktualisiert · IK neu berechnet');
    }, 10);
  }
}

// ═══════════════════════════════════════════════════
// PARSE & LOAD
// ═══════════════════════════════════════════════════
function parseAndLoad(){
  const code=document.getElementById('code-input').value;
  parsedData=parseKRL(code);const N=parsedData.positions.length;
  posLineNums=new Set(parsedData.positions.map(p=>p.lineNum).filter(n=>n!==undefined));
  for(const bp of[...breakpoints])if(!posLineNums.has(bp))breakpoints.delete(bp);
  buildGutter(code.split(/\r?\n/).length);
  const ps=document.getElementById('pos-s');ps.min=0;ps.max=Math.max(0,N-1);ps.step=0.001;ps.value=0;
  document.getElementById('pos-v').textContent=N>0?`1 / ${N}`:'— / —';
  stopSim(0);setStatus('stopped','STOPPED');document.getElementById('marker-info').style.display='none';deselectPosition();
  buildScene(parsedData.positions);
  // Pre-compute IK for all positions
  setStatus('paused','IK lädt…');
  setTimeout(()=>{
    computeIKTable(parsedData.positions);
    renderPositions(parsedData.positions);
    renderVariables(parsedData.finalState.variables);renderDigital(parsedData.finalState.digitalIn,'$IN','din-list');renderDigital(parsedData.finalState.digitalOut,'$OUT','dout-list');renderAnalog(parsedData.finalState.analogOut);
    if(N>0){applySimT(0);sim.stepIdx=0;if((parsedData.steps&&parsedData.steps.length))applyStep(0);else setStatus('paused','BEREIT');}
    else setStatus('stopped','STOPPED');
  },10);
}

document.getElementById('parse-btn').addEventListener('click',parseAndLoad);

// ═══════════════════════════════════════════════════
// INIT
// ═══════════════════════════════════════════════════
splashProgress(30, 'Kinematik wird geladen…');
buildKinConfig();
buildSteuerAxes();
buildAxisSTLUI();
splashProgress(60, 'Robotermodell wird erstellt…');
buildRobotModel([0,-90,90,0,0,0]);

document.getElementById('code-input').value=`DEF NONAME()
GLOBAL INTERRUPT DECL 3 WHEN $STOPMESS==TRUE DO IR_STOPM ( )
INTERRUPT ON 3
BAS (#INITMOV,0)
$CIRC_TYPE = #PATH
BAS (#VEL_PTP,20)
BAS (#ACC_PTP,20)
$APO.CDIS = 0.5000
$BASE = $WORLD
;$BASE={X 0, Y 0, Z 0, A 0, B 0, C 0}
$ACT_BASE=0
$TOOL=TOOL_DATA[24]
;$TOOL={X 374.5, Y 0, Z 46.5, A 0, B 90, C 0}
$ACT_TOOL=24
$advance=5
$VEL.CP=0.167
PTP {A1 -19.000, A2 -56.735, A3 134.892, A4 -92.636, A5 89.448, A6 168.170}
$VEL.CP=0.003
LIN {X 730, Y 385, Z 10, A -73.693, B 0, C 180} C_DIS
LIN {X 730, Y 385, Z 0, A -73.693, B 0, C 180} C_DIS
LIN {X 1150, Y 385, Z 0, A -82.99, B 0, C 180} C_DIS
LIN {X 1150, Y -175, Z 0, A -110.153, B 0, C 180} C_DIS
LIN {X 730, Y -175, Z 0, A -114.981, B 0, C 180} C_DIS
LIN {X 730, Y 385, Z 0, A -73.693, B 0, C 180} C_DIS
$VEL.CP=0.167
LIN {X 730, Y 385, Z 10, A -73.693, B 0, C 180} C_DIS
PTP {A1 0.000, A2 -90.000, A3 90.000, A4 0.000, A5 0.000, A6 0.000}
END`;


// STL-Dateien werden per fetch aus ./stl/ geladen
function xhrSTL(url, onDone, onErr) {
  // Absolute URL aufbauen um Pfad-Probleme zu vermeiden
  var base = window.location.href.replace(/\/[^\/]*$/, '/');
  var absUrl = url.indexOf('http') === 0 ? url : base + url.replace(/^\.\//,'');
  var xhr = new XMLHttpRequest();
  xhr.open('GET', absUrl, true);
  xhr.responseType = 'arraybuffer';
  xhr.timeout = 60000;
  xhr.onload = function() {
    if ((xhr.status >= 200 && xhr.status < 300) || xhr.status === 0) {
      if (xhr.response && xhr.response.byteLength > 0) {
        onDone(xhr.response);
      } else {
        console.warn('XHR leer:', absUrl);
        if(onErr) onErr('empty response');
      }
    } else {
      console.warn('XHR Status ' + xhr.status + ':', absUrl);
      if(onErr) onErr('HTTP ' + xhr.status);
    }
  };
  xhr.onerror   = function(e) { console.error('XHR Netzwerkfehler:', absUrl, e); if(onErr) onErr('network error'); };
  xhr.ontimeout = function()  { console.error('XHR Timeout:', absUrl); if(onErr) onErr('timeout'); };
  xhr.send();
}

function loadDefaultSTLs() {
  var axes = ['A1','A2','A3','A4','A5','A6'];
  var loaded = 0;
  var mi = document.getElementById('marker-info');
  if (mi) { mi.style.display='block'; mi.textContent='Lade STL Modelle...'; }
  var i = 0;
  function next() {
    if (i >= axes.length) {
      if (mi) { mi.style.display = loaded > 0 ? 'none' : 'block'; if(loaded===0) mi.textContent='STL nicht geladen — ↺ STL klicken'; }
      return;
    }
    var ax = axes[i++];
    var idx = parseInt(ax.replace('A','')) - 1;
    if (mi) mi.textContent = 'Lade ' + ax + '...';
    xhrSTL('./stl/' + ax.toLowerCase() + '.stl',
      function(buf) {
        try {
          window['_axisSTLBuffer'+idx] = buf;
          // Also keep a copy in the array for safe access
          if (!window._axisSTLBuffers) window._axisSTLBuffers = {};
          window._axisSTLBuffers[idx] = buf;
          var geo = stlLoader.parse(buf);
          geo.computeVertexNormals();
          if (axisSTLMeshes[idx]) { scene.remove(axisSTLMeshes[idx]); axisSTLMeshes[idx].geometry.dispose(); }
          axisSTLMeshes[idx] = new THREE.Mesh(geo, new THREE.MeshPhongMaterial({color:0xe8a020,shininess:80}));
          scene.add(axisSTLMeshes[idx]);
          // Mode anwenden falls schon gesetzt
          if (!axisSTLMode[idx]) axisSTLMode[idx] = 'solid';
          setAxisSTLMode(idx, axisSTLMode[idx]);
          var nameEl = document.getElementById('asl-name'+idx);
          if (nameEl) nameEl.textContent = ax.toLowerCase();
          var delEl = document.getElementById('asl-del'+idx);
          if (delEl) delEl.style.display = '';
          loaded++;
          buildRobotModel(jointAngles);
        } catch(e) { console.error('STL parse:', ax, e); }
        next();
      },
      function(e) { console.warn('STL load error:', ax, e); next(); }
    );
  }
  next();
}


// Podest + Tool per fetch laden
function loadDefaultSceneSTLs() {
  xhrSTL('./stl/podest.stl', function(buf) {
    try {
      window._pedestalSTLBuffer = buf;
      var geo = stlLoader.parse(buf); geo.computeVertexNormals();
      if (pedestalMesh) { scene.remove(pedestalMesh); pedestalMesh.geometry.dispose(); }
      pedestalMesh = new THREE.Mesh(geo, new THREE.MeshPhongMaterial({color:0x334455,shininess:40}));
      scene.add(pedestalMesh);
      var el = document.getElementById('pedestal-name');
      if (el) el.textContent = 'podest';
      var vb = document.getElementById('vis-pedestal');
      if (vb) vb.innerHTML = svgIconSolid;
    } catch(e) { console.error('Podest parse:', e); }
  }, function(e) { console.warn('Podest load:', e); });

  xhrSTL('./stl/tool1_tcp.stl', function(buf) {
    try {
      window._toolSTLBuffer = buf;
      var geo = stlLoader.parse(buf); geo.computeVertexNormals();
      if (toolMesh) { scene.remove(toolMesh); toolMesh.geometry.dispose(); toolMesh.material.dispose(); }
      var mat = new THREE.MeshPhongMaterial({color:0xdd9944,transparent:false,opacity:1.0,side:THREE.DoubleSide,specular:0x666666});
      toolMesh = new THREE.Mesh(geo, mat);
      scene.add(toolMesh);
      document.getElementById('tool-filename').textContent = 'Tool1_TCP';
      var tvb = document.getElementById('vis-tool');
      if (tvb) tvb.innerHTML = svgIconSolid;
      document.getElementById('tool-controls').style.display = 'block';
      buildRobotModel(jointAngles);
    } catch(e) { console.error('Tool parse:', e); }
  }, function(e) { console.warn('Tool load:', e); });
}


function toggleToolMesh() {
  showToolMesh = !showToolMesh;
  document.getElementById('btn-show-tool').classList.toggle('on', showToolMesh);
  if (toolMesh) toolMesh.visible = showRobot3D && showToolMesh;
}
function togglePedestalMesh() {
  showPedestalMesh = !showPedestalMesh;
  document.getElementById('btn-show-pedestal').classList.toggle('on', showPedestalMesh);
  if (pedestalMesh) pedestalMesh.visible = showPedestalMesh;
}

// ── 3-Panel Resize (Editor | Viewport | Parameter) ────────
(function(){
  function makeResizer(handleId, getPanel, getDir, saveKey) {
    var handle = document.getElementById(handleId);
    if (!handle) return;
    var dragging = false, startX = 0, startW = 0;
    handle.addEventListener('mousedown', function(e){
      var panel = getPanel();
      if (!panel) return;
      dragging = true; startX = e.clientX; startW = panel.offsetWidth;
      document.body.style.cursor = 'ew-resize';
      document.body.style.userSelect = 'none';
      e.preventDefault();
    });
    window.addEventListener('mousemove', function(e){
      if (!dragging) return;
      var panel = getPanel();
      if (!panel) return;
      var delta = (e.clientX - startX) * getDir();
      var w = Math.max(120, Math.min(800, startW + delta));
      panel.style.width = w + 'px';
      resize();
    });
    window.addEventListener('mouseup', function(){
      if (!dragging) return;
      dragging = false;
      document.body.style.cursor = '';
      document.body.style.userSelect = '';
      resize();
      // Breite in Settings speichern
      var panel = getPanel();
      if (panel && saveKey) {
        try {
          var s = JSON.parse(localStorage.getItem(SETTINGS_KEY) || '{}');
          s[saveKey] = panel.offsetWidth;
          localStorage.setItem(SETTINGS_KEY, JSON.stringify(s));
        } catch(e) {}
      }
    });
  }
  // Left handle: editor
  makeResizer('ep-resize-handle', function(){ return document.querySelector('.ep'); }, function(){ return 1; }, 'epWidth');
  // Right handle: info-panel
  makeResizer('ip-resize-handle', function(){ return document.getElementById('info-panel'); }, function(){ return -1; }, 'ipWidth');

  // Gespeicherte Breiten wiederherstellen
  window.addEventListener('load', function(){
    try {
      var s = JSON.parse(localStorage.getItem(SETTINGS_KEY) || '{}');
      if (s.epWidth) {
        var ep = document.querySelector('.ep');
        if (ep) { ep.style.width = s.epWidth + 'px'; resize(); }
      }
      if (s.ipWidth) {
        var ip = document.getElementById('info-panel');
        if (ip) { ip.style.width = s.ipWidth + 'px'; resize(); }
      }
    } catch(e) {}
  });
})();

// ══════════════════════════════════════════════════════════
// EINSTELLUNGSFENSTER — localStorage-Persistenz
// ══════════════════════════════════════════════════════════
var SETTINGS_KEY = 'robsim_settings';

var defaultSettings = {
  fzEditor: 17,
  fzUi:     17,
  fzPanel:  17,
  fzStatus: 17
};

function loadSettings() {
  try {
    var s = localStorage.getItem(SETTINGS_KEY);
    return s ? Object.assign({}, defaultSettings, JSON.parse(s)) : Object.assign({}, defaultSettings);
  } catch(e) { return Object.assign({}, defaultSettings); }
}

function saveSettings() {
  var s = {
    fzEditor: parseInt(document.getElementById('fz-editor').value) || 17,
    fzUi:     parseInt(document.getElementById('fz-ui').value)     || 17,
    fzPanel:  parseInt(document.getElementById('fz-panel').value)  || 17,
    fzStatus: parseInt(document.getElementById('fz-status').value) || 17
  };
  try { localStorage.setItem(SETTINGS_KEY, JSON.stringify(s)); } catch(e) {}
  applyFZ();
}

function resetSettings() {
  try { localStorage.removeItem(SETTINGS_KEY); } catch(e) {}
  applySettingsToUI(defaultSettings);
  applyFZ();
}

function applySettingsToUI(s) {
  var fields = {
    'fz-editor': s.fzEditor, 'fz-ui': s.fzUi,
    'fz-panel': s.fzPanel, 'fz-status': s.fzStatus
  };
  Object.entries(fields).forEach(function(kv) {
    var el = document.getElementById(kv[0]);
    if (el) el.value = kv[1];
  });
}

function fzWheel(e, inp) {
  // Legacy — global wheel handler takes care of this now
}

function applyFZ() {
  var fzE = parseInt(document.getElementById('fz-editor').value) || 17;
  var fzU = parseInt(document.getElementById('fz-ui').value)     || 17;
  var fzP = parseInt(document.getElementById('fz-panel').value)  || 17;
  var fzS = parseInt(document.getElementById('fz-status').value) || 17;

  // Editor + Gutter: CSS-Variablen setzen → alles synchron
  var lh = Math.round(fzE * 1.6);
  document.documentElement.style.setProperty('--fsz', fzE + 'px');
  document.documentElement.style.setProperty('--lh',  lh  + 'px');
  var codeEl = document.getElementById('code-input');
  if (codeEl) { codeEl.style.fontSize = fzE + 'px'; codeEl.style.lineHeight = lh + 'px'; }
  var gut = document.getElementById('gutter');
  if (gut) { gut.style.fontSize = fzE + 'px'; gut.style.lineHeight = lh + 'px'; }

  // Alles mit fontSize setzen via universellen Ansatz:
  // Toolbar, Buttons, Header, alle Labels
  document.querySelectorAll(
    '.tb, .sbtn, .slbl, .sv, .sstatus, header, .htag, ' +
    '.vbtn, #view-bar, #parse-btn, #parse-area'
  ).forEach(function(el) { el.style.fontSize = fzU + 'px'; });

  // Alle floating Panels (Steuerung, Einstellungen, Hilfe, Achsenkarte)
  document.querySelectorAll(
    '#steuer-panel *, #settings-panel *, #help-panel *, #axis-map-panel *,' +
    '#steuer-panel, #settings-panel, #help-panel, #axis-map-panel'
  ).forEach(function(el) { el.style.fontSize = fzU + 'px'; });

  // Rechtes Info-Panel: alle Kindelemente
  var ip = document.getElementById('info-panel');
  if (ip) {
    ip.style.fontSize = fzP + 'px';
    ip.querySelectorAll('*').forEach(function(el) { el.style.fontSize = ''; });
    // Reset dann gezielt setzen
    ip.querySelectorAll(
      '.sec-t,.sec-b,.pt,label,.stl-lbl,.stl-inp,.stl-unit,.cfg-lbl,.cfg-row,' +
      '.pc,.pf,.vr,.sr,.ar,.ep-grid,.ep-lbl,.ep-inp,.ep-unit,input,select,button,span,div'
    ).forEach(function(el) { el.style.fontSize = fzP + 'px'; });
  }

  // Statusleiste + Roboterbar
  document.querySelectorAll('#robot-bar, #robot-bar *, .rb-val, .rb-dim, .scene-hint, #marker-info')
    .forEach(function(el) { el.style.fontSize = fzS + 'px'; });
}

function initSettings() {
  var s = loadSettings();
  applySettingsToUI(s);
  applyFZ();
}

function toggleSettings() {
  var p = document.getElementById('settings-panel');
  if (!p) return;
  p.style.display = (p.style.display === 'none' || p.style.display === '') ? 'block' : 'none';
}


// Drag for settings panel
(function() {
  var panel = document.getElementById('settings-panel');
  var handle = document.getElementById('settings-drag-handle');
  if (!panel || !handle) return;
  var dragging = false, ox = 0, oy = 0;
  handle.addEventListener('mousedown', function(e) {
    dragging = true;
    ox = e.clientX - panel.offsetLeft;
    oy = e.clientY - panel.offsetTop;
    panel.style.transform = 'none';
    e.preventDefault();
  });
  window.addEventListener('mousemove', function(e) {
    if (!dragging) return;
    panel.style.left = (e.clientX - ox) + 'px';
    panel.style.top  = (e.clientY - oy) + 'px';
  });
  window.addEventListener('mouseup', function() { dragging = false; });
})();


function clearTCPTrace() {
  tcpTracePoints.length = 0;
  tcpTraceGrp.clear();
}


function resetAll() {
  if (!confirm(t('confirm_reset'))) return;
  // Roboter Heimstellung
  jointAngles = [0,-90,90,0,0,0];
  applyAngles(jointAngles);
  // TCP Trace löschen
  clearTCPTrace();
  // Simulation stoppen
  pauseSim();
  setStatus('stopped', 'STOPPED');
  // Positionen/Variablen leeren
  parsedData = {positions:[], steps:[], finalState:{variables:{},digitalIn:{},digitalOut:{},analogOut:{}}};
  ikTable = [];
  trajectory = [];
  trajMax = 0;
  buildScene([]);
  renderPositions([]);
  renderVariables({});
  renderDigital({}, '$OUT', 'dout-list');
  renderDigital({}, '$IN',  'din-list');
  renderAnalog({});
  // Deselect
  deselectPosition();
  markerGrp.visible = false;
  // TCP zurücksetzen
  TCP_DEF = {x:364.5, y:0, z:46.5, a:0, b:90, c:0};
  document.getElementById('tcp-x').value = 364.5;
  document.getElementById('tcp-y').value = 0;
  document.getElementById('tcp-z').value = 46.5;
  document.getElementById('tcp-a').value = 0;
  document.getElementById('tcp-b').value = 90;
  document.getElementById('tcp-c').value = 0;
  // Kamera auf ISO
  currentView = 'iso';
  activeCam = perspCam;
  updateCamera();
}


function togglePosFrames() {
  showPosFrames = !showPosFrames;
  document.getElementById('btn-show-posframes').classList.toggle('on', showPosFrames);
  posGrp.visible = showPosFrames;
}
function toggleBaseFrame() {
  showBaseFrame = !showBaseFrame;
  document.getElementById('btn-show-baseframe').classList.toggle('on', showBaseFrame);
  baseFrameGrp.visible = showBaseFrame;
}
function toggleTCPMarker() {
  showTCPMarker = !showTCPMarker;
  document.getElementById('btn-show-tcpmarker').classList.toggle('on', showTCPMarker);
  markerVisuals.visible = showTCPMarker;
}


// Mausrad auch ohne Fokus: beim Hover-Element
(function() {
  var hoveredNumber = null;
  document.addEventListener('mouseover', function(e) {
    if (e.target && e.target.type === 'number') hoveredNumber = e.target;
  });
  document.addEventListener('mouseout', function(e) {
    if (e.target && e.target.type === 'number') hoveredNumber = null;
  });
  document.addEventListener('wheel', function(e) {
    var el = hoveredNumber;
    if (!el) return;
    e.preventDefault();
    var step = parseFloat(el.step) || 1;
    var val  = parseFloat(el.value) || 0;
    val += e.deltaY < 0 ? step : -step;
    var mn = el.min !== '' ? parseFloat(el.min) : -Infinity;
    var mx = el.max !== '' ? parseFloat(el.max) :  Infinity;
    el.value = Math.max(mn, Math.min(mx, val));
    el.dispatchEvent(new Event('input', {bubbles:true}));
    el.dispatchEvent(new Event('change', {bubbles:true}));
  }, {passive: false});
})();


// ── DP-Solver Parameter aus UI anwenden ────────────────────────
function dpSolverApplySettings() {
  function gn(id, def) {
    var el = document.getElementById(id);
    return el ? (parseFloat(el.value) || def) : def;
  }
  var wA4    = gn('dp-wA4', 4);
  var wA5    = gn('dp-wA5', 5);
  var wA6    = gn('dp-wA6', 0.25);
  var wSwitch= gn('dp-wSwitch', 5000);
  var copies = Math.round(gn('dp-a6copies', 1));
  var safety = gn('dp-safety', 3);
  var tcptol = gn('dp-tcptol', 2);
  var sing   = gn('dp-singthresh', 0.01);

  DPSolver.settings.wAxes        = [1, 1, 1, wA4, wA5, wA6];
  DPSolver.settings.wConfigSwitch = wSwitch;
  DPSolver.settings.safetyDeg    = safety;
  DPSolver.settings.maxTcpPosError= tcptol;
  DPSolver.settings.minSingularity= sing;

  // A6-Kopien: copies=1 → [-1,0,1], copies=2 → [-2,-1,0,1,2] etc.
  var a6 = [0];
  for (var k = 1; k <= copies; k++) { a6.push(k); a6.push(-k); }
  DPSolver.settings.a6Copies = a6;

  // Sofort neu berechnen
  if (parsedData && parsedData.positions.length > 0) {
    computeIKTable(parsedData.positions);
    var ampPanel = document.getElementById('axis-map-panel');
    if (ampPanel && ampPanel.classList.contains('visible')) ampBuild();
    document.getElementById('amp-info').textContent = 'DP-Solver: Parameter aktualisiert, Plan neu berechnet.';
  }
}


// ── Hochpräzisions IK (mehr Iterationen, kleiner Schritt) ────
function solveIKPrecise(tx, ty, tz, ta, tb, tc, initAngles) {
  var clamp = function(v,lo,hi){ return Math.max(lo,Math.min(hi,v)); };
  var tp = [tx, ty, tz];
  var Rt = rotZYX(ta, tb, tc);
  var dt = 0.2, lam = 0.5, tolP = 0.05, tolO = 0.05;
  var q = initAngles ? initAngles.slice() : jointAngles.slice();
  var bestScore = Infinity, bestQ = q.slice();
  for (var iter = 0; iter < 500; iter++) {
    var e = err6(q, tp, Rt);
    var eP = Math.sqrt(e[0]*e[0]+e[1]*e[1]+e[2]*e[2]);
    var eO = Math.sqrt(e[3]*e[3]+e[4]*e[4]+e[5]*e[5]);
    var score = eP + eO;
    if (score < bestScore) { bestScore = score; bestQ = q.slice(); }
    if (eP < tolP && eO < tolO) break;
    var J = [];
    for (var i = 0; i < 6; i++) {
      var q1 = q.slice(); q1[i] += dt;
      var e1 = err6(q1, tp, Rt);
      J.push([(e1[0]-e[0])/dt,(e1[1]-e[1])/dt,(e1[2]-e[2])/dt,
               (e1[3]-e[3])/dt,(e1[4]-e[4])/dt,(e1[5]-e[5])/dt]);
    }
    var JtJ = Array.from({length:6}, function(){ return Array(6).fill(0); });
    var Jte = Array(6).fill(0);
    for (var i = 0; i < 6; i++) {
      for (var r = 0; r < 6; r++) {
        Jte[i] += J[i][r] * e[r];
        for (var j = 0; j < 6; j++) JtJ[i][j] += J[i][r] * J[j][r];
      }
      JtJ[i][i] += lam;
    }
    var dq = solve6x6(JtJ, Jte);
    var step = Math.min(0.5, 3.0 / Math.max(1, bestScore));
    for (var i = 0; i < 6; i++) {
      if (!isFinite(dq[i])) continue;
      q[i] = clamp(q[i] - step*dq[i], JOINTS_DEF[i].min, JOINTS_DEF[i].max);
    }
  }
  return { angles: bestQ, score: bestScore, ok: bestScore < 5 };
}


// ── Settings-Listener (nach DOM-Ready gebunden) ──────────────
function bindSettingsEvents() {
  function bind(id, evt, fn) {
    var el = document.getElementById(id);
    if (el) el.addEventListener(evt, fn);
  }
  // Farben
  bind('cfg-link-col',  'input',  function(){ robotColor=hexToInt(this.value); buildRobotModel(jointAngles); });
  bind('cfg-link-col',  'change', function(){ robotColor=hexToInt(this.value); buildRobotModel(jointAngles); });
  bind('cfg-joint-col', 'input',  function(){ jointColor=hexToInt(this.value); buildRobotModel(jointAngles); });
  bind('cfg-joint-col', 'change', function(){ jointColor=hexToInt(this.value); buildRobotModel(jointAngles); });
  bind('cfg-tcp-col',   'input',  function(){ tcpColor=hexToInt(this.value);   buildRobotModel(jointAngles); });
  bind('cfg-tcp-col',   'change', function(){ tcpColor=hexToInt(this.value);   buildRobotModel(jointAngles); });
  bind('cfg-path-col',  'input',  function(){ pathCol=hexToInt(this.value); buildScene(parsedData.positions); });
  bind('cfg-path-col',  'change', function(){ pathCol=hexToInt(this.value); buildScene(parsedData.positions); });
  bind('cfg-trace-col',   'input',  function(){ tcpTraceGrp.clear(); tcpTracePoints.length=0; });
  bind('cfg-planned-col', 'input',  function(){ buildScene(parsedData.positions); });
  bind('cfg-planned-col', 'change', function(){ buildScene(parsedData.positions); });
  bind('cfg-show-tool',     'change', function(){ if(this.checked!==showToolMesh)    toggleToolMesh(); });
  bind('cfg-show-pedestal', 'change', function(){ if(this.checked!==showPedestalMesh) togglePedestalMesh(); });
  bind('cfg-trace-col', 'change', function(){ tcpTraceGrp.clear(); tcpTracePoints.length=0; });
  // Sichtbarkeit
  bind('cfg-show-trace',      'change', function(){ showTCPTrace=this.checked; if(!showTCPTrace)tcpTraceGrp.clear(); var b=document.getElementById('btn-tcp-trace'); if(b)b.classList.toggle('on',this.checked); });
  bind('cfg-show-skeleton',   'change', function(){ if(this.checked!==showSkeleton) toggleSkeleton(); });
  bind('cfg-show-stlrobot',   'change', function(){ if(this.checked!==showSTLRobot) toggleSTLRobot(); });
  bind('cfg-show-posframes',  'change', function(){ if(this.checked!==showPosFrames) togglePosFrames(); });
  bind('cfg-show-baseframe',  'change', function(){ if(this.checked!==showBaseFrame) toggleBaseFrame(); });
  bind('cfg-show-tcpmarker',  'change', function(){ if(this.checked!==showTCPMarker) toggleTCPMarker(); });
}


// ── Editor Sprache: KUKA / DRCODE ────────────────────────────
var editorLang = 'kuka';

function setEditorLang(lang) {
  editorLang = lang;
  var btnKuka   = document.getElementById('btn-lang-kuka');
  var btnDrcode = document.getElementById('btn-lang-drcode');
  var label     = document.getElementById('editor-lang-label');
  if (lang === 'kuka') {
    if (btnKuka)   { btnKuka.style.background   = 'var(--acc)'; btnKuka.style.color   = '#fff'; }
    if (btnDrcode) { btnDrcode.style.background = 'transparent'; btnDrcode.style.color = 'var(--txt3)'; }
    if (label) label.textContent = 'KRL Programm';
    // DRCODE → KUKA Konvertierung (wird später implementiert)
    // convertDRCodeToKuka();
  } else {
    if (btnDrcode) { btnDrcode.style.background = 'var(--acc)'; btnDrcode.style.color   = '#fff'; }
    if (btnKuka)   { btnKuka.style.background   = 'transparent'; btnKuka.style.color = 'var(--txt3)'; }
    if (label) label.textContent = 'DRCODE Programm';
    // KUKA → DRCODE Konvertierung (wird später implementiert)
    // convertKukaToDRCode();
  }
}

splashProgress(80, 'Programm wird geparst…');
parseAndLoad();

// STL nach vollständigem Laden der Seite (inkl. Three.js CDN)
// STL wird manuell per '↺ STL' Button geladen
// Settings nach vollständigem DOM-Load anwenden
window.addEventListener('load', function() {
  splashProgress(95, 'Einstellungen werden geladen…');
  initSettings();
  bindSettingsEvents();
  splashProgress(100, 'Bereit.');
  setTimeout(splashHide, 400);
});