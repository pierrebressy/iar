// Faux moteur 8th Wall : rend des images synthétiques de la cible et fournit des poses SLAM connues
(function(){
const G=Geom;
const q=qrcode(0,'M'); q.addData('AR-ANCHOR:A1'); q.make(); const N=q.getModuleCount();
const QS=0.12,FO=0.19,FI=0.17;
function dark(x,y){const a=Math.max(Math.abs(x),Math.abs(y)); if(a<=FO/2&&a>=FI/2) return true;
 const mx=(x+QS/2)/QS*N,my=(QS/2-y)/QS*N; return mx>=0&&mx<N&&my>=0&&my<N&&q.isDark(Math.floor(my),Math.floor(mx));}
const T_world_room={R:G.rodrigues([0.1,0.8,-0.05]),t:[0.4,-0.2,1.3]};
window.__truth={T_world_room};
const FLIP=[1,0,0,0,-1,0,0,0,-1];
let modules=[],canvas,frame=0;
function camAt(k){ // pose CV caméra dans le repère pièce
  const ang=Math.sin(k*0.05)*0.6, el=Math.sin(k*0.031)*0.3, d=0.5+0.2*Math.sin(k*0.07);
  const pos=[d*Math.sin(ang)*Math.cos(el), d*Math.sin(el), d*Math.cos(ang)*Math.cos(el)];
  const look=[0.01*Math.sin(k),0.01*Math.cos(k*1.3),0];
  const zc=G.sub(look,pos); const zn=G.scale(zc,1/G.norm(zc)); let xc=G.cross([0,-1,0],zn); xc=G.scale(xc,1/G.norm(xc)); const yc=G.cross(zn,xc);
  return {R:[xc[0],yc[0],zn[0],xc[1],yc[1],zn[1],xc[2],yc[2],zn[2]],t:pos};
}
function matToQuat(m){const tr=m[0]+m[4]+m[8];let w,x,y,z;
 if(tr>0){const s=Math.sqrt(tr+1)*2;w=0.25*s;x=(m[7]-m[5])/s;y=(m[2]-m[6])/s;z=(m[3]-m[1])/s;}
 else if(m[0]>m[4]&&m[0]>m[8]){const s=Math.sqrt(1+m[0]-m[4]-m[8])*2;w=(m[7]-m[5])/s;x=0.25*s;y=(m[1]+m[3])/s;z=(m[2]+m[6])/s;}
 else if(m[4]>m[8]){const s=Math.sqrt(1+m[4]-m[0]-m[8])*2;w=(m[2]-m[6])/s;x=(m[1]+m[3])/s;y=0.25*s;z=(m[5]+m[7])/s;}
 else {const s=Math.sqrt(1+m[8]-m[0]-m[4])*2;w=(m[3]-m[1])/s;x=(m[2]+m[6])/s;y=(m[5]+m[7])/s;z=0.25*s;}return {x,y,z,w};}
function tick(){
  frame++;
  const W=innerWidth,H=innerHeight; const cols=Math.round(1280*W/H), rows=1280; const fpx=1.1*rows/2*(W/H)*1.4; // ~ focale
  const Tcv=camAt(frame); const Tcv_room_inv=G.invert(Tcv);
  // image
  const px=new Uint8Array(cols*rows*4); const n=[0,0,1]; 
  for(let v=0;v<rows;v++)for(let u=0;u<cols;u++){
    const ray=[(u+0.5-cols/2)/fpx,(v+0.5-rows/2)/fpx,1]; const dr=G.mulMV(Tcv.R,ray); // dans repère pièce
    let g=200; if(dr[2]<0){const lam=-Tcv.t[2]/dr[2]; const x=Tcv.t[0]+lam*dr[0], y=Tcv.t[1]+lam*dr[1]; g=dark(x,y)?30:215;}
    g+= (Math.random()-0.5)*8; const k=(v*cols+u)*4; px[k]=px[k+1]=px[k+2]=g; px[k+3]=255;}
  // pose caméra Three dans le monde
  const Tthree_room={R:G.mulMM(Tcv.R,FLIP),t:Tcv.t}; const Tw=G.compose(T_world_room,Tthree_room);
  const P0=2*fpx/cols, P5=2*fpx/rows; const near=0.01,far=1000;
  const intr=[P0,0,0,0, 0,P5,0,0, 0,0,-(far+near)/(far-near),-1, 0,0,-2*far*near/(far-near),0];
  const reality={position:{x:Tw.t[0],y:Tw.t[1],z:Tw.t[2]},rotation:matToQuat(Tw.R),intrinsics:intr,trackingStatus:'NORMAL',trackingReason:'UNSPECIFIED'};
  const args={processCpuResult:{reality},processGpuResult:{camerapixelarray:{rows,cols,rowBytes:cols*4,pixels:px}}};
  for(const m of modules) if(m.onUpdate) m.onUpdate(args);
  if(frame<400) setTimeout(tick,30);
}
window.XR8={
  XrController:{configure(){},pipelineModule:()=>({name:'xr'})},
  GlTextureRenderer:{pipelineModule:()=>({name:'gl'})},
  CameraPixelArray:{pipelineModule:()=>({name:'cpa'})},
  XrConfig:{device:()=>({MOBILE:'mobile',ANY:'any'})},
  addCameraPipelineModules(a){modules.push(...a);},
  run(o){canvas=o.canvas;setTimeout(tick,50);},
};
setTimeout(()=>window.dispatchEvent(new Event('xrloaded')),10);
})();
