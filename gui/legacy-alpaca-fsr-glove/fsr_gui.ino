/* FSR Validation Glove — XIAO ESP32-S3
 * 8x FSR-402 (5 fingertips, 3 palm) -> WebSocket @ 50 Hz
 * GUI EMBEDDED in firmware — no LittleFS, no plugins, one-click upload.
 * Libs: ESP Async WebServer (v3.x) + AsyncTCP (v3.x)
 */
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>

/* ================= EMBEDDED GUI — edit only between the delimiters ================= */
static const char INDEX_HTML[] = R"HTML(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>FSR Glove — Thermographic Force Map</title>
<style>
:root{--bg:#04060f;--panel:#0d141dcc;--line:#1c2735;--txt:#dfe7f2;--dim:#7b8a9c;--acc:#37c8ff}
*{box-sizing:border-box;margin:0}
body{background:radial-gradient(1200px 800px at 60% -10%,#0a1220,var(--bg));color:var(--txt);
  font:15px/1.45 "Segoe UI",system-ui,sans-serif;min-height:100vh}
header{display:flex;align-items:center;gap:14px;padding:14px 22px;border-bottom:1px solid var(--line)}
h1{font-size:17px;font-weight:600} h1 small{color:var(--dim);font-weight:400;margin-left:10px}
.spacer{flex:1}
.pill{display:flex;align-items:center;gap:8px;padding:5px 12px;border:1px solid var(--line);
  border-radius:99px;font-size:13px;color:var(--dim)}
#dot{width:9px;height:9px;border-radius:50%;background:#e5533d;box-shadow:0 0 8px #e5533d}
#dot.ok{background:#3ddc84;box-shadow:0 0 8px #3ddc84}
.sim{font-size:13px;color:var(--dim);display:flex;gap:6px;align-items:center;cursor:pointer}
main{display:grid;grid-template-columns:minmax(340px,1.15fr) minmax(330px,1fr);gap:18px;
  padding:18px;max-width:1180px;margin:0 auto}
.card{background:var(--panel);border:1px solid var(--line);border-radius:14px;padding:16px;
  backdrop-filter:blur(6px)}
.hand-panel{display:flex;flex-direction:column;align-items:center;gap:8px}
#heat{width:100%;max-width:470px;height:auto;display:block}
.legend{width:82%;height:10px;border-radius:6px;border:1px solid #000;
  background:linear-gradient(90deg,#040620,#0a1450,#143cb4,#0096d7,#1ec86e,#d2dc3c,#fa9628,#eb3c19)}
.legend-row{width:82%;display:flex;justify-content:space-between;color:var(--dim);font-size:12px}
.caption{color:var(--dim);font-size:11px;letter-spacing:.06em;text-align:center}
.side{display:flex;flex-direction:column;gap:14px}
.stats{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.stat{background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:12px 14px}
.stat .k{font-size:11px;letter-spacing:.1em;text-transform:uppercase;color:var(--dim)}
.stat .v{font-size:26px;font-weight:650;margin-top:2px}
canvas.spark{width:100%;height:64px}
.brow{display:grid;grid-template-columns:108px 1fr 42px 48px;align-items:center;gap:8px;margin:5px 0;font-size:12px}
.blab{color:var(--dim);text-align:right;line-height:1.15}
.blab em{display:block;font-style:normal;color:var(--acc);font-size:10px;letter-spacing:.05em}
.btrack{height:10px;background:#111a24;border-radius:6px;overflow:hidden}
.bfill{height:100%;width:0%;border-radius:6px}
.braw{color:var(--dim);font-size:10px;text-align:right;font-variant-numeric:tabular-nums}
.bval{color:var(--txt);font-variant-numeric:tabular-nums;font-size:11px;text-align:right}
.controls{display:flex;flex-direction:column;gap:10px}
.btnrow{display:flex;gap:10px}
button{flex:1;padding:10px;border-radius:10px;border:1px solid var(--line);background:#13202e;
  color:var(--txt);font-size:14px;cursor:pointer}
button:hover{border-color:var(--acc)}
input[type=range]{width:100%}
#toast{position:fixed;left:50%;bottom:26px;transform:translateX(-50%);background:#13202e;
  border:1px solid var(--acc);padding:10px 18px;border-radius:10px;opacity:0;transition:opacity .3s;pointer-events:none}
#toast.show{opacity:1}
@media(max-width:820px){main{grid-template-columns:1fr}}
</style>
</head>
<body>
<header>
  <h1>FSR Validation Glove<small>thermographic force field · right hand · FSR-402 × XIAO ESP32-S3</small></h1>
  <div class="spacer"></div>
  <label class="sim"><input type="checkbox" id="simMode"> Simulation mode</label>
  <div class="pill"><span id="dot"></span><span id="connText">connecting…</span></div>
</header>

<main>
  <section class="card hand-panel">
    <canvas id="heat"></canvas>
    <div class="legend"></div>
    <div class="legend-row"><span>no contact</span><span>firm contact</span></div>
    <div class="caption">PER-ZONE READOUTS IN NEWTONS — LIVE FROM RAW ADC (FSR-402 R–F CURVE)<br>
      WIRING DEBUG: PRESS ONE SENSOR — THE PIN TAG THAT LIGHTS UP = ITS TRUE CHANNEL</div>
    <label class="sim"><input type="checkbox" id="showSensors" checked> show sensor positions + pin tags</label>
  </section>

  <aside class="side">
    <div class="stats">
      <div class="stat"><div class="k">Contacts</div><div class="v" id="stContacts">0</div></div>
      <div class="stat"><div class="k">Total force</div><div class="v" id="stTotal">0.0N</div></div>
      <div class="stat"><div class="k">Peak zone</div><div class="v" id="stPeak" style="font-size:16px">—</div></div>
      <div class="stat"><div class="k">Link rate</div><div class="v" id="stRate">— Hz</div></div>
    </div>
    <div class="card"><canvas class="spark" id="spark"></canvas></div>
    <div class="card" id="bars"></div>
    <div class="card controls">
      <div class="btnrow">
        <button id="btnTare">Tare (zero)</button>
        <button id="btnCal">Calibrate</button>
      </div>
      <label style="font-size:12px;color:var(--dim)">Contact threshold <span id="thrVal">8%</span>
        <input type="range" id="thr" min="2" max="40" value="8"></label>
      <label style="font-size:12px;color:var(--dim)">Field spread (prediction range) <span id="sprVal">1.0×</span>
        <input type="range" id="spread" min="60" max="170" value="100"></label>
      <label style="font-size:12px;color:var(--dim)">Force scale (bench trim) <span id="fsVal">1.00×</span>
        <input type="range" id="fscale" min="50" max="200" value="100"></label>
      <label style="font-size:12px;color:var(--dim)">Palm sensitivity (zones 6–8) <span id="pgVal">1.0×</span>
        <input type="range" id="palmgain" min="100" max="400" value="100"></label>
      <div style="font-size:12px;color:var(--dim)">Debug: the <b>raw</b> column shows live ADC counts per
        channel — press each physical sensor and note which pin's raw count climbs. Fix wiring, or tell me
        the true mapping and I'll add a one-line remap.</div>
    </div>
  </aside>
</main>
<div id="toast"></div>

<script>
/* Channel order = analog channel: index i reads PIN[i] on the XIAO.
   Pin tags are tied to the DATA index (the truth), not the anatomy —
   that's what exposes a crossed wire. */
let PINS=[0,1,2,3,4,5,8,9];                 // XIAO pins D0-D5, D8, D9 (A0-A5, A8, A9)
const pinTag=i=>`D${PINS[i]} / A${PINS[i]}`;
const ZONES=[
 {label:"Thumb Tip",  x:348,y:300, sigma:34},
 {label:"Index Tip",  x:260,y:128, sigma:30},
 {label:"Middle Tip", x:200,y:96,  sigma:30},
 {label:"Ring Tip",   x:144,y:128, sigma:29},
 {label:"Pinky Tip",  x:97, y:190, sigma:27},
 {label:"Thenar",     x:250,y:420, sigma:58},
 {label:"Distal Palm",x:243,y:333, sigma:55},
 {label:"Hypothenar", x:128,y:378, sigma:55}];
const N=ZONES.length;
const FING=[[260,318,260,128,44],[200,315,200,96,46],[144,318,144,128,42],
            [97,335,97,190,38],[272,385,348,300,50]];
const PALM="M292,305 L108,305 Q86,340 94,395 Q102,468 196,474 Q288,468 294,395 Q302,340 292,305 Z";

const $=id=>document.getElementById(id);
const target=new Float32Array(N), shown=new Float32Array(N), eff=new Float32Array(N),
      force=new Float32Array(N), rawNow=new Uint16Array(N), hist=[];
let thr=0.08, sim=false, ws=null, frames=0, showSens=true, fScale=1.0, palmGain=1.0;

const toastEl=$("toast"); let toastT;
function toast(m){toastEl.textContent=m;toastEl.classList.add("show");
  clearTimeout(toastT);toastT=setTimeout(()=>toastEl.classList.remove("show"),2600);}

/* ---------- REAL force conversion: raw ADC -> FSR resistance -> newtons ----------
   RDIV = the fixed divider resistor actually fitted on each channel, in ohms.
   If you swap the palm channels to a larger resistor (the proper hardware fix
   for weak palm response), change entries 5..7 here to match or every force
   number on those zones will be wrong. */
const RDIV=[10000,10000,10000,10000,10000, 10000,10000,10000];
const CURVE=[[0.2,120e3],[0.5,60e3],[1,30e3],[2,15e3],[5,6e3],[10,3e3],[20,1.6e3]];
function fsrForceN(raw,rdiv){
  if(raw<6)return 0;
  const R=(rdiv||10000)*(4095-raw)/raw;
  if(R>=CURVE[0][1])return 0;
  const L=CURVE[CURVE.length-1];
  if(R<=L[1])return L[0];
  for(let s=0;s<CURVE.length-1;s++){
    const A=CURVE[s],B=CURVE[s+1];
    if(R<=A[1]&&R>=B[1]){
      const t=(Math.log(R)-Math.log(A[1]))/(Math.log(B[1])-Math.log(A[1]));
      return Math.exp(Math.log(A[0])+t*(Math.log(B[0])-Math.log(A[0])));
    }
  }
  return 0;
}

/* ---------- thermal field setup ---------- */
const VW=400, VH=520, GS=0.35, GW=Math.round(VW*GS), GH=Math.round(VH*GS);
const heat=$("heat"); heat.width=VW*2; heat.height=VH*2;
const hctx=heat.getContext("2d"); hctx.scale(2,2);
hctx.imageSmoothingEnabled=true; hctx.imageSmoothingQuality="high";

const maskC=document.createElement("canvas"); maskC.width=GW; maskC.height=GH;
const mctx=maskC.getContext("2d");
mctx.setTransform(GS,0,0,GS,0,0);
mctx.strokeStyle="#fff"; mctx.fillStyle="#fff"; mctx.lineCap="round";
for(const f of FING){ mctx.lineWidth=f[4]; mctx.beginPath();
  mctx.moveTo(f[0],f[1]); mctx.lineTo(f[2],f[3]); mctx.stroke(); }
mctx.fill(new Path2D(PALM));
const maskA=mctx.getImageData(0,0,GW,GH).data;

const NC=10, NR=13, ng=new Float32Array((NC+1)*(NR+1));
for(let i=0;i<ng.length;i++)ng[i]=Math.random();
function vnoise(u,v){
  const x=Math.min(NC-1e-6,Math.max(0,u)), y=Math.min(NR-1e-6,Math.max(0,v));
  const x0=x|0, y0=y|0; let fx=x-x0, fy=y-y0;
  fx=fx*fx*(3-2*fx); fy=fy*fy*(3-2*fy);
  const a=ng[y0*(NC+1)+x0], b=ng[y0*(NC+1)+x0+1], c=ng[(y0+1)*(NC+1)+x0], d=ng[(y0+1)*(NC+1)+x0+1];
  return a+(b-a)*fx+(c-a)*fy+(a-b-c+d)*fx*fy;
}

const STOPS=[[0,4,6,32],[.12,10,20,80],[.25,20,60,180],[.40,0,150,215],[.55,30,200,110],
             [.70,210,220,60],[.82,250,150,40],[.93,235,60,25],[1,255,120,90]];
const LUT=new Uint8Array(256*3);
for(let i=0;i<256;i++){ const t=i/255; let s=0;
  while(s<STOPS.length-2 && t>STOPS[s+1][0])s++;
  const A=STOPS[s], B=STOPS[s+1], f=Math.min(1,Math.max(0,(t-A[0])/(B[0]-A[0])));
  LUT[i*3]=A[1]+(B[1]-A[1])*f; LUT[i*3+1]=A[2]+(B[2]-A[2])*f; LUT[i*3+2]=A[3]+(B[3]-A[3])*f; }

const base=new Float32Array(GW*GH);
for(let y=0;y<GH;y++)for(let x=0;x<GW;x++){ const k=y*GW+x;
  const cx=(x+.5)/GS, cy=(y+.5)/GS;
  const dp=(cx-210)*(cx-210)+(cy-390)*(cy-390);
  base[k]=0.10+0.10*vnoise(x/GW*NC,y/GH*NR)+0.06*Math.exp(-dp/(2*95*95)); }

let kernels=[];
function buildKernels(m){
  kernels=ZONES.map(z=>{ const s=z.sigma*m, a=new Float32Array(GW*GH);
    for(let y=0;y<GH;y++)for(let x=0;x<GW;x++){ const k=y*GW+x;
      const dx=(x+.5)/GS-z.x, dy=(y+.5)/GS-z.y;
      a[k]=Math.exp(-(dx*dx+dy*dy)/(2*s*s)); }
    return a; });
}
buildKernels(1.0);

const fieldC=document.createElement("canvas"); fieldC.width=GW; fieldC.height=GH;
const fctx=fieldC.getContext("2d");
const img=fctx.createImageData(GW,GH);

function renderHeat(){
  const d=img.data;
  for(let k=0;k<GW*GH;k++){
    let v=base[k];
    for(let i=0;i<N;i++)v+=eff[i]*kernels[i][k];
    const t=v>1?1:(v<0?0:v), li=(t*255)|0, o=k*4;
    d[o]=LUT[li*3]; d[o+1]=LUT[li*3+1]; d[o+2]=LUT[li*3+2];
    d[o+3]=maskA[k*4+3];
  }
  fctx.putImageData(img,0,0);
  hctx.clearRect(0,0,VW,VH);
  try{ hctx.save(); hctx.filter="blur(16px)"; hctx.globalAlpha=0.6;
       hctx.drawImage(fieldC,0,0,VW,VH); hctx.restore(); }catch(e){}
  hctx.drawImage(fieldC,0,0,VW,VH);

  if(showSens){ hctx.setLineDash([3,4]); hctx.strokeStyle="rgba(255,255,255,.35)";
    hctx.font="700 10px Segoe UI"; hctx.textAlign="center";
    for(let i=0;i<N;i++){ const z=ZONES[i];
      hctx.beginPath(); hctx.arc(z.x,z.y,10,0,7); hctx.stroke();
      hctx.lineWidth=3; hctx.strokeStyle="rgba(0,0,0,.6)";
      hctx.strokeText("D"+PINS[i], z.x, z.y-14);
      hctx.fillStyle="#9fdcff";
      hctx.fillText("D"+PINS[i], z.x, z.y-14);
      hctx.strokeStyle="rgba(255,255,255,.35)"; hctx.lineWidth=1.2;
    }
    hctx.setLineDash([]); }

  let peak=0,pk=0; for(let i=0;i<N;i++)if(shown[i]>peak){peak=shown[i];pk=i;}
  if(peak>=thr){ const z=ZONES[pk];
    hctx.strokeStyle="rgba(255,255,255,.85)"; hctx.lineWidth=1.2;
    hctx.beginPath(); hctx.moveTo(z.x-14,z.y); hctx.lineTo(z.x-5,z.y);
    hctx.moveTo(z.x+5,z.y); hctx.lineTo(z.x+14,z.y);
    hctx.moveTo(z.x,z.y-14); hctx.lineTo(z.x,z.y-5);
    hctx.moveTo(z.x,z.y+5); hctx.lineTo(z.x,z.y+14); hctx.stroke();
    hctx.fillStyle="#fff"; hctx.font="600 11px Segoe UI"; hctx.textAlign="left";
    hctx.fillText("D"+PINS[pk]+" "+force[pk].toFixed(1)+"N", z.x+16, z.y-10); }
}

/* ---------- side panel ---------- */
const barsG=$("bars");
const els=ZONES.map((z,i)=>{
  const row=document.createElement("div"); row.className="brow";
  row.innerHTML=`<span class="blab">${z.label}<em>${pinTag(i)}</em></span>
    <div class="btrack"><div class="bfill"></div></div>
    <span class="braw">0</span><span class="bval">0.0N</span>`;
  barsG.appendChild(row);
  return {fill:row.querySelector(".bfill"),bval:row.querySelector(".bval"),
          braw:row.querySelector(".braw")};
});
function barColor(p){return `hsl(${Math.round(230-230*Math.min(1,p))},95%,${Math.round(45+15*p)}%)`;}

/* ---------- comms ---------- */
function setStatus(ok,txt){$("dot").classList.toggle("ok",ok);$("connText").textContent=txt;}
function connect(){
  if(sim)return;
  const host=location.hostname||"192.168.4.1";
  ws=new WebSocket(`ws://${host}/ws`);
  ws.onopen =()=>setStatus(true,"live · "+host);
  ws.onmessage=e=>{try{const d=JSON.parse(e.data);
    if(d.t==='init'){                       // Auto-update pins from ESP32
      // Firmware sends raw GPIO numbers; tolerate "D9"-style strings too.
      PINS = d.pins.map(s => typeof s==='number' ? s : parseInt(String(s).replace(/^[A-Za-z]+/,'')));
      document.querySelectorAll('.blab em').forEach((el, i) => {
        if(PINS[i] !== undefined) el.textContent = `D${PINS[i]} / A${PINS[i]}`;
      });
    }
    if(d.p){target.set(d.p);frames++;}
    if(d.r)rawNow.set(d.r);
  }catch(_){}};
  ws.onclose=()=>{setStatus(false,"reconnecting…");if(!sim)setTimeout(connect,1200);};
  ws.onerror=()=>ws.close();
}
const send=o=>{if(ws&&ws.readyState===1)ws.send(JSON.stringify(o));};
setInterval(()=>{$("stRate").textContent=sim?"sim":frames+" Hz";frames=0;},1000);

function simulate(t){
  const c=(t*1.4)%10-1;
  for(let i=0;i<5;i++){target[i]=Math.exp(-Math.pow(i-c,2)/1.4)*0.95;
    rawNow[i]=Math.round(target[i]*3000);}
  for(let i=5;i<8;i++){target[i]=Math.max(0,Math.sin(t*0.8+i))*0.5;
    rawNow[i]=Math.round(target[i]*3000);}
}

const sp=$("spark"),sctx=sp.getContext("2d");
function sizeSpark(){sp.width=sp.clientWidth||300;sp.height=64;}
addEventListener("resize",sizeSpark);sizeSpark();
function drawSpark(){const w=sp.width,h=sp.height;sctx.clearRect(0,0,w,h);
  sctx.beginPath();
  hist.forEach((v,i)=>{const x=i/239*w,y=h-4-Math.min(1,v)*(h-10);
    i?sctx.lineTo(x,y):sctx.moveTo(x,y);});
  sctx.strokeStyle="#37c8ff";sctx.lineWidth=2;sctx.stroke();}

/* ---------- main loop ---------- */
let last=performance.now();
function tick(now){
  const dt=(now-last)/1000;last=now;
  if(sim)simulate(now/1000);
  let sum=0,peak=0,pk=0,contacts=0,sumF=0,peakF=0,pkF=0;
  for(let i=0;i<N;i++){
    // Palm zones (5,6,7) get an extra display gain on top of the firmware's
    // per-channel span/gamma. Applied to the NORMALISED value, and to the force
    // in newtons — never to the raw ADC count, because raw feeds the resistance
    // calculation and scaling it there reports physically false forces.
    const g = (i>=5) ? palmGain : 1;
    let tv = target[i]*g; if(tv>1) tv=1;
    shown[i]+=(tv-shown[i])*Math.min(1,dt*14);

    const f=(sim? target[i]*15 : fsrForceN(rawNow[i],RDIV[i])*g)*fScale;
    force[i]+=(f-force[i])*Math.min(1,dt*10);
    const p=shown[i];sum+=p;if(p>peak){peak=p;pk=i;}if(p>=thr)contacts++;
    sumF+=force[i]; if(force[i]>peakF){peakF=force[i];pkF=i;}
    els[i].fill.style.width=(p*100).toFixed(1)+"%";
    els[i].fill.style.background=barColor(p);
    els[i].bval.textContent=force[i].toFixed(1)+"N";
    els[i].braw.textContent=rawNow[i];
  }
  eff[0]=shown[0]+.15*shown[1];
  eff[1]=shown[1]+.15*(shown[0]+shown[2]);
  eff[2]=shown[2]+.15*(shown[1]+shown[3]);
  eff[3]=shown[3]+.15*(shown[2]+shown[4]);
  eff[4]=shown[4]+.15*shown[3];
  eff[5]=shown[5]+.30*Math.max(shown[6],shown[7]);
  eff[6]=shown[6]+.30*Math.max(shown[5],shown[7]);
  eff[7]=shown[7]+.30*Math.max(shown[5],shown[6]);
  for(let i=0;i<N;i++)if(eff[i]>1)eff[i]=1;

  $("stContacts").textContent=contacts;
  $("stTotal").textContent=sumF.toFixed(1)+"N";
  $("stPeak").textContent=peak>=thr?ZONES[pkF].label+" D"+PINS[pkF]+" · "+peakF.toFixed(1)+"N":"—";
  hist.push(sum/N);if(hist.length>240)hist.shift();
  drawSpark();
  renderHeat();
  drawZoneForces();
  requestAnimationFrame(tick);
}

/* per-zone live force + pin on the hand */
function drawZoneForces(){
  hctx.font="700 12px Segoe UI"; hctx.textAlign="center";
  for(let i=0;i<N;i++){
    const z=ZONES[i];
    if(force[i]>0.05){
      const txt="D"+PINS[i]+" "+force[i].toFixed(1)+"N";
      hctx.lineWidth=3; hctx.strokeStyle="rgba(0,0,0,.65)";
      hctx.strokeText(txt, z.x, z.y+4);
      hctx.fillStyle="#fff";
      hctx.fillText(txt, z.x, z.y+4);
    }
  }
}

/* ---------- controls ---------- */
$("btnTare").onclick=()=>{if(sim)return toast("Simulation mode — nothing to tare");
  send({cmd:"tare"});toast("Zeroing sensors… keep the glove unloaded");};
$("btnCal").onclick=()=>{if(sim)return toast("Simulation mode — nothing to calibrate");
  send({cmd:"cal"});toast("Press every sensor HARD for 3 seconds…");};
$("thr").oninput=e=>{thr=+e.target.value/100;$("thrVal").textContent=e.target.value+"%";};
$("spread").oninput=e=>{const m=+e.target.value/100;$("sprVal").textContent=m.toFixed(1)+"×";
  buildKernels(m);};
$("fscale").oninput=e=>{fScale=+e.target.value/100;$("fsVal").textContent=fScale.toFixed(2)+"×";};
/* Live palm trim: find the value that feels right here, then bake it into the
   firmware by dividing SPAN_DEF[5..7] by it, and reset this back to 1.0×. */
$("palmgain").oninput=e=>{palmGain=+e.target.value/100;$("pgVal").textContent=palmGain.toFixed(1)+"×";};
$("showSensors").onchange=e=>{showSens=e.target.checked;};
$("simMode").onchange=e=>{sim=e.target.checked;
  if(sim){if(ws)ws.close();setStatus(true,"simulation");}
  else{setStatus(false,"connecting…");connect();}};

connect();
requestAnimationFrame(tick);
</script>
</body>
</html>
)HTML";
/* ================= end embedded GUI ================= */

#define N_CH 8
// Order: Thumb, Index, Middle, Ring, Pinky, Thenar, DistalPalm, Hypothenar
const int PIN[N_CH] = { A9, A8, A2, A1, A0, A4, A3, A5 };  // D0-D5, D8, D9

/* ---------- PER-CHANNEL SENSITIVITY TRIM ----------
 * Channels 5,6,7 are the palm zones (Thenar, Distal Palm, Hypothenar).
 * They read low because soft palm flesh spreads load over the whole FSR pad
 * instead of concentrating it like a fingertip, so their resistance stays high.
 *
 * SPAN  = raw ADC counts above the tare point that count as "full scale".
 *         SMALLER = MORE SENSITIVE. This is the main knob.
 * GAMMA = curve shaping applied after normalising. SMALLER = light touches
 *         pushed harder toward full scale. 1.0 = linear, 0.4 = aggressive.
 * DEAD  = raw counts above tare ignored as ADC noise. Must be > your idle
 *         ripple or the low gamma will turn noise into phantom contacts.
 * NSAMP = ADC oversampling per frame. Palm channels get 4x more because a tiny
 *         DEAD only works if the noise floor is beaten down first.
 */
const float SPAN_DEF[N_CH] = { 2500, 2500, 2500, 2500, 2500,  300,  300,  300 };
const float GAMMA[N_CH]    = { 0.60f, 0.60f, 0.60f, 0.60f, 0.60f, 0.30f, 0.30f, 0.30f };
const float DEAD[N_CH]     = {   12,   12,   12,   12,   12,    3,    3,    3 };
const int   NSAMP[N_CH]    = {    8,    8,    8,    8,    8,   32,   32,   32 };
const uint32_t SPAN_MIN    = 120;   // floor so calibration can't divide by ~0

/* Slow baseline tracker. FSRs creep badly under the constant preload of a worn
 * glove — the resting reading drifts for minutes after donning. Without this
 * the drift eats the tiny DEAD band and the zone goes dead or sticks on. Only
 * creeps while the channel is essentially untouched, so a held press is safe. */
const float BASE_RATE = 0.0006f;   // ~30 s time constant at 50 Hz
const float BASE_GATE = 0.06f;     // only track when below 6% of span

const char* STA_SSID = "YOUR_WIFI_SSID";  // bogus on purpose -> falls back to hotspot
const char* STA_PASS = "prabhu44";
const char* AP_SSID = "FSR-Glove-Demo";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

float zero[N_CH], span[N_CH];
uint32_t calMin[N_CH], calMax[N_CH];
bool calActive = false, tareActive = false;
uint32_t calStart = 0, tareCount = 0;
float tareAcc[N_CH];
static const uint32_t FRAME_MS = 20;  // 50 Hz

uint32_t readAvg(int pin, int n) {
  uint32_t s = 0;
  for (int k = 0; k < n; k++) s += analogRead(pin);
  return s / n;
}

/* Blocking tare used at boot. The glove sits on the hand with a constant
 * preload, especially on the palm pads — if that offset isn't removed it eats
 * the bottom of the range and light touches never clear the deadband. */
void autoTare(int samples) {
  float acc[N_CH] = { 0 };
  for (int n = 0; n < samples; n++) {
    for (int i = 0; i < N_CH; i++) acc[i] += readAvg(PIN[i], NSAMP[i]);
    delay(5);
  }
  Serial.print("  AUTO-TARE baseline:");
  for (int i = 0; i < N_CH; i++) {
    zero[i] = acc[i] / samples;
    Serial.print(" "); Serial.print((int)zero[i]);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial monitor time to start
  analogReadResolution(12);
  for (int i = 0; i < N_CH; i++) {
    zero[i] = 0;
    span[i] = SPAN_DEF[i];
  }

  Serial.println("=========================================");
  Serial.println("  SENTRIX FSR GLOVE - BOOTING...");

  // Keep the glove UNLOADED for the first second after power-up.
  autoTare(20);
  
  // 1. Force AP-only mode
  WiFi.mode(WIFI_AP);
  
  // 2. CRITICAL: Set a Static IP. Prevents DHCP dropouts and OS confusion.
  IPAddress local_IP(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(local_IP, gateway, subnet);

  // 3. CRITICAL: Add a password! Modern phones block "Open" (NULL) networks.
  // Password MUST be between 8 and 64 characters.
  // SECURITY: set your own AP password here before flashing — do not commit
  // a real password to version control.
  const char* ap_password = "YOUR_WIFI_PASSWORD_HERE";

  // Start hotspot on Channel 6
  bool apStarted = WiFi.softAP("FSR-Glove-Demo", ap_password, 6, 0, 4);

  delay(1000);

  if (apStarted) {
    Serial.println("  DEMO WIFI: FSR-Glove-Demo");
    Serial.println("  PASSWORD:  (set in ap_password, not printed here)");
    Serial.print("  GUI URL:   http://"); 
    Serial.println(WiFi.softAPIP()); 
    Serial.println("  (Network is 2.4 GHz only)");
    Serial.println("=========================================");
  } else {
    Serial.println("ERROR: Failed to start WiFi AP. Check power/cable.");
    while(1) delay(1000); // Halt if Wi-Fi fails
  }

  // 4. Setup mDNS (Fallback, but rely on IP)
  if (MDNS.begin("fsrglove")) {
    MDNS.addService("http", "tcp", 80);
    Serial.println("mDNS started: http://fsrglove.local");
  }

  // 5. Serve the embedded GUI
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* r){ 
    r->send(200, "text/html", INDEX_HTML); 
  });

  server.onNotFound([](AsyncWebServerRequest *request){
    request->redirect("/");
  });

  // 6. WebSocket setup
  ws.onEvent([](AsyncWebSocket*, AsyncWebSocketClient* client, AwsEventType type,
                void*, uint8_t* data, size_t len){
    if(type == WS_EVT_CONNECT){
      String cfg = "{\"t\":\"init\",\"pins\":["; 
      for(int i=0; i<N_CH; i++) {
        cfg += String(PIN[i]);
        if(i < N_CH-1) cfg += ",";
      }
      cfg += "]}";
      client->text(cfg);
      Serial.println("WebSocket Client Connected!");
    }
    if(type!=WS_EVT_DATA) return;
    String m((const char*)data, len);
    if(m.indexOf("\"tare\"")>=0){ tareActive=true; tareCount=0;
      for(int i=0; i<N_CH; i++) tareAcc[i] = 0;
    }
    if(m.indexOf("\"cal\"")>=0){ calActive=true; calStart=millis();
      for(int i=0; i<N_CH; i++) { calMin[i] = 4095; calMax[i] = 0; }
    }
  });
  
  server.addHandler(&ws);
  server.begin();
  Serial.println("HTTP Server Started");
}

void loop() {
  static uint32_t last = 0;
  if (millis() - last < FRAME_MS) return;
  last = millis();

  uint32_t raw[N_CH];
  float p[N_CH];
  for (int i = 0; i < N_CH; i++) {
    raw[i] = readAvg(PIN[i], NSAMP[i]);
    if (tareActive) tareAcc[i] += raw[i];
    if (calActive) {
      if (raw[i] < calMin[i]) calMin[i] = raw[i];
      if (raw[i] > calMax[i]) calMax[i] = raw[i];
    }

    float above = (float)raw[i] - zero[i];

    // Creep/drift rejection: pull the baseline toward the reading only while
    // this channel is sitting near idle. A real press is far above the gate,
    // so holding one will not make it fade out.
    if (!tareActive && !calActive && above < BASE_GATE * span[i])
      zero[i] += (raw[i] - zero[i]) * BASE_RATE;

    // Subtract the tare baseline AND the per-channel noise deadband, then
    // rescale so the value still reaches 1.0 at full span.
    float d = above - DEAD[i];
    if (d <= 0) { p[i] = 0; continue; }

    float val = d / span[i];
    if (val > 1) val = 1;

    // Per-channel gamma: palm channels use a lower exponent so a light touch
    // produces a large output swing instead of crawling off the floor.
    p[i] = powf(val, GAMMA[i]);
  }
  if (tareActive && ++tareCount >= 15) {
    tareActive = false;
    for (int i = 0; i < N_CH; i++) zero[i] = tareAcc[i] / 15.0;
  }
  if (calActive && millis() - calStart >= 3000) {
    calActive = false;
    for (int i = 0; i < N_CH; i++) {
      uint32_t s = (calMax[i] > calMin[i]) ? (calMax[i] - calMin[i]) : 0;
      // If a channel was barely exercised during cal, keep its tuned default
      // rather than locking in a bogus span.
      span[i] = (s < SPAN_MIN) ? SPAN_DEF[i] : (float)s;
    }
  }

  char buf[384];
  int o = 0;
  o += snprintf(buf + o, sizeof(buf) - o, "{\"t\":%lu,\"p\":[", (unsigned long)millis());
  for (int i = 0; i < N_CH; i++) o += snprintf(buf + o, sizeof(buf) - o, "%.3f%s", p[i], i < N_CH - 1 ? "," : "");
  o += snprintf(buf + o, sizeof(buf) - o, "],\"r\":[");
  for (int i = 0; i < N_CH; i++) o += snprintf(buf + o, sizeof(buf) - o, "%u%s", raw[i], i < N_CH - 1 ? "," : "");
  snprintf(buf + o, sizeof(buf) - o, "]}");
  ws.textAll(buf);
}