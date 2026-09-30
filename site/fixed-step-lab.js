import { scenarioLog } from "./scenario-log.mjs";
import {createWebGlRenderer} from './webgl-renderer.js';
import {physicsFailureMessage} from './physics-error.mjs';
const $=id=>document.getElementById(id);
const canvas=$('scene'),status=$('status'), controls=['solver','crates','projectile'];
const query=new URLSearchParams(location.search);
for(const id of controls)if([...$(id).options].some(o=>o.value===query.get(id)))$(id).value=query.get(id);
let e,renderer,paused=true,ticks=0,accumulator=0,last=null,error=null, recent=[],dirty=true,rows=[];
const approximate=()=>$('solver').value==='approximate';
const type=()=>({sphere:0,arrow:1,rigid:2}[$('projectile').value]);
let buildProvenance = null, pendingStepMs = [], attemptedSteps = 0;
scenarioLog.configure({
  environment: () => ({ build: buildProvenance, browser: navigator.userAgent,
    platform: navigator.platform, renderer: renderer?.backend ?? null,
    device_pixel_ratio: devicePixelRatio }),
  scenario: () => ({ id: "fixed-step", solver: approximate() ? "fixed-step-f64" : "sampled-event-f64",
    crate_motion: $("crates").value, projectile_type: $("projectile").value, warmup_steps: 240 }),
  onFailure(failure) {
    error = failure.message; paused = true; dirty = false;
    status.textContent = error;
    $("pause").textContent = "Resume";
    window.physicsLabState = { ...window.physicsLabState, error };
  },
});
function recordFrame(interval, started, rendered, renderMs) {
  scenarioLog.recorder.recordFrame({
    frame_interval_ms: interval, callback_ms: performance.now() - started,
    render_performed: rendered, render_ms: rendered ? renderMs : null,
    physics_steps_ms: pendingStepMs, body_count: rows.length, paused,
  });
  pendingStepMs = [];
}

const dot=(a,b)=>a.reduce((sum,v,i)=>sum+v*b[i],0);
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const norm=a=>{const l=Math.hypot(...a);return a.map(x=>x/l);};
const camera=[235,195,425],forward=norm(camera.map((x,i)=>[0,65,105][i]-x)),right=norm(cross(forward,[0,1,0])),up=cross(right,forward);
const rotate=(p,q)=>{const t=cross(q.slice(0,3),p).map(x=>2*x),u=cross(q.slice(0,3),t);return p.map((x,i)=>x+q[3]*t[i]+u[i]);};
const faces=[[[0,4,6,2],[-1,0,0]],[[1,3,7,5],[1,0,0]],[[0,1,5,4],[0,-1,0]],[[2,6,7,3],[0,1,0]],[[0,2,3,1],[0,0,-1]],[[4,5,7,6],[0,0,1]]];
function snapshot(){
  const approx=approximate();
  const ptr=approx?e.approximate_refresh_snapshot():e.sandbox_refresh_render_snapshot();
  const n=approx?e.approximate_snapshot_len():e.sandbox_render_snapshot_len();
  const data=approx?new Float64Array(e.memory.buffer,ptr,n):new Int32Array(e.memory.buffer,ptr,n);
  return Array.from({length:n/11},(_,i)=>Array.from(data.slice(i*11,i*11+11), (v,k)=>!approx&&k>=7?v/(2**30):v));
}
function render(){
  if (scenarioLog.failed) return;
  rows=snapshot(); const data=[];
  function vertex(p,n,uv,mat){const delta=p.map((v,i)=>v-camera[i]);data.push(dot(delta,right),dot(delta,up),dot(delta,forward),...n,...uv,mat);}
  for(const r of rows){
    const role=r[0],pos=r.slice(1,4),half=r.slice(4,7),q=r.slice(7);
    if(role===1)continue;
    const mat=role>=3?4:role===2?3:pos[1]<0&&half[0]>=400?0:half[1]>=60?1:2;
    if(role===3){
      const p=(i,j)=>{const a=-Math.PI/2+i*Math.PI/8,b=j*Math.PI/6;return [Math.cos(a)*Math.cos(b),Math.sin(a),Math.cos(a)*Math.sin(b)];};
      for(let i=0;i<8;i++)for(let j=0;j<12;j++)for(const [a,b] of [[i,j],[i+1,j],[i+1,j+1],[i,j],[i+1,j+1],[i,j+1]]){const n=p(a,b);vertex(n.map((v,k)=>pos[k]+v*half[0]),n,[0,0],mat);}continue;
    }
    const v=Array.from({length:8},(_,i)=>rotate(half.map((h,k)=>h*((i&(1<<k))?1:-1)),q).map((x,k)=>x+pos[k]));
    for(const [ids,n] of faces){const normal=rotate(n,q);const center=ids.map(i=>v[i]).reduce((a,b)=>a.map((x,k)=>x+b[k]/4),[0,0,0]);if(dot(normal,camera.map((x,k)=>x-center[k]))<=0)continue;
      for(const c of [0,1,2,0,2,3])vertex(v[ids[c]],normal,[[0,0],[1,0],[1,1],[0,1]][c],mat);
    }
  }
  const w=Math.round(canvas.clientWidth*devicePixelRatio),h=Math.round(canvas.clientHeight*devicePixelRatio);
  if(canvas.width!==w||canvas.height!==h){canvas.width=w;canvas.height=h;renderer.resize(w,h);}
  renderer.render(new Float32Array(data));
  const awake=rows.filter((r,i)=>r[0]===2&&(approximate()?e.approximate_body_sleeping(i):e.sandbox_body_sleeping(i))!==1).length;
  const mean=recent.length?recent.reduce((a,b)=>a+b,0)/recent.length:0;
  const text=error?error:`${approximate()?'Fixed step f64':'Sampled event f64'} · ${(ticks/60).toFixed(2)} s simulated · 32 crates · ${awake} awake · ${mean.toFixed(2)} ms/step (recent)${paused?' · paused':''}`;
  if(status.textContent!==text)status.textContent=text;
  window.physicsLabState={solver:$('solver').value,ticks,awake,error,bodyCount:rows.length};
  dirty=false;
}
function tick(){
  if (scenarioLog.failed) return;
  scenarioLog.begin("physics-step", { step_index: ++attemptedSteps, velocity: [0, 0], jump: 0, fixed_step_ms: 1000 / 60 });
  const start=performance.now(),code=approximate()?e.approximate_step_velocity(0,0,0):e.sandbox_step_velocity(0,0,0);
  const duration = performance.now()-start;
  pendingStepMs.push(duration); recent.push(duration);if(recent.length>60)recent.shift();
  if (code) {
    const detail = approximate() ? 0 : e.sandbox_error_detail();
    error = approximate() ? `Fixed-step failure ${code}` : physicsFailureMessage(code, detail);
    scenarioLog.fail(new Error(error), { source: "engine", engine_code: code, engine_detail: detail,
      pending_physics_steps_ms: pendingStepMs });
    return;
  }
  ticks++; dirty=true; scenarioLog.complete();
}
function reset(){
  try {
    scenarioLog.start(); pendingStepMs = []; attemptedSteps = 0; last = null;
    const started = performance.now();
    const rules=(1<<29)|((1<<11)-2)|(1<<14)|(2<<12)|($('crates').value==='upright'?1<<11:0);
    scenarioLog.begin("reset", { export: "sandbox_reset_tower_with_baking_options", arguments: [rules, 0, 1] });
    const resetCode = e.sandbox_reset_tower_with_baking_options(rules, 0, 1);
    if (resetCode !== 0) {
      scenarioLog.fail(new Error("Rust fixture reset failed"), { source: "engine", engine_code: resetCode });
      return;
    }
    if (approximate()) {
      scenarioLog.begin("solver-initialization", { export: "approximate_reset_from_sandbox", arguments: [4, 8] });
      const importCode = e.approximate_reset_from_sandbox(4, 8);
      if (importCode !== 0) {
        scenarioLog.fail(new Error("Approximation import failed"), { source: "engine", engine_code: importCode });
        return;
      }
    }
    ticks=0;error=null;recent=[];paused=true;accumulator=0;
    scenarioLog.complete();
    for(let i=0;i<240&&!error;i++)tick();
    if (scenarioLog.failed) return;
    recent=[];$('pause').textContent='Resume';
    $('method-title').textContent=approximate()?'Explicit approximation · 4 substeps · 8 impulse iterations':'Existing sampled-event solver · f64 arithmetic';
    $('method').textContent=approximate()?'Forces and impulses update velocity; the new velocity advances position and orientation. Contact manifolds retain accumulated impulses for the next substep. Fast projectiles use translation-only sweeps. Rapidly rotating continuous collisions are not exact.':'The reference repeatedly searches for sampled collision times, resolves the simultaneous contacts, stabilizes them, and searches the remaining time again. Its free-rotating direct-hit tower case is still known to exhaust the event budget.';
    const url=new URL(location.href);for(const id of controls)url.searchParams.set(id,$(id).value);history.replaceState(null,'',url);
    scenarioLog.begin("render"); const renderStarted = performance.now(); render();
    recordFrame(0, started, true, performance.now() - renderStarted); scenarioLog.complete();
  } catch (failure) {
    scenarioLog.fail(failure, { source: "reset", pending_physics_steps_ms: pendingStepMs });
  }
}
function shoot(miss){
  if(error || scenarioLog.failed)return;
  const set=approximate()?e.approximate_set_projectile_type:e.sandbox_set_projectile_type;
  const fire=approximate()?e.approximate_shoot:e.sandbox_shoot;
  scenarioLog.begin("shoot", { velocity: miss ? [40, 0, -87] : [0, 0, -96], projectile_type: $("projectile").value });
  if(set(type())!==0||fire(...(miss?[40,0,-87]:[0,0,-96]))<0){
    scenarioLog.recorder.recordMarker("projectile-rejected"); scenarioLog.complete();
    status.textContent='Projectile rejected; reset before changing type.';return;
  }
  scenarioLog.complete();
  paused=false;accumulator=0;$('pause').textContent='Pause';dirty=true;
}
function frame(t){
  try {
    if (!scenarioLog.failed) {
      const started = performance.now();
      const interval = last === null ? 0 : Math.max(0, t - last);
      const elapsed = Math.min(100, interval); last = t;
      if(!paused){accumulator+=elapsed;let steps=0;while(accumulator>=1000/60&&steps<4&&!paused){tick();accumulator-=1000/60;steps++;}}
      if (!scenarioLog.failed) {
        scenarioLog.begin("render");
        const renderStarted = performance.now(), rendered = dirty;
        if (dirty) render();
        recordFrame(interval, started, rendered, performance.now() - renderStarted);
        scenarioLog.complete();
      }
    }
  } catch (failure) {
    scenarioLog.fail(failure, { source: "animation-frame", pending_physics_steps_ms: pendingStepMs });
  }
  requestAnimationFrame(frame);
}
try{
  try {
    const provenanceResponse = await fetch("build-provenance.json", { cache: "no-store" });
    if (provenanceResponse.ok) buildProvenance = await provenanceResponse.json();
  } catch (failure) { console.warn("Build provenance unavailable", failure); }
  scenarioLog.recorder.refreshContext();
  const response=await fetch('physics_engine_demo.wasm');if(!response.ok)throw new Error(`WASM ${response.status}`);
  e=(await WebAssembly.instantiate(await response.arrayBuffer(),{})).instance.exports;
  renderer=createWebGlRenderer(canvas,{fovRadians:Math.PI/3,nearPlane:1,farPlane:2000});if(!renderer)throw new Error('WebGL2 is unavailable');
  scenarioLog.recorder.refreshContext();
  $('download-performance-log').onclick=()=>scenarioLog.download(scenarioLog.failed?scenarioLog.lastCrash:scenarioLog.recorder.capture());
  for(const id of controls)$(id).addEventListener('change',reset);
  $('reset').onclick=reset;$('hit').onclick=()=>shoot(false);$('miss').onclick=()=>shoot(true);
  $('pause').onclick=()=>{if(error)return;paused=!paused;accumulator=0;$('pause').textContent=paused?'Resume':'Pause';dirty=true;};
  for(const b of document.querySelectorAll('button'))b.disabled=false;
  new ResizeObserver(()=>{dirty=true;}).observe(canvas);reset();
}catch(err){status.textContent=`Unable to initialize comparison: ${err.message}`;window.physicsLabState={error:String(err)};scenarioLog.fail(err, { source: "initialization", pending_physics_steps_ms: pendingStepMs });}

// A failed warm-up must still allow an explicit reset without reloading the page.
if (e && renderer) requestAnimationFrame(frame);
