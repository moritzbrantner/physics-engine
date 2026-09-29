import {sourceIdentity} from './provenance.mjs';
// Calibration on the REAL room/player/tower adapter, with diagnostic-only reset budgets.
// State checks are outside timing. No renderer or production setting is changed.
import assert from 'node:assert/strict';
import {readFileSync,writeFileSync,appendFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import os from 'node:os';
import {createTowerRuntime} from '../../site/tower-runtime.mjs';
import {encodeScenarioRules,COLLISION_PAIRS} from '../../site/simulation-rules-config.mjs';
import {PROFILES,summary} from './report.mjs';
const [path,output]=process.argv.slice(2);assert(path&&output,'canonical.mjs diagnostic-demo.wasm result.json');
const data=readFileSync(path),module=await WebAssembly.compile(data),records=[];
const ids=(process.env.PROFILES??'4s-8v-2p,4s-6v-2p,3s-8v-2p,4s-4v-2p,2s-4v-1p,4s-8v-1p,4s-8v-0p').split(',');
const profiles=ids.map(id=>{const p=PROFILES.find(p=>p.id===id);assert(p);return p;});
const ticks=Number(process.env.TICKS??600),trials=Number(process.env.TRIALS??2);
assert(Number.isInteger(ticks)&&ticks>=600&&ticks<=3600&&Number.isInteger(trials)&&trials>=2&&trials<=6);
function rows(e){const p=e.sandbox_refresh_render_snapshot(),a=new Float64Array(e.memory.buffer,p,e.sandbox_render_snapshot_len());return Array.from({length:a.length/11},(_,i)=>Array.from(a.slice(i*11,i*11+11)));}
function bottom(r){const [x,y,z,w]=r.slice(7);return r[2]-Math.abs(2*(x*y+w*z))*r[4]-Math.abs(1-2*(x*x+z*z))*r[5]-Math.abs(2*(y*z-w*x))*r[6];}
writeFileSync(output+'.ndjson','');
for(let trial=0;trial<trials;trial++) {
 const cases=profiles.flatMap(p=>['impact-retire','physical'].flatMap(policy=>['repeated-hits','near-misses','immediate-arrow'].map(scene=>({p,policy,scene}))));
 if(trial%2)cases.reverse();
 for(const {p,policy,scene} of cases) {
  const raw=(await WebAssembly.instantiate(module,{})).exports;assert.equal(typeof raw.approximate_reset_tower_budget,'function');
  const e=createTowerRuntime({...raw,approximate_reset_tower:rules=>raw.approximate_reset_tower_budget(rules,p.substeps,p.velocity,p.position)});
  const rules=encodeScenarioRules({characterResponse:'physical',crateMotion:'free',projectileImpactPolicy:policy,enabledPairs:new Set(COLLISION_PAIRS.map(([id])=>id))});
  assert.equal(e.sandbox_reset_tower_with_baking_options(rules),0);
  const spawn=rows(e).filter(r=>r[0]===2),times={settling:[],active:[],sleeping:[]},hash=createHash('sha256'),failures=new Set();
  let peakFloor=0,initialDrift=0,changed=false,shots=0,steps=0,firstFailure=null,oldTime=e.elapsed();
  const initialTicks=scene==='immediate-arrow'?0:240;
  let initial=spawn;
  for(let t=-initialTicks;t<ticks;t++) {
   if(t===0)initial=rows(e).filter(r=>r[0]===2);
   if(t>=0 && (scene==='immediate-arrow'?t===0:t<480&&t%24===0)) {
    assert.equal(e.sandbox_set_projectile_type(scene==='immediate-arrow'?1:shots%3),0);
    const v=scene==='near-misses'?[40,0,-87]:[0,0,-96];
    if(e.sandbox_shoot(...v)<=0)failures.add('shot-rejected');shots++;
   }
   const quiet=e.sandbox_is_quiescent()===1,start=performance.now(),code=e.sandbox_step_velocity(0,0,0),ms=performance.now()-start;
   if(code!==0){failures.add('step-error');firstFailure??={tick:t,reason:'step-error',code};break;}
   times[t<0?'settling':quiet?'sleeping':'active'].push(ms);steps++;
   const all=rows(e),crates=all.filter(r=>r[0]===2);
   if(crates.length!==32||e.sandbox_body_count()-e.sandbox_projectile_count()!==44||!all.flat().every(Number.isFinite))failures.add('state-or-inventory');
   let awake=0;
   all.forEach((r,i)=>{if(Math.abs(Math.hypot(...r.slice(7))-1)>1e-8)failures.add('orientation');if(r[0]===2&&e.bodySleeping(i)!==1)awake++;});
   for(const [i,r] of crates.entries()) {
    peakFloor=Math.max(peakFloor,-bottom(r));
    if(t<0)initialDrift=Math.max(initialDrift,Math.hypot(r[1]-spawn[i][1],r[2]-spawn[i][2],r[3]-spawn[i][3]));
    if(t>=0)changed ||=r.slice(1).some((v,k)=>Math.abs(v-initial[i][k+1])>1e-6);
   }
   if(peakFloor>.5)failures.add('floor-penetration');
   if(initialDrift>1.8)failures.add('unforced-pre-shot-drift');
   if(t>=0&&scene==='near-misses'&&policy==='impact-retire'&&(awake||changed))failures.add('miss-disturbs-tower');
   if(Math.abs(e.elapsed()-oldTime-steps/60)>1e-7)failures.add('elapsed-time');
   const stats=e.stepStats();
   if(stats.fixed_substeps>p.substeps||stats.fixed_impulse_iterations>p.substeps*p.velocity||stats.fixed_position_passes>p.substeps*p.position)failures.add('work-budget');
   hash.update(JSON.stringify([all,all.map((_,i)=>e.bodySleeping(i)),stats]));
   if(failures.size&&!firstFailure)firstFailure={tick:t,reasons:[...failures]};
  }
  if(scene!=='near-misses'&&!changed)failures.add('no-hit-response');
  const r={trial,profile:p,scene,impact_policy:policy,boxes:32,fixed_and_player:12,completed:steps===ticks+initialTicks,peak_floor:peakFloor,initial_drift:initialDrift,shots,changed,
    quality_passed:failures.size===0,failures:[...failures],first_failure:firstFailure,timing:Object.fromEntries(Object.entries(times).map(([k,v])=>[k,summary(v)])),raw_ms:times,replay_sha256:hash.digest('hex')};
  records.push(r);appendFileSync(output+'.ndjson',JSON.stringify(r)+'\n');console.log(JSON.stringify({trial,profile:p.id,policy,scene,quality:r.quality_passed,floor:peakFloor,p95:r.timing.active.p95_ms}));
 }
}
for(const r of records)r.repeatable=records.filter(x=>x.profile.id===r.profile.id&&x.scene===r.scene&&x.impact_policy===r.impact_policy).every(x=>x.replay_sha256===r.replay_sha256);
writeFileSync(output,JSON.stringify({schema:'physics-canonical-budget/v1',runner_source:sourceIdentity(),ticks,trials,cpu:os.cpus()[0]?.model,node:process.version,wasm_sha256:createHash('sha256').update(data).digest('hex'),note:'Real tower adapter/fixture, diagnostic reset only; free crates/physical character. Sleeping allowed and phase-separated. Same established floor/inventory/hit/miss checks plus new1.8 pre-shot drift screening; this calibration does NOT measure pair overlap (scaling sweep does). Failures retained; no default change.',records},null,2));
if(records.some(r=>!r.repeatable))throw Error('canonical repeat mismatch');
if(records.some(r=>r.profile.id==='4s-8v-2p'&&!r.quality_passed))throw Error('Stable canonical reference failed; retain report and investigate');
