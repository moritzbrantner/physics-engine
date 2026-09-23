import {sourceIdentity} from './provenance.mjs';
// Real release-WASM step calls, with observations/hashing outside the timed interval.
import assert from 'node:assert/strict';
import {readFileSync,writeFileSync,appendFileSync,mkdirSync} from 'node:fs';
import {dirname} from 'node:path';
import {createHash} from 'node:crypto';
import os from 'node:os';
import {PROFILES,SCENES,THRESHOLDS,summary,qualityFailures,capacity} from './report.mjs';
const [wasmPath,output]=process.argv.slice(2);
if(!wasmPath||!output) throw Error('node experiments/solver-budget/run.mjs budget.wasm report.json');
const integers=(s,min,max)=>s.split(',').map(x=>{const n=Number(x);assert(Number.isInteger(n)&&n>=min&&n<=max);return n;});
const counts=integers(process.env.COUNTS??'16,32,64,128,256',8,2048);
assert(counts.every((n,i)=>(n&(n-1))===0&&(i===0||n>counts[i-1])),'COUNTS must be ascending powers of two');
const trials=integers(process.env.TRIALS??'3',1,9)[0];
const ticks=integers(process.env.TICKS??'300',240,3600)[0];
const scenes=integers(process.env.SCENES??'0,1,3',0,3);
const requested=process.env.PROFILES?.split(',');
const profiles=requested?requested.map(id=>{const p=PROFILES.find(p=>p.id===id);assert(p,`unknown profile ${id}`);return p;}):PROFILES;
const bytes=readFileSync(wasmPath),module=await WebAssembly.compile(bytes);
const workNames=['substeps','velocity_passes','position_passes','contact_points','pair_tests','narrow_tests','integrated_bodies','constraint_visits','capped_substeps','converged_substeps','position_contact_tests','position_corrections','swept_contacts'];
const diagnostic=(await WebAssembly.instantiate(module,{})).exports;
assert.equal(typeof diagnostic.budget_convergence_scope,'function','unidentified convergence policy');
const scope=diagnostic.budget_convergence_scope();assert(scope===0||scope===1);
const solverPolicy={convergence_scope:scope===1?'contact-islands':'whole-world',soft_contact:false};
const started=new Date().toISOString();mkdirSync(dirname(output),{recursive:true});
writeFileSync(output+'.ndjson','');
async function replay(count,scene,p,trial,warm=false) {
  const e=(await WebAssembly.instantiate(module,{})).exports;
  assert.equal(e.budget_reset(count,scene,p.substeps,p.velocity,p.position),0);
  const hash=createHash('sha256'),times={startup:[],active:[],sleeping:[]},raw=[];
  const peak=Array(12).fill(0), totals=Array(workNames.length).fill(0),phaseWork={startup:Array(workNames.length).fill(0),active:Array(workNames.length).fill(0),sleeping:Array(workNames.length).fill(0)};
  let failedStep=null,elapsed=0,firstQualityFailure=null,completedTicks=0,peakLive=0,contactBodySum=0,contactPairSum=0,qualitySamples=0;
  const failures=new Set();
  for(let t=0;t<(warm?40:ticks);t++) {
    const start=performance.now(),code=e.budget_step(),ms=performance.now()-start;
    if(code!==0){failedStep={tick:t,code};break;}
    completedTicks++;
    if(warm) continue;
    const ptr=e.budget_observe(),n=e.budget_snapshot_len(),state=new Float64Array(e.memory.buffer,ptr,n);
    const m=Array.from({length:12},(_,i)=>e.budget_metric(i));
    const work=workNames.map((_,i)=>e.budget_stat(i));elapsed=e.budget_stat(14);
    const phase=t<60?'startup':m[3]>0?'active':'sleeping';
    times[phase].push(ms);raw.push(ms);
    hash.update(new Uint8Array(state.buffer,ptr,n*8));hash.update(JSON.stringify([m,work]));
    for(let j=0;j<12;j++)peak[j]=Math.max(peak[j],m[j]);
    for(let j=0;j<work.length;j++){totals[j]+=work[j];phaseWork[phase][j]+=work[j];}
    peakLive=Math.max(peakLive,n/21);contactBodySum+=m[5];contactPairSum+=m[4];qualitySamples++;
    const reasons=qualityFailures(m,scene);
    if(work[0]>p.substeps || work[1]>p.substeps*p.velocity || work[2]>p.substeps*p.position)reasons.push('work-budget');
    if(Math.abs(elapsed-(t+1)/60)>1e-8)reasons.push('elapsed-time');
    if(scene!==2 && m[3]!==count)reasons.push('sustained-load-slept');
    for(const reason of reasons)failures.add(reason);
    if(reasons.length&&!firstQualityFailure)firstQualityFailure={tick:t+1,seconds:elapsed,reasons,metrics:m};
  }
  if(warm)return;
  if(failedStep)failures.add('step-error');
  if(scene===1&&(totals[12]===0||peak[6]<.01||peak[10]===0))failures.add('missing-impact-response');
  return {scene:SCENES[scene],boxes:count,fixed_bodies:1,peak_total_bodies:peakLive,profile:p,trial,completed:completedTicks===ticks,
    completed_ticks:completedTicks,elapsed_seconds:elapsed,step_error:failedStep,
    quality:{passed:!failures.size,reasons:[...failures],first_failure:firstQualityFailure,peak_floor:peak[1],peak_box_overlap:peak[2],peak_awake_boxes:peak[3],peak_touching_pairs:peak[4],peak_touching_boxes:peak[5],mean_touching_boxes:contactBodySum/qualitySamples,mean_touching_pairs:contactPairSum/qualitySamples,peak_displacement:peak[6],peak_surface_speed:peak[7],peak_energy:peak[8],shots:peak[10],retired:peak[11],sample_every_ticks:1},
    timing:Object.fromEntries(Object.entries(times).map(([k,v])=>[k,summary(v)])),raw_step_ms:raw,
    work:Object.fromEntries(workNames.map((k,i)=>[k,totals[i]])),phase_work:Object.fromEntries(Object.entries(phaseWork).map(([phase,v])=>[phase,Object.fromEntries(workNames.map((k,i)=>[k,v[i]]))])),
    replay_sha256:hash.digest('hex')};
}
// Warm the actual module/configurations before measured cases; retain all measured outliers.
for(const p of profiles)await replay(32,0,p,0,true);
const records=[];
for(let trial=0;trial<trials;trial++) {
  const configs=counts.flatMap(boxes=>scenes.flatMap(scene=>profiles.map(profile=>({boxes,scene,profile}))));
  // Reversed order every second trial; rotate the remainder to avoid permanently favoring a profile.
  if(trial%2)configs.reverse();const offset=Math.floor(configs.length*trial/trials);configs.push(...configs.splice(0,offset));
  for(const {boxes,scene,profile} of configs) {
    const r=await replay(boxes,scene,profile,trial);records.push(r);appendFileSync(output+'.ndjson',JSON.stringify(r)+'\n');
    console.log(JSON.stringify({trial,boxes,scene:r.scene,profile:profile.id,p95:r.timing.active.p95_ms,quality:r.quality.passed,failures:r.quality.reasons,overlap:r.quality.peak_box_overlap}));
  }
}
for(const r of records)r.repeatable=trials>=2&&records.filter(x=>x.scene===r.scene&&x.boxes===r.boxes&&x.profile.id===r.profile.id).every(x=>x.replay_sha256===r.replay_sha256);
const report={schema:'physics-solver-budget/v1',solver_policy:solverPolicy,runner_source:sourceIdentity(),historical_baseline:'3074f49ca2219b89cd093af6b6f8099bb387b887',wasm_sha256:createHash('sha256').update(bytes).digest('hex'),started,finished:new Date().toISOString(),environment:{node:process.version,platform:process.platform,arch:process.arch,cpu:os.cpus()[0]?.model,logical_cpus:os.cpus().length},counts,profiles,scenes:scenes.map(i=>SCENES[i]),trials,ticks,dt:1/60,thresholds:THRESHOLDS,
  methodology:{timed:'One real WASM f64 world step, including scheduled projectile insertion/lifecycle. Excludes all observations, independent geometric QA, hashing, rendering and JS loop.',startup_ticks:60,sleeping:'Disabled on capacity fixtures; enabled ONLY in sleeping-control.',contact_geometry:'Stacks: four levels; shallow contact: one level; growing rectangular touching footprint; 36-unit crates, mass2, friction1, zero restitution, one fixed floor. Not the complete Pages room/player fixture.',impacts:'Three wave types at ticks60/120/180; one shot per four front-face columns. Real CCD, no bounce/impact cap or hidden retirement beyond declared impact-retire policy.',quality:'Every tick independent full OBB SAT for box pairs; floor .5 inherited, pair penetration/drift 1.8 are explicitly new screening thresholds (5% of box width). Failures retained, never capacity denominators.',capacity:'Worst individual trial p95; requires repeated trajectories and every quality gate, contiguous passing grid. Does not promise an entire rendered frame fits.'},records,capacity:capacity(records,counts,profiles)};
writeFileSync(output,JSON.stringify(report,null,2)+'\n');
if(records.some(r=>trials>=2&&!r.repeatable))throw Error('Nondeterministic repeated replay; see retained output');
