// Compare stopping policy only: identical fixture, CCD, math and iteration ceiling.
import assert from 'node:assert/strict';
import {createHash} from 'node:crypto';
import {readFileSync, writeFileSync} from 'node:fs';
import os from 'node:os';

const [candidatePath, output, baselinePath] = process.argv.slice(2);
if (!candidatePath || !output) throw new Error('usage: node scripts/benchmark-convergence.mjs candidate.wasm result.json [baseline.wasm]');
const trials = Number(process.env.TRIALS ?? 3), ticks = Number(process.env.TICKS ?? 240);
if (!Number.isInteger(trials) || trials < 2 || trials > 9 || !Number.isInteger(ticks) || ticks < 120 || ticks > 1200) throw new Error('TRIALS must be 2..9 and TICKS 120..1200');
const digest = bytes => createHash('sha256').update(bytes).digest('hex');
const candidateBytes=readFileSync(candidatePath), baselineBytes=baselinePath?readFileSync(baselinePath):null;
const candidate=await WebAssembly.compile(candidateBytes), baseline=baselineBytes?await WebAssembly.compile(baselineBytes):null;
const summary = values => {
  const v=[...values].sort((a,b)=>a-b);
  return {count:v.length,mean_ms:v.length?values.reduce((a,b)=>a+b,0)/v.length:null,p95_ms:v[Math.min(v.length-1,Math.floor(v.length*.95))]??null,max_ms:v.at(-1)??null};
};
const names=['constraint_visits','residual_checks','residual_constraint_visits','converged_substeps','capped_substeps','empty_substeps','skipped_iterations','max_exit_impulse_delta','max_exit_velocity_residual','fixed_substeps','probe_passes','delta_constraint_checks'];
const phases=['settling','active','sleeping'];
function snapshot(e) {
  const ptr=e.approximate_refresh_snapshot();
  return new Float64Array(e.memory.buffer,ptr,e.approximate_snapshot_len()).slice();
}
function floorDepth(r) {
  const [x,y,z,w]=r.slice(7);
  return Math.max(0,-(r[2]-Math.abs(2*(x*y+w*z))*r[4]-Math.abs(1-2*(x*x+z*z))*r[5]-Math.abs(2*(y*z-w*x))*r[6]));
}
async function replay(mode,upright,type,shot,measured=true) {
  const e=(await WebAssembly.instantiate(mode==='baseline'?baseline:candidate,{})).exports;
  const rules=(1<<29)|((1<<11)-2)|(1<<14)|(2<<12)|(upright?1<<11:0);
  assert.equal(e.sandbox_reset_tower_with_baking_options(rules,0,1),0);
  const reset=mode==='adaptive'?e.approximate_reset_from_sandbox:(e.approximate_reset_fixed_iterations_from_sandbox??e.approximate_reset_from_sandbox);
  assert.equal(reset(4,8),0);
  const history=[], physics=createHash('sha256'), legacyWork=createHash('sha256'), decisions=createHash('sha256');
  const raw=Object.fromEntries(phases.map(p=>[p,[]]));
  const work=Object.fromEntries(phases.map(p=>[p,{passes:0,...Object.fromEntries(names.map(n=>[n,0]))}]));
  let peakFloor=0,peakAwake=0,changed=false,rotated=false,missPreserved=true;
  function step(settling) {
    const phase=settling?'settling':e.approximate_is_quiescent()===1?'sleeping':'active';
    const start=performance.now(), error=e.approximate_step_velocity(0,0,0), duration=performance.now()-start;
    assert.equal(error,0,`${mode}/${upright}/${type}/${shot}: step failed`);
    const state=snapshot(e);
    assert.ok(state.every(Number.isFinite));
    if (!measured) return state;
    raw[phase].push(duration);
    const extras=[];
    for(let i=0;i<state.length/11;i++) {
      extras.push(e.approximate_body_sleeping(i));
      for(let j=0;j<3;j++) extras.push(e.approximate_body_velocity(i,j));
    }
    assert.ok(extras.every(Number.isFinite));
    physics.update(new Uint8Array(state.buffer));physics.update(JSON.stringify(extras));
    const legacy=Array.from({length:40},(_,i)=>e.approximate_stat(i));
    assert.ok(legacy.every(Number.isFinite));legacyWork.update(JSON.stringify(legacy));
    assert.ok(legacy[1]<=4&&legacy[5]<=32);
    work[phase].passes+=legacy[5];
    if(mode!=='baseline') {
      const row=names.map((_,i)=>e.approximate_stat(40+i));
      assert.ok(row.every(Number.isFinite));decisions.update(JSON.stringify(row));
      assert.equal(legacy[5]+row[6],legacy[1]*8,'iteration accounting lost work');
      if(mode==='fixed') assert.equal(row[3]+row[4]+row[5]+row[6],0,'fixed reference terminated early');
      for(let i=0;i<row.length;i++)work[phase][names[i]]=names[i].startsWith('max_')?Math.max(work[phase][names[i]],row[i]):work[phase][names[i]]+row[i];
    }
    history.push({state,extras});
    return state;
  }
  for(let i=0;i<240;i++)step(true);
  assert.equal(e.approximate_is_quiescent(),1,`${mode}: tower did not settle`);
  const initial=snapshot(e);
  const indices=Array.from({length:initial.length/11},(_,i)=>i).filter(i=>initial[i*11]===2);
  assert.equal(indices.length,32);
  assert.equal(e.approximate_set_projectile_type(type),0);
  assert.ok(e.approximate_shoot(...(shot==='hit'?[0,0,-96]:[40,0,-87]))>=0);
  for(let tick=0;tick<(measured?ticks:60);tick++) {
    const s=step(false);let awake=0;
    for(const i of indices) {
      const row=s.slice(i*11,i*11+11),old=initial.slice(i*11,i*11+11);
      assert.equal(row[0],2);
      peakFloor=Math.max(peakFloor,floorDepth(row));
      changed ||= row.slice(1).some((v,k)=>Math.abs(v-old[k+1])>1e-6);
      rotated ||= row.slice(7).some((v,k)=>Math.abs(v-old[k+7])>1e-5);
      awake+=e.approximate_body_sleeping(i)===0?1:0;
      if(shot==='miss')missPreserved&&=row.every((v,k)=>Object.is(v,old[k]));
    }
    peakAwake=Math.max(peakAwake,awake);
  }
  assert.ok(peakFloor<=0.5,`${mode}: floor penetration ${peakFloor}`);
  assert.ok(!upright||!rotated,'rotation lock changed');
  assert.ok(shot==='hit'?changed:!changed,'incorrect hit/miss response');
  if(shot==='miss'){assert.equal(peakAwake,0);assert.ok(missPreserved,'near miss changed a sleeping pose');}
  assert.ok(Math.abs(e.approximate_stat(0)-(240+(measured?ticks:60))/60)<1e-9,'simulation dropped time');
  return {physics_hash:physics.digest('hex'),legacy_work_hash:legacyWork.digest('hex'),decisions_hash:decisions.digest('hex'),
    max_floor_penetration:peakFloor,max_awake_crates:peakAwake,changed,rotated,miss_preserved:missPreserved,
    quiescent_after:e.approximate_is_quiescent()===1,
    phases:Object.fromEntries(phases.map(p=>[p,{...summary(raw[p]),work:work[p],raw_ms:raw[p]}])),history};
}
function difference(a,b) {
  assert.equal(a.length,b.length);
  let maxPosition=0,maxVelocity=0,maxQuaternion=0,sleepDifferences=0;
  for(let t=0;t<a.length;t++)for(let i=0;i<a[t].state.length/11;i++) {
    if(a[t].state[i*11]!==2)continue;
    assert.equal(b[t].state[i*11],2);
    for(const j of [1,2,3])maxPosition=Math.max(maxPosition,Math.abs(a[t].state[i*11+j]-b[t].state[i*11+j]));
    for(const j of [7,8,9,10])maxQuaternion=Math.max(maxQuaternion,Math.abs(a[t].state[i*11+j]-b[t].state[i*11+j]));
    for(const j of [1,2,3])maxVelocity=Math.max(maxVelocity,Math.abs(a[t].extras[i*4+j]-b[t].extras[i*4+j]));
    sleepDifferences+=a[t].extras[i*4]!==b[t].extras[i*4]?1:0;
  }
  return {max_crate_position_delta:maxPosition,max_crate_velocity_delta:maxVelocity,max_crate_quaternion_delta:maxQuaternion,crate_sleep_observation_differences:sleepDifferences};
}
assert.ok(WebAssembly.Module.exports(candidate).some(e=>e.name==='approximate_reset_fixed_iterations_from_sandbox'),'fixed-pass reference export missing');
for(const mode of baseline?['baseline','fixed','adaptive']:['fixed','adaptive'])await replay(mode,false,0,'hit',false);
const records=[];
for(const upright of [false,true])for(const [projectile,type] of [['sphere',0],['arrow',1],['rigid',2]])for(const shot of ['hit','miss']) {
  const runs=[];let baselineRecord=null;
  // A baseline replay is parity evidence only, not a one-sample speedup denominator.
  if(baseline){baselineRecord=await replay('baseline',upright,type,shot);delete baselineRecord.history;}
  for(let trial=0;trial<trials;trial++) {
    const pair={trial};
    for(const mode of trial%2?['adaptive','fixed']:['fixed','adaptive'])pair[mode]=await replay(mode,upright,type,shot);
    if(baselineRecord){
      assert.equal(pair.fixed.physics_hash,baselineRecord.physics_hash,'fixed reference changed baseline physics');
      assert.equal(pair.fixed.legacy_work_hash,baselineRecord.legacy_work_hash,'fixed reference changed baseline work');
    }
    for(const mode of ['fixed','adaptive'])if(trial){
      assert.equal(pair[mode].physics_hash,runs[0][mode].physics_hash,`${mode} replay differs`);
      assert.equal(pair[mode].decisions_hash,runs[0][mode].decisions_hash,`${mode} stopping decisions differ`);
    }
    pair.difference=difference(pair.fixed.history,pair.adaptive.history);
    delete pair.fixed.history;delete pair.adaptive.history;
    runs.push(pair);
  }
  const aggregate=Object.fromEntries(phases.map(p=>[p,Object.fromEntries(['fixed','adaptive'].map(m=>[m,summary(runs.flatMap(r=>r[m].phases[p].raw_ms))]))]));
  const record={crates:upright?'upright':'free',projectile,shot,baseline_parity:baselineRecord?true:null,repeatable:true,passed:true,phases:aggregate,runs};
  records.push(record);
  writeFileSync(output+'.partial',JSON.stringify({complete:false,records},null,2)+'\n');
  console.log(JSON.stringify({crates:record.crates,projectile,shot,phases:aggregate,difference:runs[0].difference,
    fixed_work:runs[0].fixed.phases.active.work,adaptive_work:runs[0].adaptive.phases.active.work}));
}
writeFileSync(output,JSON.stringify({kind:'fixed-step-convergence-v1',complete:true,cpu:os.cpus()[0]?.model,node:process.version,v8:process.versions.v8,
  candidate_sha256:digest(candidateBytes),baseline_sha256:baselineBytes?digest(baselineBytes):null,trials,post_shot_ticks:ticks,settle_ticks:240,
  note:'Same-binary fixed-eight-pass vs convergence stopping. CPU-call timing excludes setup, snapshots and checks. Active/settling/quiescent phases are separate. Baseline replay verifies reference parity only. Cross-policy hashes may differ; fixed inputs and stopping decisions must repeat within each policy.',
  records},null,2)+'\n');
