// Controlled stability experiment. Timing, quality, and same-build determinism are separate.
import assert from 'node:assert/strict';
import {createHash} from 'node:crypto';
import {readFileSync, writeFileSync} from 'node:fs';
import os from 'node:os';
const [candidatePath, output, baselinePath] = process.argv.slice(2);
if (!candidatePath || !output) throw new Error('usage: node scripts/benchmark-correction.mjs candidate.wasm result.json [merged.wasm]');
const trials=Number(process.env.TRIALS??2), ticks=Number(process.env.TICKS??1200);
if(!Number.isInteger(trials)||trials<2||trials>8||!Number.isInteger(ticks)||ticks<240||ticks>3600)throw new Error('TRIALS=2..8 and TICKS=240..3600 required');
const hash=x=>createHash('sha256').update(x).digest('hex');
const bytes=readFileSync(candidatePath),baseBytes=baselinePath?readFileSync(baselinePath):null;
const candidate=await WebAssembly.compile(bytes),baseline=baseBytes?await WebAssembly.compile(baseBytes):null;
const modes=baseline?['baseline','baumgarte','soft','relaxed']:['baumgarte','soft','relaxed'];
const phaseNames=['settling','active','sleeping'];
const summary=v=>{const s=[...v].sort((a,b)=>a-b);return {count:v.length,mean_ms:v.length?v.reduce((a,b)=>a+b,0)/v.length:null,total_ms:v.reduce((a,b)=>a+b,0),p95_ms:s[Math.floor(s.length*.95)]??null,max_ms:s.at(-1)??null};};
function snapshot(e){const p=e.approximate_refresh_snapshot();return new Float64Array(e.memory.buffer,p,e.approximate_snapshot_len()).slice();}
function floor(r){const [x,y,z,w]=r.slice(7);return Math.max(0,-(r[2]-Math.abs(2*(x*y+w*z))*r[4]-Math.abs(1-2*(x*x+z*z))*r[5]-Math.abs(2*(y*z-w*x))*r[6]));}
function extras(e,s){return Array.from({length:s.length/11},(_,i)=>[e.approximate_body_sleeping(i),...Array.from({length:3},(_,a)=>e.approximate_body_velocity(i,a))]).flat();}
function diagnostics(e,s){let energy=typeof e.approximate_body_kinetic_energy==='function'?0:null,speed2=0,angular2=typeof e.approximate_body_angular_velocity==='function'?0:null,penetration=0,awake=0;
 for(let i=0;i<s.length/11;i++)if(s[i*11]===2){penetration=Math.max(penetration,floor(s.slice(i*11,i*11+11)));awake+=e.approximate_body_sleeping(i)===0?1:0;
  if(energy!==null)energy+=e.approximate_body_kinetic_energy(i);
  for(let a=0;a<3;a++){speed2+=e.approximate_body_velocity(i,a)**2;if(angular2!==null)angular2+=e.approximate_body_angular_velocity(i,a)**2;}
 }
 assert.ok([speed2,penetration,awake,...(energy===null?[]:[energy]),...(angular2===null?[]:[angular2])].every(Number.isFinite));
 return {kinetic:energy,linear_rms:Math.sqrt(speed2/32),angular_rms:angular2===null?null:Math.sqrt(angular2/32),floor_penetration:penetration,awake};
}
async function replay(mode,upright,type,shot,measure=true){
 const e=(await WebAssembly.instantiate(mode==='baseline'?baseline:candidate,{})).exports;
 const rules=(1<<29)|((1<<11)-2)|(1<<14)|(2<<12)|(upright?1<<11:0);
 assert.equal(e.sandbox_reset_tower_with_baking_options(rules,0,1),0);
 assert.equal(mode==='soft'||mode==='relaxed'?e.approximate_reset_soft_from_sandbox(4,8,60,1,mode==='relaxed'?2:0):e.approximate_reset_from_sandbox(4,8),0);
 const physics=createHash('sha256'), workHash=createHash('sha256'),decisions=createHash('sha256');
 const timing=Object.fromEntries(phaseNames.map(p=>[p,[]])),work=Object.fromEntries(phaseNames.map(p=>[p,{primary_iterations:0,relaxation_iterations:0,narrow_tests:0,constraint_visits:0,relaxation_constraint_visits:0,woken:0}]));
 let maxFloor=0,maxSettleFloor=0,maxAwake=0,energyPeak=null,energyIntegral=null,final=null,changed=false,rotated=false,wakes=0,sleeps=0,lastAwake=-1,scratchPeak=0;
 const series=[],late=[];let initial,ids,prev=[];
 function step(phase,t){
  const start=performance.now(),error=e.approximate_step_velocity(0,0,0),ms=performance.now()-start;
  assert.equal(error,0,`${mode}/${upright}/${type}/${shot}/${phase}/${t}: step error`);
  const s=snapshot(e);assert.ok(s.every(Number.isFinite));
  if(!measure)return s;
  timing[phase].push(ms);physics.update(new Uint8Array(s.buffer));const x=extras(e,s);assert.ok(x.every(Number.isFinite));physics.update(JSON.stringify(x));
  const stats=Array.from({length:52},(_,i)=>e.approximate_stat(i));assert.ok(stats.every(Number.isFinite));
  // The existing default path's entire work history remains the reference, not softened poses.
  workHash.update(JSON.stringify(stats));
  const relax=mode==='baseline'?0:e.approximate_stat(53),visits=mode==='baseline'?0:e.approximate_stat(54);
  assert.ok(stats[1]<=4&&stats[5]<=40);
  assert.equal(stats[5]-relax+stats[46],stats[1]*8,'primary iteration accounting');
  if(mode!=='relaxed')assert.equal(relax,0);
  assert.ok(relax<=stats[1]*2);
  const w=work[phase];w.primary_iterations+=stats[5]-relax;w.relaxation_iterations+=relax;w.narrow_tests+=stats[3];w.constraint_visits+=stats[40];w.relaxation_constraint_visits+=visits;w.woken+=stats[7];
  const correction=mode==='baseline'?[]:Array.from({length:7},(_,i)=>e.approximate_stat(52+i));assert.ok(correction.every(Number.isFinite));decisions.update(JSON.stringify([...stats.slice(40),...correction]));
  scratchPeak=Math.max(scratchPeak,stats[23]+(correction[5]??0));
  const d=diagnostics(e,s);
  if(phase==='settling')maxSettleFloor=Math.max(maxSettleFloor,d.floor_penetration);
  else {
   final=d;maxFloor=Math.max(maxFloor,d.floor_penetration);maxAwake=Math.max(maxAwake,d.awake);if(e.approximate_is_quiescent()!==1)lastAwake=t;
   if(d.kinetic!==null){energyPeak=Math.max(energyPeak??0,d.kinetic);energyIntegral=(energyIntegral??0)+d.kinetic/60;}
   for(const i of ids){const row=s.slice(i*11,i*11+11),old=initial.slice(i*11,i*11+11);assert.equal(row[0],2);
    changed ||= row.some((v,k)=>k>0&&Math.abs(v-old[k])>1e-6);rotated ||= row.slice(7).some((v,k)=>Math.abs(v-old[k+7])>1e-5);
    if(shot==='miss')assert.ok(row.every((v,k)=>Object.is(v,old[k])),'near miss changed sleeping pose');
    const sleep=x[i*4]===1;if(prev[i]!==undefined&&prev[i]!==sleep){if(sleep)sleeps++;else wakes++;}prev[i]=sleep;
   }
   if(t%60===0||t===ticks-1)series.push({tick:t+1,...d});if(t>=ticks-120)late.push(d);
  }
  return s;
 }
 for(let t=0;t<240;t++)step('settling',t);
 assert.equal(e.approximate_is_quiescent(),1,`${mode}: tower not settled before shot`);
 initial=snapshot(e);ids=Array.from({length:initial.length/11},(_,i)=>i).filter(i=>initial[i*11]===2);assert.equal(ids.length,32);prev=Array.from({length:initial.length/11},(_,i)=>e.approximate_body_sleeping(i)===1);
 assert.equal(e.approximate_set_projectile_type(type),0);assert.ok(e.approximate_shoot(...(shot==='hit'?[0,0,-96]:[40,0,-87]))>=0);
 for(let t=0;t<(measure?ticks:60);t++)step(e.approximate_is_quiescent()===1?'sleeping':'active',t);
 if(!measure)return;
 assert.ok(maxFloor<=0.5&&maxSettleFloor<=0.5,`${mode}: floor penetration ${maxFloor}/${maxSettleFloor}`);
 assert.ok(shot==='hit'?changed:!changed,'hit/miss response');assert.ok(!upright||!rotated,'rotation lock');if(shot==='miss')assert.equal(maxAwake,0);
 assert.ok(Math.abs(e.approximate_stat(0)-(240+ticks)/60)<1e-9,'dropped requested time');
 return {physics_hash:physics.digest('hex'),work_hash:workHash.digest('hex'),decisions_hash:decisions.digest('hex'),
  changed,rotated,max_floor_penetration:maxFloor,settle_max_floor_penetration:maxSettleFloor,max_awake_crates:maxAwake,quiescent_after:e.approximate_is_quiescent()===1,
  settled_seconds:e.approximate_is_quiescent()===1?(lastAwake+2)/60:null,energy_peak:energyPeak,kinetic_time_integral:energyIntegral,
  late_linear_rms:Math.sqrt(late.reduce((a,d)=>a+d.linear_rms**2,0)/late.length),late_angular_rms:late[0]?.angular_rms===null?null:Math.sqrt(late.reduce((a,d)=>a+d.angular_rms**2,0)/late.length),final,wakes,sleeps,scratch_peak_bytes:scratchPeak,series,
  phases:Object.fromEntries(phaseNames.map(p=>[p,{...summary(timing[p]),work:work[p],raw_ms:timing[p]}]))};
}
for(const mode of modes)await replay(mode,false,0,'hit',false);
const records=[];
for(const upright of [false,true])for(const [projectile,type] of [['sphere',0],['arrow',1],['rigid',2]])for(const shot of ['hit','miss']){
 const runs=[];
 for(let trial=0;trial<trials;trial++){
  const order=[...modes.slice(trial%modes.length),...modes.slice(0,trial%modes.length)];if(Math.floor(trial/modes.length)%2)order.reverse();
  const pair={trial,order};for(const mode of order)pair[mode]=await replay(mode,upright,type,shot);
  if(baseline){assert.equal(pair.baumgarte.physics_hash,pair.baseline.physics_hash,'default physics changed');assert.equal(pair.baumgarte.work_hash,pair.baseline.work_hash,'default work changed');}
  if(trial)for(const mode of modes){assert.equal(pair[mode].physics_hash,runs[0][mode].physics_hash,`${mode}: nondeterministic state`);assert.equal(pair[mode].decisions_hash,runs[0][mode].decisions_hash,`${mode}: nondeterministic stopping`);}
  runs.push(pair);
 }
 const row={crates:upright?'upright':'free',projectile,shot,passed:true,repeatable:true,baseline_parity:baseline?true:null,runs,
  phases:Object.fromEntries(phaseNames.map(p=>[p,Object.fromEntries(modes.map(m=>[m,summary(runs.flatMap(r=>r[m].phases[p].raw_ms))]))]))};
 records.push(row);console.log(JSON.stringify({crates:row.crates,projectile,shot,active:row.phases.active,quality:Object.fromEntries(modes.map(m=>[m,{settled_seconds:runs[0][m].settled_seconds,floor:runs[0][m].max_floor_penetration,energy:runs[0][m].final.kinetic}]))}));
 writeFileSync(output+'.partial',JSON.stringify({complete:false,records},null,2)+'\n');
}
writeFileSync(output,JSON.stringify({kind:'soft-contact-correction-v1',complete:true,cpu:os.cpus()[0]?.model,node:process.version,v8:process.versions.v8,
 candidate_sha256:hash(bytes),baseline_sha256:baseBytes?hash(baseBytes):null,trials,post_shot_ticks:ticks,settle_ticks:240,
 policies:{baumgarte:'merged default with validated convergence',soft:'60 Hz dynamic-pair compliance, hard fixed supports, damping ratio 1, no relaxation',relaxed:'same contact rule + at most two relaxation passes per substep'},
 note:'All requested time is simulated. Timing excludes setup/observations and separates settling, non-quiescent post-shot and quiescent calls. Different correction policies may change contacts, trajectories, sleep timing and active workloads. A lower active mean is not an equal-work throughput claim. Kinetic energy uses engine mass/scene units, not SI joules. Default parity uses the separately built merged module; energy is unavailable for that old module, not fabricated. No end-of-window sleep assertion or forced sleep.',records},null,2)+'\n');
