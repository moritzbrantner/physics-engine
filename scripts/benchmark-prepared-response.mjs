// Compare the same fixed-step solver before/after response preparation, not different solvers.
import assert from 'node:assert/strict';
import {createHash} from 'node:crypto';
import {readFileSync, writeFileSync} from 'node:fs';
import os from 'node:os';

const [basePath, candidatePath, output, option] = process.argv.slice(2);
const bookkeepingMode = option === '--bookkeeping';
const inlineGeometryMode = option === '--geometry-inline';
const geometryMode = option === '--geometry' || inlineGeometryMode;
if (option && !bookkeepingMode && !geometryMode) throw new Error('Unknown option; expected --bookkeeping, --geometry or --geometry-inline');
if (!basePath || !candidatePath || !output) {
  throw new Error('usage: node scripts/benchmark-prepared-response.mjs base.wasm candidate.wasm result.json');
}
const trials = Number(process.env.TRIALS ?? 2);
const ticks = Number(process.env.TICKS ?? 240);
if (!Number.isInteger(trials) || trials < 2 || trials > 9 ||
    !Number.isInteger(ticks) || ticks < 120 || ticks > 1200) {
  throw new Error('TRIALS must be 2..9; TICKS must be 120..1200');
}
const hash = bytes => createHash('sha256').update(bytes).digest('hex');
const bytes = {base: readFileSync(basePath), candidate: readFileSync(candidatePath)};
const modules = Object.fromEntries(await Promise.all(Object.entries(bytes).map(async ([k,v])=>[k,await WebAssembly.compile(v)])));
const summarize = values => {
  const sorted = [...values].sort((a,b)=>a-b);
  return {count: values.length, mean_ms: values.length ? values.reduce((a,b)=>a+b,0)/values.length : null,
    p95_ms: sorted.length ? sorted[Math.min(sorted.length-1,Math.floor(sorted.length*.95))] : null,
    max_ms: sorted.at(-1) ?? null};
};
function snapshot(e) {
  const ptr = e.approximate_refresh_snapshot();
  return new Float64Array(e.memory.buffer,ptr,e.approximate_snapshot_len()).slice();
}
function floorDepth(r) {
  const [x,y,z,w] = r.slice(7);
  return Math.max(0, -(r[2] - Math.abs(2*(x*y+w*z))*r[4] - Math.abs(1-2*(x*x+z*z))*r[5] - Math.abs(2*(y*z-w*x))*r[6]));
}
async function replay(build, upright, type, shot, measured=true) {
  const e = (await WebAssembly.instantiate(modules[build],{})).exports;
  const rules = (1<<29)|((1<<11)-2)|(1<<14)|(2<<12)|(upright?1<<11:0);
  assert.equal(e.sandbox_reset_tower_with_baking_options(rules,0,1),0);
  assert.equal(e.approximate_reset_from_sandbox(4,8),0);
  const raw = {settling:[],active:[],sleeping:[]};
  const scratchPeakBytes = {settling:0,active:0,sleeping:0};
  const trace = createHash('sha256');
  const work = Array(11).fill(0);
  const preparation = {bodies:0,inertias:0,inertia_applications:0};
  const bookkeepingNames = ['active_view_rebuilds','active_body_scans','adjacency_rebuilds',
    'adjacency_edges_indexed','island_body_visits','island_edge_visits','bounds_updates',
    'bound_rows_sorted','scratch_growths','scratch_retained_bytes','bounds_order_checks'];
  const bookkeeping = Object.fromEntries(['settling','active','sleeping'].map(p=>[p,Array(11).fill(0)]));
  const geometryNames = ['current_queries','manifold_hits','negative_hits','manifold_refreshes',
    'frame_preparations','frame_reuses','projection_preparations','projection_reuses','sat_queries',
    'sat_axes_tested','clip_passes','sweep_queries','pair_invalidations','cached_pairs_peak','retained_bytes'];
  const geometry = Object.fromEntries(['settling','active','sleeping'].map(p=>[p,Array(15).fill(0)]));
  let peakFloor=0, changed=false, rotated=false, peakAwake=0;
  function step(phase) {
    const quiet = e.approximate_is_quiescent()===1;
    const start=performance.now();
    const error=e.approximate_step_velocity(0,0,0);
    const duration=performance.now()-start;
    assert.equal(error,0,`${build}: ${shot} tick failed`);
    if (measured) raw[phase==='settling'?'settling':quiet?'sleeping':'active'].push(duration);
    const state=snapshot(e);
    if (!measured) return state;
    trace.update(new Uint8Array(state.buffer));
    const observations=[];
    for(let i=0;i<state.length/11;i++) {
      observations.push(e.approximate_body_sleeping(i));
      for(let axis=0;axis<3;axis++) observations.push(e.approximate_body_velocity(i,axis));
    }
    for(let i=0;i<=10;i++) {
      const value=e.approximate_stat(i);
      assert.ok(Number.isFinite(value));
      observations.push(value);
      work[i]+=value;
    }
    if (bookkeepingMode || geometryMode) {
      // Compare all counters introduced by the preceding preparation change too.
      for (const i of [11,12,13]) {
        const v=e.approximate_stat(i);assert.ok(Number.isFinite(v));observations.push(v);
      }
      if (bookkeepingMode && build==='candidate') {
        const row=bookkeeping[phase==='settling'?'settling':quiet?'sleeping':'active'];
        for(let k=0;k<11;k++) {
          const value=e.approximate_stat(14+k);assert.ok(Number.isFinite(value));
          row[k]=k===9?Math.max(row[k],value):row[k]+value;
        }
      }
    }
    if (geometryMode) {
      // Inline manifolds deliberately change retained storage size, not physical work.
      // Only the explicit --geometry-inline experiment separates counter 23 from the equality
      // hash; record its peak for BOTH builds. Every other old counter must still match.
      for (let i=14;i<=24;i++) {
        const v=e.approximate_stat(i); assert.ok(Number.isFinite(v));
        if (i===23) {
          const key=phase==='settling'?'settling':quiet?'sleeping':'active';
          scratchPeakBytes[key]=Math.max(scratchPeakBytes[key],v);
        }
        if (!inlineGeometryMode || i!==23) observations.push(v);
      }
      if (build==='candidate') {
        const row=geometry[phase==='settling'?'settling':quiet?'sleeping':'active'];
        for(let k=0;k<15;k++) {
          const value=e.approximate_stat(25+k);assert.ok(Number.isFinite(value));
          row[k]=k>=13?Math.max(row[k],value):row[k]+value;
        }
      }
    }
    trace.update(JSON.stringify(observations));
    if(build==='candidate') {
      const stats=[11,12,13].map(i=>e.approximate_stat(i));
      assert.ok(stats.every(Number.isFinite), 'candidate must export prepared-response counters');
      preparation.bodies+=stats[0];preparation.inertias+=stats[1];preparation.inertia_applications+=stats[2];
    }
    return state;
  }
  for(let i=0;i<240;i++)step('settling');
  assert.equal(e.approximate_is_quiescent(),1,'tower must settle');
  const initial=snapshot(e);
  const indices=Array.from({length:initial.length/11},(_,i)=>i).filter(i=>initial[i*11]===2);
  assert.equal(indices.length,32);
  assert.equal(e.approximate_set_projectile_type(type),0);
  const direction=shot==='hit'?[0,0,-96]:[40,0,-87];
  assert.ok(e.approximate_shoot(...direction)>=0);
  for(let tick=0;tick<(measured?ticks:60);tick++) {
    const state=step(shot);
    let awake=0;
    for(const i of indices) {
      const row=state.slice(i*11,i*11+11), old=initial.slice(i*11,i*11+11);
      assert.equal(row[0],2);
      assert.ok(row.every(Number.isFinite));
      peakFloor=Math.max(peakFloor,floorDepth(row));
      changed ||= row.slice(1).some((v,k)=>Math.abs(v-old[k+1])>1e-6);
      rotated ||= row.slice(7).some((v,k)=>Math.abs(v-old[k+7])>1e-5);
      awake+=e.approximate_body_sleeping(i)===0?1:0;
    }
    peakAwake=Math.max(peakAwake,awake);
    assert.ok(e.approximate_stat(1)<=4 && e.approximate_stat(5)<=32);
  }
  assert.ok(peakFloor<=0.5,`${build}: floor penetration ${peakFloor}`);
  assert.ok(!upright||!rotated);
  assert.ok(shot==='hit'?changed:!changed);
  if(shot==='miss')assert.equal(peakAwake,0);
  return {scratch_peak_bytes: geometryMode?scratchPeakBytes:null, geometry:geometryMode&&build==='candidate'?Object.fromEntries(Object.entries(geometry).map(([phase,row])=>[phase,Object.fromEntries(geometryNames.map((n,i)=>[n,row[i]]))])):null,
    bookkeeping:bookkeepingMode&&build==='candidate'?Object.fromEntries(Object.entries(bookkeeping).map(([phase,row])=>[phase,Object.fromEntries(bookkeepingNames.map((n,i)=>[n,row[i]]))])):null,
    trace_sha256:trace.digest('hex'),work,preparation:build==='candidate'?preparation:null,
    max_floor_penetration:peakFloor,crates_changed:changed,crates_rotated:rotated,max_awake_crates:peakAwake,
    quiescent_after:e.approximate_is_quiescent()===1,
    phases:Object.fromEntries(Object.entries(raw).map(([k,v])=>[k,{...summarize(v),raw_ms:v}]))};
}
// Warm each actual module before timed trials. Setup/observation work never enters physics timings.
for(const build of ['base','candidate'])await replay(build,false,0,'hit',false);
const records=[];
const crateMode=process.env.CRATE_MODE ?? 'both';
if(!['both','free','upright'].includes(crateMode))throw new Error('CRATE_MODE must be both, free or upright');
const rotations=crateMode==='both'?[false,true]:[crateMode==='upright'];
for(const upright of rotations)for(const [projectile,type] of [['sphere',0],['arrow',1],['rigid',2]])for(const shot of ['hit','miss']) {
  const runs=[];
  for(let trial=0;trial<trials;trial++) {
    const pair={trial};
    for(const build of trial%2?['candidate','base']:['base','candidate'])pair[build]=await replay(build,upright,type,shot);
    assert.equal(pair.base.trace_sha256,pair.candidate.trace_sha256,'base/candidate physical state or compared work counters diverged');
    if(trial)assert.equal(pair.base.trace_sha256,runs[0].base.trace_sha256,'same-build replay diverged');
    runs.push(pair);
  }
  const phases={};
  for(const phase of ['settling','active','sleeping']) {
    phases[phase]={};
    for(const build of ['base','candidate'])phases[phase][build]=summarize(runs.flatMap(r=>r[build].phases[phase].raw_ms));
  }
  const record={crates:upright?'upright':'free',projectile,shot,bit_identical:true,phases,runs};
  records.push(record);
  writeFileSync(output+'.partial',JSON.stringify({complete:false,records},null,2)+'\n');
  console.log(JSON.stringify({crates:record.crates,projectile,shot,bit_identical:true,phases}));
}
writeFileSync(output,JSON.stringify({kind:inlineGeometryMode?'contact-geometry-inline-v2':geometryMode?'contact-geometry-reuse-v1':bookkeepingMode?'fixed-step-bookkeeping-v1':'prepared-response-v1',cpu:os.cpus()[0]?.model,node:process.version,v8:process.versions.v8,
  base_sha256:hash(bytes.base),candidate_sha256:hash(bytes.candidate),trials,post_shot_ticks:ticks,settle_ticks:240,substeps:4,iterations:8,
  note:'Alternating build order after module warmup. Timings include only calls to approximate_step_velocity, not setup or observations. Settling, non-quiescent post-shot ticks, and quiescent ticks are separate. Raw repetitions retained; no wall-clock gate. Hash includes all visible poses, linear velocities, sleep flags and the pre-existing report counters.',
  excluded_memory_stat_indices:inlineGeometryMode?[23]:[],
  memory_note:inlineGeometryMode?'Counter 23 measures retained scratch capacity. Inline manifold storage changes its size; record both builds, do not classify memory layout as physical state. All other old counters 0..24 remain equal.':null,
  crate_mode:crateMode,passed:true,records},null,2)+'\n');
