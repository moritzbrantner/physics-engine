// Paired actual-WASM acceptance. Older references explicitly report unavailable sleep telemetry as null. A failed event run is reported, never a speedup denominator.
import { createHash } from 'node:crypto';
import { readFileSync, writeFileSync } from 'node:fs';
import os from 'node:os';
const [wasmPath, outputPath, referencePath] = process.argv.slice(2);
if (!wasmPath || !outputPath) throw new Error('usage: node scripts/benchmark-fixed-step.mjs candidate.wasm result.json [event-reference.wasm]');
const count = Number(process.env.TRIALS ?? 2);
if (!Number.isInteger(count) || count < 2 || count > 5) throw new Error('TRIALS must be 2..5 for replay evidence');
const bytes = readFileSync(wasmPath), refBytes = readFileSync(referencePath ?? wasmPath);
const modules = { approximate: await WebAssembly.compile(bytes), event: await WebAssembly.compile(refBytes) };
const sha = b => createHash('sha256').update(b).digest('hex');
const ticks = Number(process.env.TICKS ?? 240);
if (!Number.isInteger(ticks) || ticks < 120 || ticks > 1200) throw new Error('TICKS must be 120..1200');
const records = [];
const quantile = (xs, p) => xs.slice().sort((a,b)=>a-b)[Math.min(xs.length-1, Math.floor(xs.length*p))] ?? null;
const rows = data => Array.from({length:data.length/11}, (_,i)=>Array.from(data.slice(i*11,i*11+11)));
function bottom(r) {
  const [x,y,z,w] = r.slice(7);
  return r[2] - Math.abs(2*(x*y+w*z))*r[4] - Math.abs(1-2*(x*x+z*z))*r[5] - Math.abs(2*(y*z-w*x))*r[6];
}
for (const mode of ['event', 'approximate']) {
  for (const upright of [false, true]) for (const [kind, type] of [['sphere',0],['arrow',1],['rigid',2]]) for (const shot of ['hit','miss']) {
    const runs=[];
    for (let trial=0; trial<count; trial++) {
      const e = (await WebAssembly.instantiate(modules[mode],{})).exports;
      const rules=(1<<29)|((1<<11)-2)|(1<<14)|(2<<12)|(upright ? 1<<11 : 0);
      if(e.sandbox_reset_tower_with_baking_options(rules,0,1)) throw new Error('fixture reset');
      const approx=mode==='approximate';
      if(approx && e.approximate_reset_from_sandbox(4,8)) throw new Error('approximation import');
      const step=approx ? ()=>e.approximate_step_velocity(0,0,0) : ()=>e.sandbox_step_velocity(0,0,0);
      const snapshot=()=>{
        if(approx){ const ptr=e.approximate_refresh_snapshot(); return rows(new Float64Array(e.memory.buffer,ptr,e.approximate_snapshot_len()).slice()); }
        const ptr=e.sandbox_refresh_render_snapshot();
        return rows(new Int32Array(e.memory.buffer,ptr,e.sandbox_render_snapshot_len()).slice()).map(r=>r.map((v,i)=>i>=7?v/(2**30):v));
      };
      const sleepStatusAvailable=approx || typeof e.sandbox_body_sleeping==='function';
      const asleep=i=>approx ? e.approximate_body_sleeping(i)===1 : sleepStatusAvailable ? e.sandbox_body_sleeping(i)===1 : null;
      const quiet=()=>approx ? e.approximate_is_quiescent()===1 : e.sandbox_is_quiescent()===1;
      const trace=createHash('sha256');
      let failure=null, complete=0, peakFloor=0, peakAwake=sleepStatusAvailable?0:null, changed=false, rotated=false, peakSubsteps=0, peakIterations=0, peakEvents=0, peakSwept=0;
      for(let t=0;t<240;t++){const error=step(); if(error){failure={phase:'settle',tick:t,error,detail:approx?null:e.sandbox_error_detail()};break;}}
      const initial=snapshot(), crateIndices=initial.map((r,i)=>r[0]===2?i:-1).filter(i=>i>=0);
      if(crateIndices.length!==32) throw new Error('must import all 32 crates');
      const settled=quiet();
      if(!settled&&!failure)failure={phase:'settle',reason:'not quiescent after 240 ticks'};
      const times=[];
      if(!failure){
        if((approx?e.approximate_set_projectile_type(type):e.sandbox_set_projectile_type(type))!==0)throw new Error('projectile type');
        const v=shot==='hit'?[0,0,-96]:[40,0,-87];
        if((approx?e.approximate_shoot(...v):e.sandbox_shoot(...v))<0)throw new Error('projectile spawn');
        for(let t=0;t<ticks;t++){
          const start=performance.now(), error=step(); times.push(performance.now()-start);
          if(error){ failure={phase:'impact',tick:t,error,detail:approx?null:e.sandbox_error_detail()};break; }
          complete++;
          const state=snapshot();
          trace.update(JSON.stringify(state));
          let awake=0;
          for(const i of crateIndices){
            const r=state[i], before=initial[i];
            if(!r||r[0]!==2||!r.every(Number.isFinite))throw new Error('nonfinite or missing crate');
            peakFloor=Math.max(peakFloor,-bottom(r));
            changed ||= r.slice(1).some((v,k)=>Math.abs(v-before[k+1])>1e-6);
            rotated ||= r.slice(7).some((v,k)=>Math.abs(v-before[k+7])>1e-5);
            if(asleep(i)===false)awake++;
          }
          if(sleepStatusAvailable)peakAwake=Math.max(peakAwake,awake);
          if(approx){
            peakSubsteps=Math.max(peakSubsteps,e.approximate_stat(1));
            peakIterations=Math.max(peakIterations,e.approximate_stat(5));
            peakSwept=Math.max(peakSwept,e.approximate_stat(8));
          }else peakEvents=Math.max(peakEvents,e.sandbox_last_sampled_events?.() ?? 0);
        }
      }
      const checks={ completed:complete===ticks, changed_on_hit:shot!=='hit'||changed, unchanged_on_miss:shot!=='miss'||!changed, sleeping_on_miss:shot!=='miss'||(sleepStatusAvailable ? peakAwake===0 : null), upright_preserved:!upright||!rotated, floor_penetration_bounded:peakFloor<=0.5, bounded_work:!approx||(peakSubsteps<=4&&peakIterations<=32)};
      runs.push({trial,passed:!failure&&Object.values(checks).every(value=>value===true||value===null),failure,checks,unavailable_checks:Object.keys(checks).filter(key=>checks[key]===null),sleep_status_available:sleepStatusAvailable,completed_ticks:complete,settled_before:settled,quiescent_after:quiet(),crates_changed:changed,crates_rotated:rotated,max_floor_penetration:peakFloor,max_awake_crates:peakAwake,max_substeps:peakSubsteps,max_impulse_iterations:peakIterations,max_sampled_events:peakEvents,max_swept_contacts:peakSwept,simulated_seconds:approx?e.approximate_stat(0):(240+complete)/60,steps_ms:{mean:times.length?times.reduce((a,b)=>a+b,0)/times.length:null,p95:quantile(times,.95),max:times.length?Math.max(...times):null},replay_sha256:trace.digest('hex'),raw_ms:times});
    }
    const record={mode,crates:upright?'upright':'free',projectile:kind,shot,runs,repeatable:runs.every(r=>r.replay_sha256===runs[0].replay_sha256),passed:runs.every(r=>r.passed)};
    records.push(record); console.log(JSON.stringify({...record,runs:runs.map(({raw_ms,...r})=>r)}));
  }
}
const report={kind:'fixed-step-vs-event-v1',node:process.version,v8:process.versions.v8,cpu:os.cpus()[0]?.model,candidate_sha256:sha(bytes),reference_sha256:sha(refBytes),requested_ticks:ticks,warmup_ticks:240,substeps:4,iterations:8,trials:count,note:'Sequential host Node/V8 physics-only times exclude snapshot/checks, include post-impact idle ticks. No speedup is meaningful for a reference run that failed. Repeated observable pose hashes are same-mode checks, not cross-mode exactness.',records,approximation_passed:records.filter(r=>r.mode==='approximate').every(r=>r.passed&&r.repeatable)};
writeFileSync(outputPath,JSON.stringify(report,null,2)+'\n');
if(!report.approximation_passed)process.exitCode=1;
