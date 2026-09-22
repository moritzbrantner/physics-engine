// Uses the SAME runtime adapter and Rust entry point as /scenarios/tower/, not the comparison page.
// Behavioral acceptance is deterministic; recorded wall times are diagnostic only, never a CI gate.
import assert from 'node:assert/strict';
import { readFileSync, writeFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import os from 'node:os';
import { createTowerRuntime } from '../site/tower-runtime.mjs';
import { encodeScenarioRules, COLLISION_PAIRS } from '../site/simulation-rules-config.mjs';
const [wasmPath, outputPath] = process.argv.slice(2);
if (!wasmPath) throw new Error('Usage: node scripts/test-tower-runtime.mjs engine.wasm [report.json]');
const bytes = readFileSync(wasmPath), module = await WebAssembly.compile(bytes), records = [];
const ticks = Number(process.env.TOWER_TICKS ?? 1200), trials = Number(process.env.TOWER_TRIALS ?? 2);
assert(Number.isInteger(ticks) && ticks >= 600 && ticks <= 3600);
assert(Number.isInteger(trials) && trials >= 1 && trials <= 6);
function rows(e) {
  const p=e.sandbox_refresh_render_snapshot();
  const a=new Float64Array(e.memory.buffer,p,e.sandbox_render_snapshot_len());
  return Array.from({length:a.length/11},(_,i)=>Array.from(a.slice(i*11,i*11+11)));
}
function bottom(r) {
  const [x,y,z,w]=r.slice(7);
  return r[2]-Math.abs(2*(x*y+w*z))*r[4]-Math.abs(1-2*(x*x+z*z))*r[5]-Math.abs(2*(y*z-w*x))*r[6];
}
for (const characterResponse of ['physical','linear']) for (const crateMotion of ['free','upright']) {
 for (const projectileImpactPolicy of ['impact-retire','physical','inelastic']) {
  for (const scenario of ['repeated-hits','near-misses','immediate-hit','mixed-burst']) {
   const runs=[];
   for(let trial=0;trial<trials;trial++){
    const raw=(await WebAssembly.instantiate(module,{})).exports,e=createTowerRuntime(raw);
    const rules=encodeScenarioRules({characterResponse,crateMotion,projectileImpactPolicy,enabledPairs:new Set(COLLISION_PAIRS.map(([id])=>id))});
    assert.equal(e.sandbox_reset_tower_with_baking_options(rules),0);
    assert.equal(e.sandbox_body_count(),44);
    if(scenario!=='immediate-hit')for(let i=0;i<240;i++)assert.equal(e.sandbox_step_velocity(0,0,0),0);
    const initial=rows(e).filter(r=>r[0]===2),before=e.elapsed(),hash=createHash('sha256');
    assert.equal(initial.length,32);
    let peakFloor=0,peakAwake=0,peakLive=0,maxIterations=0,shots=0,changed=false,rotated=false;
    const times=[];
    for(let t=0;t<ticks;t++){
      const fire=scenario==='immediate-hit'? t===0 : scenario==='mixed-burst'?t<12:t<480&&t%24===0;
      if(fire){
        const burst=1;
        for(let j=0;j<burst;j++){
          assert.equal(e.sandbox_set_projectile_type(scenario==='immediate-hit'?1:shots%3),0,'selection must preserve live projectiles');
          const v=scenario==='near-misses'?[40,0,-87]:scenario==='mixed-burst'?[((shots%3)-1)*3,0,-96]:[0,0,-96];
          assert(e.sandbox_shoot(...v)>0,'accepted shot');shots++;
        }
      }
      const start=performance.now();assert.equal(e.sandbox_step_velocity(0,0,0),0,`${scenario}/${characterResponse}/${crateMotion}/${projectileImpactPolicy} tick ${t}`);times.push(performance.now()-start);
      const all=rows(e),crates=all.filter(r=>r[0]===2);assert.equal(crates.length,32);
      assert(all.flat().every(Number.isFinite),'finite snapshot');
      assert.equal(e.error(),0);
      for(const [i,r] of crates.entries()){
        const penetration=-bottom(r);peakFloor=Math.max(peakFloor,penetration);
        changed ||=r.slice(1).some((v,k)=>Math.abs(v-initial[i][k+1])>1e-6);
        rotated ||=r.slice(7).some((v,k)=>Math.abs(v-initial[i][k+7])>1e-5);
        assert(Math.abs(Math.hypot(...r.slice(7))-1)<1e-8,'normalized orientation');
      }
      let awake=0;all.forEach((r,i)=>{if(r[0]===2&&e.bodySleeping(i)!==1)awake++;});peakAwake=Math.max(peakAwake,awake);
      if(scenario==='near-misses' && t<2) assert.equal(awake,0,'first flight is a miss; later physical ricochets may hit');
      peakLive=Math.max(peakLive,e.sandbox_projectile_count());
      const stats=e.stepStats();maxIterations=Math.max(maxIterations,stats.fixed_impulse_iterations);
      hash.update(JSON.stringify(all));hash.update(JSON.stringify(all.map((_,i)=>e.bodySleeping(i))));
      assert(stats.fixed_substeps<=4&&stats.fixed_impulse_iterations<=32&&stats.fixed_position_passes<=8,'unchanged bounded work');
    }
    assert(Math.abs(e.elapsed()-before-ticks/60)<1e-7,'all requested time advanced');
    assert.equal(e.sandbox_body_count()-e.sandbox_projectile_count(),44,'no missing player/crates/world');
    assert(peakFloor<=0.5,`penetration ${peakFloor}: ${scenario}/${characterResponse}/${crateMotion}/${projectileImpactPolicy}`);
    if(scenario==='near-misses'){if(projectileImpactPolicy==='impact-retire'){assert(!changed,'misses changed tower');assert.equal(peakAwake,0);}}else assert(changed,'hits must affect tower');
    if(crateMotion==='upright')assert(!rotated,'rotation lock');
    const sorted=times.toSorted((a,b)=>a-b);
    runs.push({trial,shots,peakFloor,peakAwake,peakLive,maxIterations,changed,rotated,hash:hash.digest('hex'),mean_ms:times.reduce((a,b)=>a+b,0)/times.length,p95_ms:sorted[Math.floor(sorted.length*.95)],max_ms:sorted.at(-1),raw_ms:times});
   }
   assert(runs.every(r=>r.hash===runs[0].hash),'same-build replay');
   records.push({characterResponse,crateMotion,projectileImpactPolicy,scenario,runs,passed:true});
   console.log(JSON.stringify({...records.at(-1),runs:runs.map(({raw_ms,...r})=>r)}));
   if(outputPath)writeFileSync(outputPath,JSON.stringify({passed:false,in_progress:true,records},null,2));
  }
 }
}
const result={passed:true,kind:'production-tower-projectiles-v1',node:process.version,cpu:os.cpus()[0]?.model,wasm_sha256:createHash('sha256').update(bytes).digest('hex'),ticks,trials,note:'Same production runtime; equal simulated duration. Timings exclude snapshot checks and include sleeping periods; no wall-clock gate.',records};
if(outputPath)writeFileSync(outputPath,JSON.stringify(result,null,2)+'\n');
