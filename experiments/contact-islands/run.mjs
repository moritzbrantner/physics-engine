// Same-input global/island stopping comparisons, with optional immutable pre-change WASM.
// Scene construction is diagnostic; all collision/integration decisions remain Rust-owned.
import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { readFileSync, writeFileSync } from 'node:fs';
import os from 'node:os';
const [candidatePath, output, baselinePath] = process.argv.slice(2);
if (!candidatePath || !output) throw new Error('Usage: node experiments/contact-islands/run.mjs candidate.wasm report.json [baseline.wasm]');
const trials = Number(process.env.TRIALS ?? 4), ticks = Number(process.env.TICKS ?? 600);
assert(Number.isInteger(trials) && trials >= 1 && trials <= 9);
assert(Number.isInteger(ticks) && ticks >= 300 && ticks <= 3600);
const hash = x => createHash('sha256').update(x).digest('hex');
const bytes = { candidate: readFileSync(candidatePath), ...(baselinePath ? { baseline: readFileSync(baselinePath) } : {}) };
const modules = Object.fromEntries(await Promise.all(Object.entries(bytes).map(async ([key,data])=>[key,await WebAssembly.compile(data)])));
const modes = baselinePath ? ['baseline','global','islands'] : ['global','islands'];
const statsNames = ['elapsed','substeps','rounds','row_visits','residual_rows','skipped_rounds','contact_rows','narrow_tests',
  'woken','retired','max_position_correction','max_exit_velocity_residual','max_exit_impulse_delta','swept','quiet','tick',
  'partition_builds','rows_indexed','endpoints_checked','dynamic_nodes','union_attempts','root_links_followed','islands',
  'max_island_rows','single_island_substeps','island_iterations','converged_islands','capped_islands',
  'skipped_constraint_visits','scratch_growths','scratch_retained_bytes','epoch_resets','shared_prefix_iterations','prefix_converged_substeps'];
const gauges = new Set([0,10,11,12,15,23,30]);
function summary(x) {
  if (!x.length) return { count:0,total_ms:0,mean_ms:null,p95_ms:null,max_ms:null };
  const sorted=x.toSorted((a,b)=>a-b),sum=x.reduce((a,b)=>a+b,0);
  return {count:x.length,total_ms:sum,mean_ms:sum/x.length,p95_ms:sorted[Math.ceil(x.length*.95)-1],max_ms:sorted.at(-1)};
}
function snapshot(e) { const p=e.island_snapshot();return new Float64Array(e.memory.buffer,p,e.island_snapshot_len()).slice(); }
const configurations = [
  {scene:0,easy:32,name:'mixed-rest-32'}, {scene:0,easy:128,name:'mixed-rest-128'},
  {scene:1,easy:32,name:'mixed-hit-32'}, {scene:1,easy:128,name:'mixed-hit-128'},
  {scene:2,easy:32,name:'independent-32'}, {scene:2,easy:128,name:'independent-128'},
  {scene:3,easy:0,name:'hard-only-hit'}, {scene:4,easy:128,name:'sleeping-miss'},
  {scene:5,easy:16,name:'swept-bridge'},
].filter(c=>!process.env.CASES || process.env.CASES.split(',').includes(c.name));
assert(configurations.length>0);
async function replay(mode, config, measured=true) {
  const e=(await WebAssembly.instantiate(mode==='baseline'?modules.baseline:modules.candidate,{})).exports;
  assert.equal(e.island_reset(config.scene,config.easy,mode==='islands'?1:0),0);
  const initial=snapshot(e), expectedIds=[];
  for(let i=0;i<initial.length;i+=16) expectedIds.push(initial[i]);
  const history=[],physics=createHash('sha256'),workHash=createHash('sha256'),legacyWork=createHash('sha256');
  const phases={startup:[],active:[],sleeping:[]};
  const work=Object.fromEntries(Object.keys(phases).map(p=>[p,Array(statsNames.length).fill(0)]));
  let peakFloor=0,peakDrift=0,peakBridgeRows=0,retirements=0,wakes=0;const changedBridge=new Set();
  let sleeperSnapshot;
  // Main timing window is AFTER 240 startup ticks. Those startup timings/quality remain separate.
  const count=measured ? 240+ticks : 300;
  for(let t=0;t<count;t++) {
    const scheduledInput=t===240 && [1,3,4,5].includes(config.scene);
    const phase=t<240?'startup':!scheduledInput && e.island_stat(14)===1?'sleeping':'active';
    const before=performance.now(),code=e.island_step(),elapsed=performance.now()-before;
    assert.equal(code,0,`${mode}/${config.name}/tick${t}`);
    if (!measured) continue;
    phases[phase].push(elapsed);
    const s=snapshot(e),stat=statsNames.map((_,i)=>e.island_stat(i));
    assert(s.every(Number.isFinite));assert(stat.slice(0,16).every(Number.isFinite));
    if(mode!=='baseline')assert(stat.every(Number.isFinite));
    assert(Math.abs(stat[0]-(t+1)/60)<1e-7,'complete requested time');
    assert(stat[1]<=4&&stat[2]<=32);
    assert.equal(stat[2]+stat[5],stat[1]*8,'world-round accounting');
    if(mode==='islands')assert.equal(stat[3]+stat[28],stat[6]*8,'actual visits plus proven skips equal the unchanged row budget');
    const ids=[];let drift=0;
    for(let k=0;k<s.length;k+=16) {
      assert(Math.abs(Math.hypot(...s.slice(k+4,k+8))-1)<1e-8);
      if(s[k]===2000)continue;
      ids.push(s[k]);
      if(s[k]===0)continue;
      peakFloor=Math.max(peakFloor,-s[k+15]);
      const original=expectedIds.indexOf(s[k])*16;
      drift=Math.max(drift,Math.hypot(...[1,2,3].map(j=>s[k+j]-initial[original+j])));
      if(config.scene===0||config.scene===2||t<240) peakDrift=Math.max(peakDrift,drift);
      if(t>=245 && s[k]<100 && config.scene===5) {if(Math.abs(s[k+3]-initial[original+3])>1e-4) changedBridge.add(s[k]);}
      if(config.scene<=3)assert.equal(s[k+14],0,'awake fixture is not a sleeping-speed benchmark');
    }
    assert.deepEqual(ids,expectedIds,'no disappearing body');
    if(config.scene===4) {
      if(t===239) {sleeperSnapshot=s;assert.equal(stat[14],1,'fixture must naturally settle');}
      if(t>=240) {
        assert(sleeperSnapshot);
        for(let k=0;k<sleeperSnapshot.length;k++)assert.equal(s[k],sleeperSnapshot[k],'miss disturbed sleepers');
      }
    }
    if(t>=240) {
      retirements+=stat[9];wakes+=stat[8];
      if(config.scene===5&&mode==='islands')peakBridgeRows=Math.max(peakBridgeRows,stat[23]);
    }
    assert(peakFloor<=.5,`floor penetration ${peakFloor}`);
    assert(peakDrift<=1.8,`unforced drift ${peakDrift}`);
    physics.update(new Uint8Array(s.buffer));
    legacyWork.update(new Uint8Array(new Float64Array(stat.slice(0,16)).buffer));
    workHash.update(new Uint8Array(new Float64Array(stat.filter(Number.isFinite)).buffer));
    for(let i=0;i<stat.length;i++)if(Number.isFinite(stat[i])) work[phase][i]=gauges.has(i)?Math.max(work[phase][i],stat[i]):work[phase][i]+stat[i];
    history.push(s);
  }
  if(!measured)return;
  if([1,3,5].includes(config.scene)) assert(retirements>=1,'genuine projectile must hit');
  if(config.scene===5) {
    assert.equal(changedBridge.size,2,'bridge impact must affect BOTH initially disconnected boxes');
    assert(wakes>=2,'both sleeping bridge targets must wake');
    if(mode==='islands')assert(peakBridgeRows>4,'new swept contact must combine constraints before further solving');
  }
  return {physics_hash:physics.digest('hex'),work_hash:workHash.digest('hex'),legacy_work_hash:legacyWork.digest('hex'),history,
    peak_floor_penetration:peakFloor,peak_unforced_drift:peakDrift,peak_bridge_rows:peakBridgeRows,
    retirements,wakes,
    phases:Object.fromEntries(Object.keys(phases).map(p=>[p,{timing:summary(phases[p]),raw_ms:phases[p],
      work:Object.fromEntries(statsNames.map((n,i)=>[n,mode==='baseline'&&i>=16?null:work[p][i]]))}]))};
}
function compare(a,b) {
  assert.equal(a.length,b.length);let position=0,velocity=0,orientation=0,sleep=0;
  for(let t=0;t<a.length;t++) {
    assert.equal(a[t].length,b[t].length,'identical lifecycle at each tick');
    for(let i=0;i<a[t].length;i+=16) {
      assert.equal(a[t][i],b[t][i]);
      for(const j of [1,2,3])position=Math.max(position,Math.abs(a[t][i+j]-b[t][i+j]));
      for(const j of [4,5,6,7])orientation=Math.max(orientation,Math.abs(a[t][i+j]-b[t][i+j]));
      for(const j of [8,9,10,11,12,13])velocity=Math.max(velocity,Math.abs(a[t][i+j]-b[t][i+j]));
      sleep+=a[t][i+14]!==b[t][i+14]?1:0;
    }
  }
  // Explicit paired-trajectory regression bound; does not replace independent quality above.
  assert(position<=1e-3&&velocity<=1e-3&&orientation<=1e-4&&sleep===0,
    `scope delta position${position},velocity${velocity},orientation${orientation},sleep${sleep}`);
  return {max_position_delta:position,max_linear_or_angular_velocity_delta:velocity,
    max_quaternion_component_delta:orientation,sleep_flag_differences:sleep};
}
const report={schema:'contact-island-convergence-v1',passed:false,in_progress:true,trials,ticks,
  environment:{cpu:os.cpus()[0]?.model,node:process.version,platform:process.platform,arch:process.arch},
  wasm_sha256:Object.fromEntries(Object.entries(bytes).map(([k,b])=>[k,hash(b)])),
  note:'Physics-only release WASM, single thread. Same 60Hz tick, four substeps and eight-pass ceiling. Timings INCLUDE grouping, insertion and integration but exclude snapshots/assertions. Startup (240 ticks) and active/sleeping windows separate. Alternate mode order. No wall-clock gate; no failed run is a speedup denominator.',records:[]};
try {
  for(const c of configurations) {
    for(const mode of modes)await replay(mode,c,false);
    const record={config:c,runs:[]};report.records.push(record);
    for(let trial=0;trial<trials;trial++) {
      const results={},order=[...modes.slice(trial%modes.length),...modes.slice(0,trial%modes.length)];
      if(Math.floor(trial/modes.length)%2)order.reverse();
      for(const mode of order)results[mode]=await replay(mode,c);
      if(results.baseline) assert.equal(results.global.physics_hash,results.baseline.physics_hash,
        'refactored WholeWorld physics differs from immutable baseline');
      if(results.baseline)assert.equal(results.global.legacy_work_hash,results.baseline.legacy_work_hash,'global reference changed old work observations');
      const difference=compare(results.global.history,results.islands.history);
      if(record.runs.length)for(const mode of modes)for(const key of ['physics_hash','work_hash'])
        assert.equal(results[mode][key],record.runs[0][mode][key],`same-mode ${mode} repeatability`);
      for(const r of Object.values(results))delete r.history;
      record.runs.push({trial,order,difference,...results});
      writeFileSync(output,JSON.stringify(report));
    }
    record.phases=Object.fromEntries(['startup','active','sleeping'].map(p=>[p,Object.fromEntries(modes.map(m=>[m,
      summary(record.runs.flatMap(r=>r[m].phases[p].raw_ms))]))]));
    record.passed=true;
    console.log(JSON.stringify({config:c,passed:true,active:record.phases.active,
      row_visits: Object.fromEntries(modes.map(m=>[m,record.runs[0][m].phases.active.work.row_visits]))}));
  }
  report.passed=true;report.in_progress=false;
} catch(error) {report.failure=String(error);report.in_progress=false;throw error;}
finally {writeFileSync(output,JSON.stringify(report,null,2)+'\n');}
