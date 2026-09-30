import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import test from 'node:test';
import {assertDenseQuality} from './check-dense-quality.mjs';
const evidence=JSON.parse(readFileSync(new URL('../../docs/dense-contact-quality-2026-09-30.json',import.meta.url),'utf8'));
function report(baseline=false) {
  const records=baseline?evidence.baseline_300_tick_controls:evidence.wasm_1200_tick_controls.filter(r=>['sustained-stack','mixed-impacts'].includes(r.scene));
  return {schema:'physics-solver-budget/v1',counts:[32,64,128],scenes:['sustained-stack','mixed-impacts'],profiles:evidence.profiles,thresholds:evidence.thresholds,trials:2,ticks:baseline?300:1200,dt:1/60,solver_policy:evidence.policy,
    records:structuredClone(records).map(r=>({...r,profile:evidence.profiles[0],step_error:null}))};
}
test('actual retained repaired traces pass the quality gate',()=>assertDenseQuality(report()));
test('actual historical failure rejects at the physical symptom',()=>assert.throws(()=>assertDenseQuality(report(true)),/128\/mixed-impacts.*quality failure.*box-box-penetration/));
test('missing population cannot become a passing result',()=>{const r=report();r.records=r.records.filter(x=>x.boxes!==64);assert.throws(()=>assertDenseQuality(r));});
test('late failed or incomplete steps are rejected',()=>{const r=report();r.records.at(-1).completed=false;r.records.at(-1).step_error={tick:1199,code:-2};assert.throws(()=>assertDenseQuality(r),/incomplete time/);});
test('a claimed pass cannot hide penetration or replay mismatch',()=>{
  const r=report();r.records.at(-1).quality.peak_box_overlap=4.75;assert.throws(()=>assertDenseQuality(r),/penetration/);
  const repeated=report();repeated.records.at(-1).replay_sha256='0'.repeat(64);assert.throws(()=>assertDenseQuality(repeated),/replay mismatch/);
});
