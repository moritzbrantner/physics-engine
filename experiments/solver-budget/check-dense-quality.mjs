import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import {THRESHOLDS} from './report.mjs';

export function assertDenseQuality(report) {
  assert.equal(report.schema,'physics-solver-budget/v1');
  assert.deepEqual(report.counts,[32,64,128]);
  assert.deepEqual(report.scenes,['sustained-stack','mixed-impacts']);
  assert.equal(report.profiles.length,1);
  assert.deepEqual(report.profiles[0],{id:'4s-8v-2p',substeps:4,velocity:8,position:2});
  assert.deepEqual(report.thresholds,THRESHOLDS);
  assert(Number.isInteger(report.trials)&&report.trials>=2);
  assert(Number.isInteger(report.ticks)&&report.ticks>=300);
  assert.equal(report.dt,1/60);
  assert.equal(report.records.length,6*report.trials);
  for(const boxes of report.counts)for(const scene of report.scenes) {
    const rows=report.records.filter(r=>r.boxes===boxes&&r.scene===scene);
    assert.equal(rows.length,report.trials,`${boxes}/${scene}: missing control`);
    assert.deepEqual(rows.map(r=>r.trial).sort((a,b)=>a-b),Array.from({length:report.trials},(_,i)=>i),`${boxes}/${scene}: missing repetition`);
    for(const row of rows) {
      const label=`${boxes}/${scene}/trial${row.trial}`;
      assert(row.completed&&row.completed_ticks===report.ticks,`${label}: incomplete time`);
      assert.equal(row.step_error,null,`${label}: returned step error`);
      assert(row.quality.passed&&row.quality.reasons.length===0,`${label}: quality failure ${row.quality.reasons.join(',')}`);
      assert(row.quality.peak_box_overlap<=THRESHOLDS.boxOverlap,`${label}: penetration`);
      assert(row.quality.peak_floor<=THRESHOLDS.floor,`${label}: floor penetration`);
      if(scene==='sustained-stack')assert(row.quality.peak_displacement<=THRESHOLDS.unforcedDisplacement,`${label}: unforced drift`);
      assert.equal(row.quality.peak_awake_boxes,boxes,`${label}: active population`);
      assert(Math.abs(row.elapsed_seconds-report.ticks/60)<=1e-8,`${label}: elapsed time`);
      assert.deepEqual(row.profile,report.profiles[0],`${label}: changed budget`);
      assert(row.work.substeps<=report.ticks*4&&row.work.velocity_passes<=report.ticks*32&&row.work.position_passes<=report.ticks*8,`${label}: work ceiling`);
      assert(row.repeatable&&row.replay_sha256===rows[0].replay_sha256,`${label}: replay mismatch`);
    }
  }
  assert.equal(report.solver_policy.position_correction,'admitted-contacts');
  assert.equal(report.solver_policy.soft_contact,false);
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url)) {
  const path=process.argv[2];assert(path,'usage: node check-dense-quality.mjs report.json');
  assertDenseQuality(JSON.parse(readFileSync(path,'utf8')));
  console.log('Dense contact quality: original 32/64/128 stack and impact controls completed and repeated under unchanged physical bounds and work ceilings.');
}
