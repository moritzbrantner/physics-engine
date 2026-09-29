// Create a compact CSV without combining different machines or measurement sessions.
import {readFileSync,writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
const [input,output]=process.argv.slice(2);assert(input&&output,'summarize.mjs sweep.json summary.csv');
const r=JSON.parse(readFileSync(input,'utf8'));
assert.equal(r.schema,'physics-solver-budget/v1');
const expected=r.counts.length*r.profiles.length*r.scenes.length*r.trials;
assert.equal(r.records.length,expected,'Incomplete reports cannot be summarized as complete sweeps');
const columns=['scene','profile','dynamic_boxes','trials','all_completed','all_quality_pass','repeatable',
  'mean_active_ms','worst_trial_p95_ms','max_active_ms','peak_floor_penetration','peak_box_penetration','peak_unforced_or_impact_displacement',
  'mean_touching_boxes','mean_touching_box_pairs','mean_velocity_passes','mean_position_passes','mean_contact_points_across_substeps','mean_constraint_visits','quality_failures'];
const lines=[columns];
for(const scene of r.scenes)for(const p of r.profiles)for(const count of r.counts) {
  const rows=r.records.filter(x=>x.scene===scene&&x.profile.id===p.id&&x.boxes===count);
  assert.equal(new Set(rows.map(x=>x.trial)).size,r.trials);
  const frames=rows.reduce((n,x)=>n+x.timing.active.count,0),mean=k=>rows.reduce((n,x)=>n+x.phase_work.active[k],0)/frames;
  lines.push([scene,p.id,count,rows.length,rows.every(x=>x.completed),rows.every(x=>x.quality.passed),rows.every(x=>x.repeatable),
    frames?rows.reduce((n,x)=>n+x.timing.active.total_ms,0)/frames:null,
    Math.max(...rows.map(x=>x.timing.active.p95_ms??Infinity)),Math.max(...rows.map(x=>x.timing.active.max_ms??0)),
    Math.max(...rows.map(x=>x.quality.peak_floor)),Math.max(...rows.map(x=>x.quality.peak_box_overlap)),Math.max(...rows.map(x=>x.quality.peak_displacement)),
    rows.reduce((n,x)=>n+x.quality.mean_touching_boxes,0)/rows.length,rows.reduce((n,x)=>n+x.quality.mean_touching_pairs,0)/rows.length,
    mean('velocity_passes'),mean('position_passes'),mean('contact_points'),mean('constraint_visits'),
    [...new Set(rows.flatMap(x=>x.quality.reasons))].join(';')]);
}
writeFileSync(output,lines.map(line=>line.map(x=>`"${String(x??'').replaceAll('"','""')}"`).join(',')).join('\n')+'\n');
