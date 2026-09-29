// Pure reporting: time is advisory; rejected quality never becomes usable capacity.
export const PROFILES = [
  {id:'4s-8v-2p',substeps:4,velocity:8,position:2},
  {id:'4s-6v-2p',substeps:4,velocity:6,position:2},
  {id:'3s-8v-2p',substeps:3,velocity:8,position:2},
  {id:'4s-4v-2p',substeps:4,velocity:4,position:2},
  {id:'4s-2v-2p',substeps:4,velocity:2,position:2},
  {id:'4s-1v-2p',substeps:4,velocity:1,position:2},
  {id:'4s-8v-1p',substeps:4,velocity:8,position:1},
  {id:'4s-8v-0p',substeps:4,velocity:8,position:0},
  {id:'2s-8v-2p',substeps:2,velocity:8,position:2},
  {id:'2s-4v-1p',substeps:2,velocity:4,position:1},
  {id:'2s-2v-1p',substeps:2,velocity:2,position:1},
  {id:'1s-8v-2p',substeps:1,velocity:8,position:2},
  {id:'1s-2v-1p',substeps:1,velocity:2,position:1},
  {id:'1s-1v-0p',substeps:1,velocity:1,position:0},
];
export const THRESHOLDS = {floor:0.5,boxOverlap:1.8,unforcedDisplacement:1.8};
export const SCENES=['sustained-stack','mixed-impacts','sleeping-control','shallow-contact'];
export function summary(values) {
  if (!values.length) return {count:0,mean_ms:null,p95_ms:null,max_ms:null,total_ms:0};
  const sorted=values.toSorted((a,b)=>a-b),total=values.reduce((a,b)=>a+b,0);
  return {count:values.length,mean_ms:total/values.length,p95_ms:sorted[Math.ceil(sorted.length*.95)-1],max_ms:sorted.at(-1),total_ms:total};
}
export function qualityFailures(m,scene) {
  const reasons=[];
  if(m[0]!==1) reasons.push('finite-state-or-inventory');
  if(m[1]>THRESHOLDS.floor) reasons.push('floor-penetration');
  if(m[2]>THRESHOLDS.boxOverlap) reasons.push('box-box-penetration');
  if(scene!==1 && m[6]>THRESHOLDS.unforcedDisplacement) reasons.push('unforced-stack-displacement');
  return reasons;
}
export function capacity(records,counts,profiles,budgets=[4,8,1000/60]) {
  return [SCENES[0],SCENES[1],SCENES[3]].flatMap(scene=>profiles.flatMap(p=>budgets.map(budget=>{
    let qualified=0,raw=0,blocked=false;
    for(const count of counts) {
      const rows=records.filter(r=>r.scene===scene&&r.profile.id===p.id&&r.boxes===count);
      const within=rows.length>0&&rows.every(r=>r.completed&&r.timing.active.p95_ms!==null&&r.timing.active.p95_ms<=budget);
      if(within) raw=count;
      if(!blocked&&within&&rows.every(r=>r.quality.passed&&r.repeatable)) qualified=count;
      else blocked=true;
    }
    return {scene,profile:p.id,budget_ms:budget,largest_raw_timed_count:raw,largest_contiguous_quality_count:qualified,
      qualified_at_grid_limit:qualified===counts.at(-1),note:'Discrete tested grid; no extrapolation, frame includes physics only. Worst trial p95, all quality gates.'};
  })));
}
