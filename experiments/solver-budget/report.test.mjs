import test from 'node:test';import assert from 'node:assert/strict';
import {summary,qualityFailures,capacity} from './report.mjs';
test('p95 uses nearest rank and empty timings are not zero',()=>{assert.equal(summary([]).p95_ms,null);assert.equal(summary(Array.from({length:20},(_,i)=>i+1)).p95_ms,19);});
test('fast but penetrated bodies fail quality',()=>{assert.deepEqual(qualityFailures([1,.02,3,32,100,32,0,0],0),['box-box-penetration']);});
test('capacity never skips a failed smaller population or treats a missing case as passed',()=>{
 const p={id:'test'},r=n=>({scene:'sustained-stack',profile:p,boxes:n,completed:true,repeatable:true,quality:{passed:true},timing:{active:{p95_ms:1}}});
 const a=r(16),b=r(32),c=r(64);b.quality.passed=false;
 const out=capacity([a,b,c],[16,32,64],[p],[8]);
 assert.equal(out[0].largest_raw_timed_count,64);assert.equal(out[0].largest_contiguous_quality_count,16);assert.equal(out[1].largest_contiguous_quality_count,0);
});
