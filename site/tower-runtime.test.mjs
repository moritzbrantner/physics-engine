import test from 'node:test';
import assert from 'node:assert/strict';
import {createTowerRuntime} from './tower-runtime.mjs';
import {readFileSync} from 'node:fs';

test('tower refuses a stale event-only build rather than silently falling back', () => {
  assert.throws(()=>createTowerRuntime({memory:new WebAssembly.Memory({initial:1}),sandbox_step_velocity:()=>0}),/missing approximate_reset_tower/);
});
test('tower adapter routes all simulation commands to the bounded Rust world', () => {
  const called=[], memory=new WebAssembly.Memory({initial:1});
  const e=new Proxy({memory},{get(target,key){
    if(key==='memory')return memory;
    assert.match(key,/^approximate_/,`legacy API accessed: ${key}`);
    return (...args)=>{called.push([key,...args]);return 0;};
  }});
  const tower=createTowerRuntime(e);
  tower.sandbox_reset_tower_with_baking_options(123,456,1);
  tower.sandbox_step_velocity(0,60,1);tower.sandbox_shoot(0,0,-96);
  assert.deepEqual(called,[['approximate_reset_tower',123],['approximate_step_velocity',0,60,1],['approximate_shoot',0,0,-96]]);
  assert.equal(tower.snapshotArray,Float64Array);
  assert.equal(tower.orientationScale,1);
  assert.equal(tower.sandbox_last_sampled_events,undefined);
});
test('canonical tower and app explicitly select the bounded runtime', () => {
  const html=readFileSync(new URL('./scenarios/tower/index.html',import.meta.url),'utf8');
  assert.match(html,/data-scenario="tower"/);
  const app=readFileSync(new URL('./app.js',import.meta.url),'utf8');
  assert.match(app,/if \(scenarioId === "tower"\) engine = createTowerRuntime\(engine\)/);
  assert.match(app,/engine\.snapshotArray \?\? Int32Array/);
  const settings=readFileSync(new URL('./physics-settings.mjs',import.meta.url),'utf8');
  assert.match(settings,/fixedStepTower && id === "simulation.projectile_type"/);
});

test('closing settings restores gameplay focus and keyboard pause cannot hide a failed state', () => {
  const settings = readFileSync(new URL('./physics-settings.mjs', import.meta.url), 'utf8');
  const app = readFileSync(new URL('./app.js', import.meta.url), 'utf8');
  assert.match(settings, /document\.querySelector\("#scene"\)\?\.focus\(\{ preventScroll: true \}\)/);
  assert.match(app, /event\.code === "KeyP" && !event\.repeat && !simulationError/);
});
