// Small CI regression gate; broad sweeps still retain rejected experimental settings.
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
const [path] = process.argv.slice(2);
assert(path, 'Usage: node experiments/solver-budget/check-smoke.mjs report.json');
const report = JSON.parse(readFileSync(path, 'utf8'));
assert.equal(report.solver_policy.convergence_scope, 'contact-islands');
assert.equal(report.solver_policy.soft_contact, false);
assert(report.trials >= 2);
assert(report.records.every(r => r.completed && r.repeatable), 'Smoke must finish and repeat');
for (const scene of ['sustained-stack','shallow-contact']) {
  const reference = report.records.filter(r => r.boxes === 32 && r.profile.id === '4s-8v-2p' && r.scene === scene);
  assert.equal(reference.length, report.trials, 'Missing reference cases');
  assert(reference.every(r => r.quality.passed), `${scene}: established small reference failed`);
}
console.log('Budget smoke reference and replay checks passed; experimental quality failures stay explicit');
