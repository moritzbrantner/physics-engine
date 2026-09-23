import test from 'node:test';
import assert from 'node:assert/strict';
import {checkDiagnosticExports} from './check-diagnostic-exports.mjs';
const base = {approximate_reset_tower() {}};
const soft = {approximate_reset_soft_from_sandbox() {}};
const budget = {approximate_reset_tower_budget() {}};
test('production rejects either diagnostic entry point', () => {
  checkDiagnosticExports(base, 'production');
  assert.throws(() => checkDiagnosticExports({...base,...soft}, 'production'));
  assert.throws(() => checkDiagnosticExports({...base,...budget}, 'production'));
});
test('explicit feature combinations must match the compiled exports', () => {
  checkDiagnosticExports({...base,...soft}, 'soft');
  checkDiagnosticExports({...base,...budget}, 'budget');
  checkDiagnosticExports({...base,...soft,...budget}, 'combined');
  assert.throws(() => checkDiagnosticExports(base, 'combined'));
  assert.throws(() => checkDiagnosticExports(base, 'unknown'));
  assert.throws(() => checkDiagnosticExports({}, 'production'));
});
