// Feature isolation is a build contract, not a runtime feature toggle.
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {pathToFileURL} from 'node:url';
const modes = new Set(['production', 'soft', 'budget', 'combined']);
export function checkDiagnosticExports(exports, mode) {
  assert(modes.has(mode), `Unknown diagnostic export mode: ${mode}`);
  assert.equal(typeof exports.approximate_reset_tower, 'function', 'Canonical tower export missing');
  for (const [name, enabled] of [
    ['approximate_reset_soft_from_sandbox', mode === 'soft' || mode === 'combined'],
    ['approximate_reset_tower_budget', mode === 'budget' || mode === 'combined'],
  ]) {
    assert.equal(typeof exports[name] === 'function', enabled, `${name}: unexpected ${mode} feature surface`);
  }
}
if (process.argv[1] && import.meta.url === pathToFileURL(process.argv[1]).href) {
  const [wasm, mode = 'production'] = process.argv.slice(2);
  assert(wasm, 'Usage: node scripts/check-diagnostic-exports.mjs module.wasm [production|soft|budget|combined]');
  const {instance} = await WebAssembly.instantiate(readFileSync(wasm), {});
  checkDiagnosticExports(instance.exports, mode);
  assert.equal(instance.exports.sandbox_numeric_backend(), 64, 'Diagnostic Pages modules still use f64 by default');
  console.log(`Verified ${mode} WASM feature surface`);
}
