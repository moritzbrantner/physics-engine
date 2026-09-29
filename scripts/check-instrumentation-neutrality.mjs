import assert from "node:assert/strict";
import { spawnSync } from "node:child_process";

const marker = "COUNTER_SNAPSHOT ";
function cargo(args) {
  const result = spawnSync("cargo", args,
    { cwd: new URL("../", import.meta.url), encoding: "utf8", maxBuffer: 16 * 1024 * 1024 });
  assert.ifError(result.error);
  assert.equal(result.status, 0, `${result.stdout}\n${result.stderr}`);
  return result.stdout;
}
function snapshots(features) {
  const output = cargo([
    "test", "--locked", ...features, "--test", "instrumentation_neutrality",
    "--", "--nocapture", "--test-threads=1",
  ]);
  const values = output.split("\n")
    .filter((line) => line.includes(marker))
    .map((line) => line.slice(line.indexOf(marker)));
  assert(values.length >= 30, "physical snapshot fixtures did not execute");
  return values;
}

// The canonical root test capability also retains the full existing Cargo suite.
if (process.argv.includes("--full")) cargo(["test", "--locked"]);
const enabled = snapshots([]);
const disabled = snapshots(["--no-default-features"]);
assert.deepEqual(disabled, enabled, "disabling work counters changed physical results");
console.log(`Instrumentation neutrality passed: ${enabled.length} physical snapshots match across both builds.`);
