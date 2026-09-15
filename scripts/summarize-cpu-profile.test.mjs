import assert from "node:assert/strict";
import test from "node:test";

import { summarizeCpuProfile } from "./summarize-cpu-profile.mjs";

test("CPU profile summary ranks self time and strips local file paths", () => {
  const summary = summarizeCpuProfile({
    nodes: [
      { id: 1, callFrame: { functionName: "step", url: "wasm://engine/abc", lineNumber: 0 } },
      { id: 2, callFrame: { functionName: "measure", url: "file:///tmp/private/benchmark.mjs", lineNumber: 9 } },
    ],
    samples: [2, 1, 1],
    timeDeltas: [1_000, 4_000, 5_000],
  });

  assert.equal(summary.total_sampled_ms, 10);
  assert.equal(summary.hot_functions[0].function, "step");
  assert.equal(summary.hot_functions[0].self_percent, 90);
  assert.equal(summary.hot_functions[1].url, "benchmark.mjs");
});

test("CPU profile summary rejects incomplete sampling evidence", () => {
  assert.throws(
    () => summarizeCpuProfile({ nodes: [], samples: [1], timeDeltas: [] }),
    /equal length/,
  );
});
