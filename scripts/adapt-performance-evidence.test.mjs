import assert from "node:assert/strict";
import { createHash } from "node:crypto";
import { mkdtemp, mkdir, readFile, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import test from "node:test";

import { buildCanonicalPerformanceEvidence } from "./adapt-performance-evidence.mjs";

const sha256 = (value) => `sha256:${createHash("sha256").update(value).digest("hex")}`;

test("sandbox adapter emits canonical head and baseline evidence", async () => {
  const temporary = await mkdtemp(join(tmpdir(), "physics-performance-evidence-"));
  const evidence = join(temporary, "evidence");
  await mkdir(evidence, { recursive: true });
  await writeFile(join(evidence, "benchmark-sandbox.mjs"), "// deterministic benchmark\n");
  await writeFile(
    join(evidence, "performance-contract.json"),
    JSON.stringify({ scenarios: [{ id: "settled-idle", dimensions: { ticks: 180, projectiles: 0 } }] }),
  );
  await writeFile(
    join(evidence, "sandbox.json"),
    JSON.stringify({
      workload: "sandbox-projectiles-v5",
      environment: { node: "v24", v8: "13", platform: "linux", arch: "x64", cpu: "test" },
      head_revision: "head-revision",
      baseline_revision: "base-revision",
      baseline: {
        cases: [{
          name: "settled-idle",
          trials: [{
            steps: { count: 180, mean_ms: 0.2, p50_ms: 0.18, p95_ms: 0.3, max_ms: 0.4 },
            replay_sha256: "baseline-replay",
            body_count: 5,
            event_sum: 0,
            work: { sampled_events: 0, broad_phase_queries: 0 },
          }],
        }],
      },
      head: {
        cases: [{
          name: "settled-idle",
          trials: [{
            steps: { count: 180, mean_ms: 0.1, p50_ms: 0.09, p95_ms: 0.2, max_ms: 0.3 },
            replay_sha256: "head-replay",
            body_count: 5,
            event_sum: 0,
            work: { sampled_events: 0, broad_phase_queries: 0 },
          }],
        }],
      },
    }),
  );

  const paths = await buildCanonicalPerformanceEvidence({ evidenceDirectory: evidence, sourceDirty: false });
  assert.deepEqual(paths, [
    "canonical/sandbox/settled-idle/baseline-trial-1.json",
    "canonical/sandbox/settled-idle/head-trial-1.json",
  ]);

  const baselineBytes = await readFile(join(evidence, paths[0]));
  const head = JSON.parse(await readFile(join(evidence, paths[1]), "utf8"));
  assert.equal(head.schema_version, "1.0.0");
  assert.equal(head.source.revision, "head-revision");
  assert.equal(head.baseline.source_revision, "base-revision");
  assert.equal(head.baseline.evidence_hash, sha256(baselineBytes));
  assert.equal(head.scenario.workload.parameters.ticks, 180);
  assert.ok(head.measurements.useful_work.some((entry) => entry.name === "physics.simulation_steps"));
  assert.ok(head.measurements.induced_work.some((entry) => entry.name === "physics.broad_phase_queries"));
  assert.ok(head.measurements.outcomes.some((entry) => entry.name === "physics.step.p95_ms"));
  assert.ok(head.artifacts.some((entry) => entry.path === "sandbox.json"));
});

test("browser adapter preserves the raw session as hashed supporting evidence", async () => {
  const temporary = await mkdtemp(join(tmpdir(), "physics-browser-evidence-"));
  const evidence = join(temporary, "evidence");
  await mkdir(join(evidence, "sessions"), { recursive: true });
  const sessionPath = join(evidence, "sessions", "capture.json");
  await writeFile(
    sessionPath,
    JSON.stringify({
      schema_version: 2,
      kind: "physics-engine-browser-session",
      duration_ms: 1000,
      environment: { build: { revision: "browser-revision", dirty: false }, browser: "test-browser" },
      scenario: { projectile_policy: "physical" },
      summary: {
        recorded_frames: 60,
        omitted_frames: 0,
        omitted_markers: 0,
        physics_step_count: 60,
        physics_work: { sampled_events: 4, broad_phase_queries: 10 },
        frames: { mean_ms: 16.6, p95_ms: 17.1, max_ms: 20 },
        callbacks: { p95_ms: 2 },
        rendering: { p95_ms: 1 },
        dropped_accumulator_ms: 0,
        projectile_lifecycle: { retired_on_contact: 0 },
      },
    }),
  );

  const paths = await buildCanonicalPerformanceEvidence({ evidenceDirectory: evidence, sourceDirty: true });
  assert.deepEqual(paths, ["canonical/browser/capture.evidence.json"]);
  const canonical = JSON.parse(await readFile(join(evidence, paths[0]), "utf8"));
  assert.equal(canonical.source.revision, "browser-revision");
  assert.equal(canonical.source.dirty, false);
  assert.equal(canonical.artifacts[0].path, "sessions/capture.json");
  assert.equal(canonical.artifacts[0].sha256, sha256(await readFile(sessionPath)));
  assert.ok(canonical.measurements.induced_work.some((entry) => entry.name === "physics.sampled_events"));
  assert.ok(canonical.measurements.outcomes.some((entry) => entry.name === "browser.frame.p95_ms"));
});
