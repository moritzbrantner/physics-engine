import assert from "node:assert/strict";
import { mkdtemp, readFile, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import test from "node:test";

import { buildPerformanceLogBundle } from "./package-performance-log.mjs";

function browserSession() {
  return {
    schema_version: 2,
    kind: "physics-engine-browser-session",
    duration_ms: 1000,
    environment: {
      build: { revision: "test-revision", dirty: false },
      browser: "test-browser",
    },
    scenario: { projectile_policy: "physical" },
    summary: {
      recorded_frames: 1,
      omitted_frames: 0,
      omitted_markers: 0,
      physics_step_count: 1,
      physics_work: { sampled_events: 0, broad_phase_queries: 1 },
      frames: { mean_ms: 16, p95_ms: 16, max_ms: 16 },
      callbacks: { p95_ms: 1 },
      rendering: { p95_ms: 1 },
      dropped_accumulator_ms: 0,
      projectile_lifecycle: { retired_on_contact: 0 },
    },
  };
}

test("bundle records canonical evidence, profiles and raw browser evidence", async () => {
  const temporary = await mkdtemp(join(tmpdir(), "physics-performance-log-"));
  const evidence = join(temporary, "evidence");
  const session = join(temporary, "session.json");
  const archive = join(temporary, "bundle.tar.gz");
  await writeFile(session, JSON.stringify(browserSession()));
  const { manifest, provenance } = await buildPerformanceLogBundle({
    evidenceDirectory: evidence,
    archivePath: archive,
    sessionPaths: [session],
  });

  assert.equal(manifest.schema_version, 2);
  assert.equal(
    manifest.performance_evidence_contract.revision,
    "068d880a1b76f41e06548c557af0a7bf8c8057eb",
  );
  assert.deepEqual(manifest.canonical_evidence, ["canonical/browser/session.evidence.json"]);
  assert.deepEqual(manifest.measurement_profiles, ["profiles/physics-engine-sandbox-v1.json"]);
  assert.ok(manifest.files.some((entry) => entry.kind === "interactive session"));
  assert.ok(manifest.files.some((entry) => entry.kind === "measurement profile"));
  assert.ok(manifest.files.some((entry) => entry.kind === "canonical performance evidence"));
  assert.equal(provenance.performance_evidence_contract.schema_version, "1.0.0");

  const checksums = await readFile(join(evidence, "SHA256SUMS"), "utf8");
  assert.match(checksums, /sessions\/session\.json/);
  assert.match(checksums, /canonical\/browser\/session\.evidence\.json/);
  assert.match(checksums, /profiles\/physics-engine-sandbox-v1\.json/);
  assert.ok((await readFile(archive)).length > 0);
});

test("bundle rejects obsolete browser-session schemas", async () => {
  const temporary = await mkdtemp(join(tmpdir(), "physics-performance-log-"));
  const session = join(temporary, "session.json");
  await writeFile(session, JSON.stringify({ schema_version: 1, kind: "physics-engine-browser-session" }));

  await assert.rejects(
    buildPerformanceLogBundle({ evidenceDirectory: join(temporary, "evidence"), sessionPaths: [session] }),
    /not a current physics-engine browser performance session/,
  );
});
