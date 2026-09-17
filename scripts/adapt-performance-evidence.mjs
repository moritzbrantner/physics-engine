import { createHash } from "node:crypto";
import { mkdir, readFile, readdir, rm, stat, writeFile } from "node:fs/promises";
import { basename, join, relative, resolve } from "node:path";

const EVIDENCE_SCHEMA_VERSION = "1.0.0";
const REPOSITORY = "https://github.com/moritzbrantner/physics-engine";
const COLLECTOR_VERSION = "1.0.0";

function sha256Bytes(value) {
  return `sha256:${createHash("sha256").update(value).digest("hex")}`;
}

function stableJson(value) {
  if (Array.isArray(value)) return `[${value.map(stableJson).join(",")}]`;
  if (value && typeof value === "object") {
    return `{${Object.keys(value).sort().map((key) => `${JSON.stringify(key)}:${stableJson(value[key])}`).join(",")}}`;
  }
  return JSON.stringify(value);
}

function workloadHash(value) {
  return sha256Bytes(Buffer.from(stableJson(value)));
}

function measurement(name, value, unit, measurementType, description) {
  if (!Number.isFinite(value) || value < 0) return null;
  return { name, value, unit, measurement_type: measurementType, description };
}

const WORK_UNITS = new Map([
  ["sampled_events", "event"],
  ["tail_contacts", "contact"],
  ["tail_slices", "slice"],
  ["tail_replays", "replay"],
  ["tail_candidate_pairs", "pair"],
  ["tail_broad_phase_queries", "query"],
  ["tail_broad_phase_rebuilds", "rebuild"],
  ["tail_broad_phase_reuses", "reuse"],
  ["broad_phase_queries", "query"],
  ["broad_phase_rebuilds", "rebuild"],
  ["broad_phase_reuses", "reuse"],
  ["broad_phase_incremental_updates", "update"],
  ["broad_phase_reinserts", "reinsert"],
  ["broad_phase_rotations", "rotation"],
  ["broad_phase_partial_queries", "query"],
  ["broad_phase_partial_body_updates", "body"],
  ["event_response_passes", "pass"],
  ["stabilization_passes", "pass"],
  ["stabilizations_hitting_limit", "occurrence"],
  ["stabilization_candidate_pairs", "pair"],
  ["stabilization_exact_contacts", "contact"],
  ["stabilization_active_bodies", "body"],
]);

function inducedMeasurements(work = {}) {
  return Object.entries(work)
    .filter(([, value]) => Number.isFinite(value) && value >= 0)
    .sort(([left], [right]) => left.localeCompare(right))
    .map(([name, value]) => measurement(
      `physics.${name}`,
      value,
      WORK_UNITS.get(name) ?? "count",
      "counter",
      `Physics implementation work recorded by ${name}.`,
    ));
}

async function readJson(path) {
  return JSON.parse(await readFile(path, "utf8"));
}

async function fileExists(path) {
  try {
    return (await stat(path)).isFile();
  } catch {
    return false;
  }
}

async function artifact(root, path, kind, mediaType = "application/json") {
  const contents = await readFile(path);
  return {
    kind,
    path: relative(root, path).replaceAll("\\", "/"),
    sha256: sha256Bytes(contents),
    media_type: mediaType,
  };
}

function environmentEvidence(environment, collectorName) {
  const normalized = environment ?? {};
  const platform = {};
  for (const key of ["platform", "arch", "cpu", "browser"]) {
    if (typeof normalized[key] === "string") platform[key] = normalized[key];
  }
  const toolchain = {};
  for (const key of ["node", "v8", "rustc", "cargo"]) {
    if (typeof normalized[key] === "string") toolchain[key] = normalized[key];
  }
  const evidence = {
    fingerprint: workloadHash(normalized),
    collector: { name: collectorName, version: COLLECTOR_VERSION },
  };
  if (Object.keys(platform).length) evidence.platform = platform;
  if (Object.keys(toolchain).length) evidence.toolchain = toolchain;
  return evidence;
}

function sourceRevisionForSession(session, fallback) {
  const build = session.environment?.build;
  for (const candidate of [build?.revision, build?.sha, session.environment?.revision, fallback]) {
    if (typeof candidate === "string" && candidate.length > 0) return candidate;
  }
  return "unknown";
}

async function writeEvidence(path, evidence) {
  const contents = `${JSON.stringify(evidence, null, 2)}\n`;
  await mkdir(resolve(path, ".."), { recursive: true });
  await writeFile(path, contents);
  return sha256Bytes(Buffer.from(contents));
}

function sandboxEvidence({ result, side, caseEntry, trial, trialIndex, dimensions, environment, artifacts, sourceDirty, baseline }) {
  const revision = side === "baseline"
    ? (result.baseline_revision ?? "unknown")
    : (result.head_revision ?? process.env.HEAD_SHA ?? "unknown");
  const usefulWork = [
    measurement(
      "physics.simulation_steps",
      trial.steps?.count ?? 0,
      "step",
      "counter",
      "Simulation steps completed for the deterministic workload.",
    ),
    measurement(
      "physics.collision_events",
      trial.event_sum ?? 0,
      "event",
      "counter",
      "Collision events produced by the workload.",
    ),
  ].filter(Boolean);
  const outcomes = [
    measurement("physics.step.mean_ms", trial.steps?.mean_ms, "ms", "duration", "Mean physics-step duration."),
    measurement("physics.step.p50_ms", trial.steps?.p50_ms, "ms", "duration", "Median physics-step duration."),
    measurement("physics.step.p95_ms", trial.steps?.p95_ms, "ms", "duration", "95th-percentile physics-step duration."),
    measurement("physics.step.max_ms", trial.steps?.max_ms, "ms", "duration", "Maximum physics-step duration."),
    measurement("physics.body_count", trial.body_count, "body", "gauge", "Bodies present after the deterministic workload."),
  ].filter(Boolean);
  const evidence = {
    schema_version: EVIDENCE_SCHEMA_VERSION,
    scenario: {
      id: "physics-engine/sandbox-projectiles-v4",
      description: `Deterministic physics-engine sandbox case ${caseEntry.name}.`,
      workload: {
        id: result.workload ?? "sandbox-projectiles-v4",
        hash: workloadHash({
          workload: result.workload ?? "sandbox-projectiles-v4",
          case: caseEntry.name,
          trial: trialIndex + 1,
          dimensions,
          benchmark: artifacts.find((entry) => entry.kind === "benchmark-source")?.sha256 ?? null,
        }),
        parameters: { case: caseEntry.name, trial: trialIndex + 1, ...dimensions },
      },
    },
    source: { repository: REPOSITORY, revision, dirty: side === "baseline" ? false : sourceDirty },
    environment,
    measurements: {
      useful_work: usefulWork,
      induced_work: inducedMeasurements(trial.work),
      outcomes,
    },
    artifacts,
    extensions: {
      "physics-engine.replay": {
        side,
        case: caseEntry.name,
        trial: trialIndex + 1,
        replay_sha256: trial.replay_sha256 ?? null,
        workload_note: result.note ?? null,
      },
    },
  };
  if (baseline) evidence.baseline = baseline;
  return evidence;
}

async function adaptSandbox(root, canonicalRoot, sourceDirty) {
  const sandboxPath = join(root, "sandbox.json");
  if (!(await fileExists(sandboxPath))) return [];
  const result = await readJson(sandboxPath);
  if (!result.head?.cases) return [];

  const contractPath = join(root, "performance-contract.json");
  const benchmarkPath = join(root, "benchmark-sandbox.mjs");
  const contract = (await fileExists(contractPath)) ? await readJson(contractPath) : null;
  const artifacts = [await artifact(root, sandboxPath, "raw-benchmark")];
  if (await fileExists(benchmarkPath)) artifacts.push(await artifact(root, benchmarkPath, "benchmark-source", "text/javascript"));
  if (await fileExists(contractPath)) artifacts.push(await artifact(root, contractPath, "physics-performance-policy"));
  const environment = environmentEvidence(result.environment, "physics-engine.sandbox-adapter");
  const written = [];
  const baselineHashes = new Map();

  if (result.baseline?.cases) {
    for (const caseEntry of result.baseline.cases) {
      const dimensions = contract?.scenarios?.find((entry) => entry.id === caseEntry.name)?.dimensions ?? {};
      for (const [trialIndex, trial] of caseEntry.trials.entries()) {
        const evidence = sandboxEvidence({ result, side: "baseline", caseEntry, trial, trialIndex, dimensions, environment, artifacts, sourceDirty });
        const path = join(canonicalRoot, "sandbox", caseEntry.name, `baseline-trial-${trialIndex + 1}.json`);
        const hash = await writeEvidence(path, evidence);
        baselineHashes.set(`${caseEntry.name}:${trialIndex}`, { source_revision: evidence.source.revision, evidence_hash: hash });
        written.push(path);
      }
    }
  }

  for (const caseEntry of result.head.cases) {
    const dimensions = contract?.scenarios?.find((entry) => entry.id === caseEntry.name)?.dimensions ?? {};
    for (const [trialIndex, trial] of caseEntry.trials.entries()) {
      const evidence = sandboxEvidence({
        result,
        side: "head",
        caseEntry,
        trial,
        trialIndex,
        dimensions,
        environment,
        artifacts,
        sourceDirty,
        baseline: baselineHashes.get(`${caseEntry.name}:${trialIndex}`),
      });
      const path = join(canonicalRoot, "sandbox", caseEntry.name, `head-trial-${trialIndex + 1}.json`);
      await writeEvidence(path, evidence);
      written.push(path);
    }
  }
  return written;
}

function sessionEvidence({ session, sessionPath, sourceDirty, fallbackRevision, artifactEntry }) {
  const summary = session.summary ?? {};
  const usefulWork = [
    measurement("physics.simulation_steps", summary.physics_step_count, "step", "counter", "Physics steps completed during the browser session."),
    measurement("physics.projectiles_retired_on_contact", summary.projectile_lifecycle?.retired_on_contact, "projectile", "counter", "Projectiles retired by contact policy."),
  ].filter(Boolean);
  const outcomes = [
    measurement("browser.session_duration_ms", session.duration_ms, "ms", "duration", "Browser performance-session duration."),
    measurement("browser.frame.mean_ms", summary.frames?.mean_ms, "ms", "duration", "Mean browser frame interval."),
    measurement("browser.frame.p95_ms", summary.frames?.p95_ms, "ms", "duration", "95th-percentile browser frame interval."),
    measurement("browser.frame.max_ms", summary.frames?.max_ms, "ms", "duration", "Maximum browser frame interval."),
    measurement("browser.callback.p95_ms", summary.callbacks?.p95_ms, "ms", "duration", "95th-percentile animation callback duration."),
    measurement("browser.render.p95_ms", summary.rendering?.p95_ms, "ms", "duration", "95th-percentile render duration."),
    measurement("browser.dropped_accumulator_ms", summary.dropped_accumulator_ms, "ms", "duration", "Accumulated simulation time dropped by the fixed-step cap."),
  ].filter(Boolean);
  return {
    schema_version: EVIDENCE_SCHEMA_VERSION,
    scenario: {
      id: "physics-engine/browser-session-v2",
      description: "Bounded interactive browser performance session for the physics sandbox.",
      workload: {
        id: "browser-interactive-v2",
        hash: workloadHash({ scenario: session.scenario ?? {}, schema_version: session.schema_version }),
        parameters: session.scenario ?? {},
      },
    },
    source: {
      repository: REPOSITORY,
      revision: sourceRevisionForSession(session, fallbackRevision),
      dirty: typeof session.environment?.build?.dirty === "boolean" ? session.environment.build.dirty : sourceDirty,
    },
    environment: environmentEvidence(session.environment, "physics-engine.browser-session-adapter"),
    measurements: {
      useful_work: usefulWork,
      induced_work: inducedMeasurements(summary.physics_work),
      outcomes,
    },
    artifacts: [artifactEntry],
    extensions: {
      "physics-engine.session": {
        started_at: session.started_at ?? null,
        ended_at: session.ended_at ?? null,
        recorded_frames: summary.recorded_frames ?? null,
        omitted_frames: summary.omitted_frames ?? null,
        omitted_markers: summary.omitted_markers ?? null,
        note: session.note ?? null,
        source_file: basename(sessionPath),
      },
    },
  };
}

async function adaptSessions(root, canonicalRoot, sourceDirty) {
  const sessionsRoot = join(root, "sessions");
  let entries;
  try {
    entries = await readdir(sessionsRoot, { withFileTypes: true });
  } catch {
    return [];
  }
  const written = [];
  for (const entry of entries.filter((value) => value.isFile() && value.name.endsWith(".json")).sort((a, b) => a.name.localeCompare(b.name))) {
    const path = join(sessionsRoot, entry.name);
    const session = await readJson(path);
    if (session.kind !== "physics-engine-browser-session" || session.schema_version !== 2) continue;
    const artifactEntry = await artifact(root, path, "interactive-session");
    const evidence = sessionEvidence({ session, sessionPath: path, sourceDirty, fallbackRevision: process.env.HEAD_SHA, artifactEntry });
    const target = join(canonicalRoot, "browser", `${entry.name.replace(/\.json$/, "")}.evidence.json`);
    await writeEvidence(target, evidence);
    written.push(target);
  }
  return written;
}

export async function buildCanonicalPerformanceEvidence({ evidenceDirectory, sourceDirty = false } = {}) {
  const root = resolve(evidenceDirectory);
  const canonicalRoot = join(root, "canonical");
  await rm(canonicalRoot, { recursive: true, force: true });
  await mkdir(canonicalRoot, { recursive: true });
  const written = [
    ...(await adaptSandbox(root, canonicalRoot, sourceDirty)),
    ...(await adaptSessions(root, canonicalRoot, sourceDirty)),
  ];
  return written.map((path) => relative(root, path).replaceAll("\\", "/"));
}
