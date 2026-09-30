import assert from "node:assert/strict";
import test from "node:test";
import { createScenarioLog } from "./scenario-log.mjs";

import { createPerformanceSessionRecorder, serializePerformanceSession } from "./performance-log.mjs";

test("session output keeps raw timings and derives stable summaries", () => {
  let monotonic = 100;
  const timestamps = ["2026-09-15T10:00:00.000Z", "2026-09-15T10:00:01.000Z"];
  const recorder = createPerformanceSessionRecorder({
    environment: { renderer: "webgl2" },
    scenario: { character: "linear", projectile_impact: "impact-retire" },
    now: () => monotonic,
    wallClock: () => timestamps.shift(),
  });

  recorder.start();
  recorder.recordFrame({
    frame_interval_ms: 16,
    callback_ms: 5,
    render_performed: true,
    render_ms: 3,
    physics_steps_ms: [1, 2],
    physics_step_stats: [
      {
        sampled_events: 1,
        tail_contacts: 2,
        tail_slices: 3,
        tail_replays: 0,
        tail_candidate_pairs: 4,
        tail_broad_phase_queries: 5,
        tail_broad_phase_rebuilds: 1,
        tail_broad_phase_reuses: 4,
        broad_phase_queries: 6,
        broad_phase_rebuilds: 1,
        broad_phase_reuses: 5,
      },
      {
        sampled_events: 2,
        tail_contacts: 3,
        tail_slices: 4,
        tail_replays: 1,
        tail_candidate_pairs: 5,
        tail_broad_phase_queries: 6,
        tail_broad_phase_rebuilds: 0,
        tail_broad_phase_reuses: 6,
        broad_phase_queries: 7,
        broad_phase_rebuilds: 0,
        broad_phase_reuses: 7,
      },
    ],
    body_count: 12,
    projectile_count: 4,
    active_projectile_count: 3,
    projectiles_retired_on_contact: 2,
    projectiles_retired_out_of_bounds: 1,
    projectiles_evicted_by_cap: 0,
    collision_contacts: 2,
    paused: false,
  });
  monotonic = 1_100;
  const result = recorder.finish();

  assert.equal(result.duration_ms, 1_000);
  assert.deepEqual(result.summary.physics_steps, {
    count: 2,
    mean_ms: 1.5,
    p50_ms: 1,
    p95_ms: 2,
    max_ms: 2,
  });
  assert.deepEqual(result.summary.physics_work, {
    fixed_position_passes: 0,
    fixed_position_bounds_tests: 0,
    fixed_position_contact_tests: 0,
    fixed_position_corrections: 0,

    fixed_substeps: 0, fixed_pair_tests: 0, fixed_narrow_tests: 0, fixed_contact_points: 0,
    fixed_impulse_iterations: 0, fixed_integrated_bodies: 0, fixed_woken_bodies: 0, fixed_swept_contacts: 0,
    sampled_events: 3,
    tail_contacts: 5,
    tail_slices: 7,
    tail_replays: 1,
    tail_candidate_pairs: 9,
    tail_broad_phase_queries: 11,
    tail_broad_phase_rebuilds: 1,
    tail_broad_phase_reuses: 10,
    broad_phase_queries: 13,
    broad_phase_rebuilds: 1,
    broad_phase_reuses: 12,
    broad_phase_incremental_updates: 0,
    broad_phase_reinserts: 0,
    broad_phase_rotations: 0,
    broad_phase_partial_queries: 0,
    broad_phase_partial_body_updates: 0,
    response_scratch_index_rebuilds: 0,
    event_response_passes: 0,
    stabilization_passes: 0,
    stabilizations_hitting_limit: 0,
    stabilization_candidate_pairs: 0,
    stabilization_exact_contacts: 0,
    stabilization_active_bodies: 0,
    ballistic_query_rounds: 0,
    ballistic_target_bound_checks: 0,
    ballistic_broad_phase_candidates: 0,
    ballistic_toi_tests: 0,
    ballistic_feature_tests: 0,
    ballistic_motion_samples: 0,
    ballistic_impacts: 0,
    ballistic_retired: 0,
  });
  assert.deepEqual(result.summary.projectile_lifecycle, {
    max_live_projectiles: 4,
    max_active_projectiles: 3,
    retired_on_contact: 2,
    retired_out_of_bounds: 1,
    evicted_by_cap: 0,
  });
  assert.equal(result.raw.frames[0].physics_step_stats.length, 2);
  assert.equal(result.raw.frames[0].body_count, 12);
  assert.equal(result.raw.frames[0].projectile_count, 4);
  assert.equal(result.raw.frames[0].active_projectile_count, 3);
  assert.equal(result.summary.rendered_frames, 1);
  assert.equal(JSON.parse(serializePerformanceSession(result)).schema_version, 2);
});

test("projectile lifecycle totals remain additive across a sandbox reset", () => {
  const recorder = createPerformanceSessionRecorder();
  recorder.start();
  const frame = (retired) => ({
    frame_interval_ms: 16,
    callback_ms: 1,
    render_performed: false,
    projectile_count: 1,
    projectiles_retired_on_contact: retired,
    projectiles_retired_out_of_bounds: 0,
    projectiles_evicted_by_cap: 0,
  });
  recorder.recordFrame(frame(2));
  recorder.recordFrame(frame(3));
  recorder.recordFrame(frame(0));
  recorder.recordFrame(frame(2));
  assert.equal(recorder.finish().summary.projectile_lifecycle.retired_on_contact, 5);
});

test("session storage is bounded and reports omitted frames", () => {
  const recorder = createPerformanceSessionRecorder({ maxFrames: 1, maxMarkers: 1 });
  recorder.start();
  const frame = {
    frame_interval_ms: 16,
    callback_ms: 1,
    render_performed: false,
    render_ms: null,
  };
  recorder.recordFrame(frame);
  recorder.recordFrame(frame);
  recorder.recordMarker("first");
  recorder.recordMarker("second");
  const result = recorder.finish();

  assert.equal(result.summary.recorded_frames, 1);
  assert.equal(result.summary.omitted_frames, 1);
  assert.equal(result.summary.omitted_markers, 1);
  assert.equal(result.summary.rendering.count, 0);
  assert.equal(result.summary.no_op_render_frames, 1);
});

test("environment and scenario identify the start of the recorded interval", () => {
  const scenario = { mode: "before" };
  const recorder = createPerformanceSessionRecorder({ scenario: () => scenario });
  recorder.start();
  scenario.mode = "after";

  assert.equal(recorder.finish().scenario.mode, "before");
});

test("physics step diagnostics must align with timed physics steps", () => {
  const recorder = createPerformanceSessionRecorder();
  recorder.start();
  assert.throws(
    () => recorder.recordFrame({
      frame_interval_ms: 16,
      callback_ms: 2,
      render_performed: false,
      physics_steps_ms: [1, 1],
      physics_step_stats: [{ tail_slices: 1 }],
    }),
    /align one-to-one/,
  );
});

test("invalid timing evidence fails instead of being silently normalized", () => {
  const recorder = createPerformanceSessionRecorder();
  recorder.start();
  assert.throws(
    () => recorder.recordFrame({
      frame_interval_ms: Number.NaN,
      callback_ms: 1,
      render_performed: true,
      render_ms: 1,
    }),
    /frame_interval_ms/,
  );
});
test('fixed-step telemetry is retained without inventing legacy event work', () => {
  const recorder = createPerformanceSessionRecorder({now:()=>0, wallClock:()=>new Date(0)});
  recorder.start();
  recorder.recordFrame({frame_interval_ms:16,callback_ms:1,render_performed:false,render_ms:null,physics_steps_ms:[1],physics_step_stats:[{fixed_substeps:8,fixed_impulse_iterations:36,fixed_contact_points:14}],dropped_accumulator_ms:0});
  const result=recorder.finish();
  assert.equal(result.summary.physics_work.fixed_substeps,8);
  assert.equal(result.summary.physics_work.fixed_impulse_iterations,36);
  assert.equal(result.raw.frames[0].physics_step_stats[0].sampled_events,null);
});

test("rolling recordings retain the crash lead-up in chronological order", () => {
  let time = 0;
  const recorder = createPerformanceSessionRecorder({
    maxFrames: 3, maxMarkers: 2, retention: "latest", now: () => time,
  });
  recorder.start();
  for (let index = 0; index < 8; index += 1) {
    time = index * 16;
    recorder.recordFrame({
      frame_interval_ms: 16, callback_ms: index, render_performed: false,
      projectiles_retired_on_contact: index,
    });
    recorder.recordMarker("input", { index });
  }
  const result = recorder.finish();
  assert.deepEqual(result.raw.frames.map(frame => frame.callback_ms), [5, 6, 7]);
  assert.deepEqual(result.raw.frames.map(frame => frame.offset_ms), [80, 96, 112]);
  assert.deepEqual(result.raw.markers.map(marker => marker.detail.index), [6, 7]);
  assert.equal(result.summary.omitted_frames, 5);
  assert.equal(result.summary.omitted_markers, 6);
  assert.equal(result.summary.projectile_lifecycle.retired_on_contact, 3);
  assert.equal(result.retention.summary_scope, "retained-frames");
});

test("capture does not stop or mutate the automatically running recording", () => {
  const recorder = createPerformanceSessionRecorder({ maxFrames: 1, retention: "latest" });
  const frame = value => ({ frame_interval_ms: 16, callback_ms: value, render_performed: false });
  recorder.start();
  recorder.recordFrame(frame(1));
  const captured = recorder.capture();
  assert.equal(recorder.active, true);
  recorder.recordFrame(frame(2));
  assert.equal(captured.raw.frames[0].callback_ms, 1);
  assert.equal(recorder.finish().raw.frames[0].callback_ms, 2);
});

test("restarting discards healthy-run history and snapshots the new context", () => {
  const scenario = { id: "first" };
  const recorder = createPerformanceSessionRecorder({ scenario: () => scenario });
  recorder.start();
  recorder.recordMarker("old-run");
  scenario.id = "second";
  recorder.start({ restart: true });
  const result = recorder.finish();
  assert.equal(result.scenario.id, "second");
  assert.equal(result.raw.markers.length, 0);
});

test("rolling cumulative counters keep the last known evicted baseline across unavailable samples", () => {
  const recorder = createPerformanceSessionRecorder({ maxFrames: 1, retention: "latest" });
  recorder.start();
  for (const count of [10, null, 11]) {
    recorder.recordFrame({ frame_interval_ms: 16, callback_ms: 1, render_performed: false,
      projectiles_retired_on_contact: count });
  }
  assert.equal(recorder.finish().summary.projectile_lifecycle.retired_on_contact, 1);
});

const frame = (value = 1) => ({ frame_interval_ms: 16, callback_ms: value, render_performed: false });

test("scenario creation records without opting in and freezes the first failure", () => {
  let time = 0;
  const reports = [];
  const log = createScenarioLog({ now: () => time, scenario: { id: "tower" }, onCrash: report => reports.push(report) });
  log.recorder.recordFrame(frame());
  log.begin("physics-step", { step: 12, velocity: [4, 0], jump: 0 });
  time = 9;
  const error = new Error("engine failed");
  log.fail(error, { engine_code: 6, engine_detail: 611 });
  log.fail(new Error("secondary failure"));
  log.recorder.recordFrame(frame(99));
  assert.equal(log.failed, true);
  assert.equal(log.recorder.active, false);
  assert.equal(reports.length, 1);
  assert.equal(reports[0].failure.message, "engine failed");
  assert.equal(reports[0].failure.stack, error.stack);
  assert.equal(reports[0].failure.engine_detail, 611);
  assert.equal(reports[0].failure.operation.name, "physics-step");
  assert.equal(reports[0].failure.operation.duration_ms, 9);
  assert.equal(reports[0].summary.recorded_frames, 1);
  assert.equal(reports[0].outcome, "crashed");
});

test("initialization failure produces a report before any frames exist", () => {
  const log = createScenarioLog({ scenario: { id: "parkour" } });
  log.begin("initialization");
  log.fail(new WebAssembly.RuntimeError("unreachable"));
  assert.equal(log.lastCrash.scenario.id, "parkour");
  assert.equal(log.lastCrash.summary.recorded_frames, 0);
  assert.equal(log.lastCrash.failure.name, "RuntimeError");
});

test("reset isolates runs but preserves the previous crash report", () => {
  const scenario = { id: "first" };
  const log = createScenarioLog({ scenario: () => scenario });
  log.fail(new Error("first failure"));
  const first = log.lastCrash;
  scenario.id = "second";
  log.start();
  assert.equal(log.failed, false);
  assert.equal(log.lastCrash, first);
  log.recorder.recordFrame(frame());
  const capture = log.recorder.capture();
  assert.equal(log.recorder.active, true);
  assert.equal(capture.scenario.id, "second");
  assert.equal(capture.raw.markers.some(marker => marker.name === "crash"), false);
  log.fail("second failure");
  assert.equal(first.failure.message, "first failure");
  assert.equal(log.lastCrash.failure.message, "second failure");
});

test("recording and crash reporting use captured data, not a broken engine", () => {
  let unavailable = false;
  const log = createScenarioLog({
    environment: () => { if (unavailable) throw new Error("engine cannot be read"); return { build: "test-build" }; },
  });
  log.recorder.recordFrame(frame());
  log.begin("physics-step", { step: 1 });
  unavailable = true;
  log.fail(new Error("WASM trapped"));
  assert.equal(log.lastCrash.environment.build, "test-build");
  assert.equal(log.lastCrash.failure.message, "WASM trapped");
});

test("reporter and stop-handler failures cannot replace the original crash", () => {
  const diagnostics = [];
  const log = createScenarioLog({
    onCrash: () => { throw new Error("download blocked"); },
    onDiagnosticError: error => diagnostics.push(error.message),
  });
  log.configure({ onFailure: () => { throw new Error("UI unavailable"); } });
  assert.doesNotThrow(() => log.fail(new Error("original failure")));
  assert.equal(log.lastCrash.failure.message, "original failure");
  assert.deepEqual(diagnostics, ["UI unavailable", "download blocked"]);
});

test("operation inputs are recorded by value and completion clears in-flight attribution", () => {
  const log = createScenarioLog();
  const input = { velocity: [1, 2, 3] };
  log.begin("shoot", input);
  input.velocity[0] = 100;
  log.complete();
  log.fail(new Error("outside operation"));
  assert.equal(log.lastCrash.failure.operation, null);
  assert.deepEqual(log.lastCrash.raw.markers.find(marker => marker.name === "shoot").detail.velocity, [1, 2, 3]);
});
