import assert from "node:assert/strict";
import test from "node:test";

import { createPerformanceSessionRecorder, serializePerformanceSession } from "./performance-log.mjs";

test("session output keeps raw timings and derives stable summaries", () => {
  let monotonic = 100;
  const timestamps = ["2026-09-15T10:00:00.000Z", "2026-09-15T10:00:01.000Z"];
  const recorder = createPerformanceSessionRecorder({
    environment: { renderer: "webgl2" },
    scenario: { character: "linear" },
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
    body_count: 12,
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
  assert.equal(result.raw.frames[0].body_count, 12);
  assert.equal(result.summary.rendered_frames, 1);
  assert.equal(JSON.parse(serializePerformanceSession(result)).schema_version, 1);
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
