const SCHEMA_VERSION = 1;
const DEFAULT_MAX_FRAMES = 36_000;
const DEFAULT_MAX_MARKERS = 10_000;

function snapshot(value) {
  return JSON.parse(JSON.stringify(typeof value === "function" ? value() : (value ?? {})));
}

function finiteNumber(value, name) {
  if (!Number.isFinite(value) || value < 0) throw new Error(`${name} must be finite and non-negative`);
  return value;
}

function statistics(values) {
  if (values.length === 0) return { count: 0, mean_ms: null, p50_ms: null, p95_ms: null, max_ms: null };
  const sorted = [...values].sort((left, right) => left - right);
  const percentile = (fraction) => sorted[Math.max(0, Math.ceil(sorted.length * fraction) - 1)];
  return {
    count: values.length,
    mean_ms: values.reduce((sum, value) => sum + value, 0) / values.length,
    p50_ms: percentile(0.5),
    p95_ms: percentile(0.95),
    max_ms: sorted.at(-1),
  };
}

export function createPerformanceSessionRecorder({
  environment,
  scenario,
  now = () => performance.now(),
  wallClock = () => new Date().toISOString(),
  maxFrames = DEFAULT_MAX_FRAMES,
  maxMarkers = DEFAULT_MAX_MARKERS,
} = {}) {
  if (!Number.isInteger(maxFrames) || maxFrames < 1) throw new Error("maxFrames must be a positive integer");
  if (!Number.isInteger(maxMarkers) || maxMarkers < 1) throw new Error("maxMarkers must be a positive integer");

  let active = false;
  let startedAt = null;
  let startedAtMonotonic = null;
  let frames = [];
  let markers = [];
  let omittedFrames = 0;
  let omittedMarkers = 0;
  let recordedEnvironment = {};
  let recordedScenario = {};

  return {
    get active() {
      return active;
    },

    start() {
      if (active) throw new Error("performance session is already recording");
      active = true;
      startedAt = wallClock();
      startedAtMonotonic = now();
      frames = [];
      markers = [];
      omittedFrames = 0;
      omittedMarkers = 0;
      recordedEnvironment = snapshot(environment);
      recordedScenario = snapshot(scenario);
    },

    recordFrame(frame) {
      if (!active) return;
      const normalized = {
        frame_interval_ms: finiteNumber(frame.frame_interval_ms, "frame_interval_ms"),
        callback_ms: finiteNumber(frame.callback_ms, "callback_ms"),
        render_ms: finiteNumber(frame.render_ms, "render_ms"),
        physics_steps_ms: (frame.physics_steps_ms ?? []).map((value) =>
          finiteNumber(value, "physics_steps_ms"),
        ),
        dropped_accumulator_ms: finiteNumber(
          frame.dropped_accumulator_ms ?? 0,
          "dropped_accumulator_ms",
        ),
        body_count: Number.isInteger(frame.body_count) ? frame.body_count : null,
        collision_contacts: Number.isInteger(frame.collision_contacts)
          ? frame.collision_contacts
          : null,
        paused: Boolean(frame.paused),
      };
      if (frames.length < maxFrames) frames.push(normalized);
      else omittedFrames += 1;
    },

    recordMarker(name, detail = {}) {
      if (!active) return;
      if (markers.length < maxMarkers) {
        markers.push({ offset_ms: now() - startedAtMonotonic, name, detail: snapshot(detail) });
      } else {
        omittedMarkers += 1;
      }
    },

    finish() {
      if (!active) throw new Error("performance session is not recording");
      const endedAt = wallClock();
      const durationMs = now() - startedAtMonotonic;
      active = false;
      const frameIntervals = frames.map((frame) => frame.frame_interval_ms);
      const callbacks = frames.map((frame) => frame.callback_ms);
      const renders = frames.map((frame) => frame.render_ms);
      const physicsSteps = frames.flatMap((frame) => frame.physics_steps_ms);
      return {
        schema_version: SCHEMA_VERSION,
        kind: "physics-engine-browser-session",
        note:
          "Interactive browser timings include scheduling and rendering noise. They are diagnostic evidence, not a deterministic benchmark or a pass/fail performance gate.",
        started_at: startedAt,
        ended_at: endedAt,
        duration_ms: durationMs,
        environment: recordedEnvironment,
        scenario: recordedScenario,
        summary: {
          recorded_frames: frames.length,
          omitted_frames: omittedFrames,
          omitted_markers: omittedMarkers,
          frames: statistics(frameIntervals),
          callbacks: statistics(callbacks),
          rendering: statistics(renders),
          physics_steps: statistics(physicsSteps),
          physics_step_count: physicsSteps.length,
          frames_with_dropped_accumulator: frames.filter(
            (frame) => frame.dropped_accumulator_ms > 0,
          ).length,
          dropped_accumulator_ms: frames.reduce(
            (sum, frame) => sum + frame.dropped_accumulator_ms,
            0,
          ),
        },
        raw: { frames, markers },
      };
    },
  };
}

export function serializePerformanceSession(session) {
  return `${JSON.stringify(session, null, 2)}\n`;
}
