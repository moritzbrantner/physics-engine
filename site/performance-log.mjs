const SCHEMA_VERSION = 2;
const DEFAULT_MAX_FRAMES = 36_000;
const DEFAULT_MAX_MARKERS = 10_000;

function snapshot(value) {
  return JSON.parse(JSON.stringify(typeof value === "function" ? value() : (value ?? {})));
}

function finiteNumber(value, name) {
  if (!Number.isFinite(value) || value < 0) throw new Error(`${name} must be finite and non-negative`);
  return value;
}

function optionalCounter(value, name) {
  if (value === null || value === undefined) return null;
  if (!Number.isInteger(value) || value < 0) throw new Error(`${name} must be a non-negative integer`);
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

function sumCounter(steps, name) {
  return steps.reduce((sum, step) => sum + (step[name] ?? 0), 0);
}

function cumulativeCounterTotal(frames, name) {
  let total = 0;
  let previous = 0;
  for (const frame of frames) {
    const current = frame[name];
    if (current === null || current === undefined) continue;
    total += current >= previous ? current - previous : current;
    previous = current;
  }
  return total;
}

function normalizeStepStats(step) {
  return {
    sampled_events: optionalCounter(step.sampled_events, "sampled_events"),
    tail_contacts: optionalCounter(step.tail_contacts, "tail_contacts"),
    tail_slices: optionalCounter(step.tail_slices, "tail_slices"),
    tail_replays: optionalCounter(step.tail_replays, "tail_replays"),
    tail_candidate_pairs: optionalCounter(step.tail_candidate_pairs, "tail_candidate_pairs"),
    tail_broad_phase_queries: optionalCounter(
      step.tail_broad_phase_queries,
      "tail_broad_phase_queries",
    ),
    tail_broad_phase_rebuilds: optionalCounter(
      step.tail_broad_phase_rebuilds,
      "tail_broad_phase_rebuilds",
    ),
    tail_broad_phase_reuses: optionalCounter(
      step.tail_broad_phase_reuses,
      "tail_broad_phase_reuses",
    ),
    broad_phase_queries: optionalCounter(step.broad_phase_queries, "broad_phase_queries"),
    broad_phase_rebuilds: optionalCounter(step.broad_phase_rebuilds, "broad_phase_rebuilds"),
    broad_phase_reuses: optionalCounter(step.broad_phase_reuses, "broad_phase_reuses"),
    broad_phase_incremental_updates: optionalCounter(step.broad_phase_incremental_updates, "broad_phase_incremental_updates"),
    broad_phase_reinserts: optionalCounter(step.broad_phase_reinserts, "broad_phase_reinserts"),
    broad_phase_rotations: optionalCounter(step.broad_phase_rotations, "broad_phase_rotations"),
    broad_phase_partial_queries: optionalCounter(step.broad_phase_partial_queries, "broad_phase_partial_queries"),
    broad_phase_partial_body_updates: optionalCounter(step.broad_phase_partial_body_updates, "broad_phase_partial_body_updates"),
    event_response_passes: optionalCounter(step.event_response_passes, "event_response_passes"),
    stabilization_passes: optionalCounter(step.stabilization_passes, "stabilization_passes"),
    stabilizations_hitting_limit: optionalCounter(step.stabilizations_hitting_limit, "stabilizations_hitting_limit"),
    stabilization_candidate_pairs: optionalCounter(step.stabilization_candidate_pairs, "stabilization_candidate_pairs"),
    stabilization_exact_contacts: optionalCounter(step.stabilization_exact_contacts, "stabilization_exact_contacts"),
    stabilization_active_bodies: optionalCounter(step.stabilization_active_bodies, "stabilization_active_bodies"),
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
        render_performed: Boolean(frame.render_performed),
        render_ms: frame.render_performed ? finiteNumber(frame.render_ms, "render_ms") : null,
        physics_steps_ms: (frame.physics_steps_ms ?? []).map((value) =>
          finiteNumber(value, "physics_steps_ms"),
        ),
        physics_step_stats: (frame.physics_step_stats ?? []).map(normalizeStepStats),
        dropped_accumulator_ms: finiteNumber(
          frame.dropped_accumulator_ms ?? 0,
          "dropped_accumulator_ms",
        ),
        body_count: optionalCounter(frame.body_count, "body_count"),
        projectile_count: optionalCounter(frame.projectile_count, "projectile_count"),
        active_projectile_count: optionalCounter(
          frame.active_projectile_count,
          "active_projectile_count",
        ),
        projectiles_retired_on_contact: optionalCounter(
          frame.projectiles_retired_on_contact,
          "projectiles_retired_on_contact",
        ),
        projectiles_retired_out_of_bounds: optionalCounter(
          frame.projectiles_retired_out_of_bounds,
          "projectiles_retired_out_of_bounds",
        ),
        projectiles_evicted_by_cap: optionalCounter(
          frame.projectiles_evicted_by_cap,
          "projectiles_evicted_by_cap",
        ),
        collision_contacts: optionalCounter(frame.collision_contacts, "collision_contacts"),
        paused: Boolean(frame.paused),
      };
      if (normalized.physics_step_stats.length !== 0 && normalized.physics_step_stats.length !== normalized.physics_steps_ms.length) {
        throw new Error("physics_step_stats must align one-to-one with physics_steps_ms");
      }
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
      const renders = frames
        .filter((frame) => frame.render_performed)
        .map((frame) => frame.render_ms);
      const physicsSteps = frames.flatMap((frame) => frame.physics_steps_ms);
      const physicsStepStats = frames.flatMap((frame) => frame.physics_step_stats);
      const liveProjectileCounts = frames
        .map((frame) => frame.projectile_count)
        .filter((value) => value !== null);
      const activeProjectileCounts = frames
        .map((frame) => frame.active_projectile_count)
        .filter((value) => value !== null);
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
          rendered_frames: renders.length,
          no_op_render_frames: frames.length - renders.length,
          physics_steps: statistics(physicsSteps),
          physics_step_count: physicsSteps.length,
          physics_work: {
            sampled_events: sumCounter(physicsStepStats, "sampled_events"),
            tail_contacts: sumCounter(physicsStepStats, "tail_contacts"),
            tail_slices: sumCounter(physicsStepStats, "tail_slices"),
            tail_replays: sumCounter(physicsStepStats, "tail_replays"),
            tail_candidate_pairs: sumCounter(physicsStepStats, "tail_candidate_pairs"),
            tail_broad_phase_queries: sumCounter(physicsStepStats, "tail_broad_phase_queries"),
            tail_broad_phase_rebuilds: sumCounter(physicsStepStats, "tail_broad_phase_rebuilds"),
            tail_broad_phase_reuses: sumCounter(physicsStepStats, "tail_broad_phase_reuses"),
            broad_phase_queries: sumCounter(physicsStepStats, "broad_phase_queries"),
            broad_phase_rebuilds: sumCounter(physicsStepStats, "broad_phase_rebuilds"),
            broad_phase_reuses: sumCounter(physicsStepStats, "broad_phase_reuses"),
            broad_phase_incremental_updates: sumCounter(physicsStepStats, "broad_phase_incremental_updates"),
            broad_phase_reinserts: sumCounter(physicsStepStats, "broad_phase_reinserts"),
            broad_phase_rotations: sumCounter(physicsStepStats, "broad_phase_rotations"),
            broad_phase_partial_queries: sumCounter(physicsStepStats, "broad_phase_partial_queries"),
            broad_phase_partial_body_updates: sumCounter(physicsStepStats, "broad_phase_partial_body_updates"),
            event_response_passes: sumCounter(physicsStepStats, "event_response_passes"),
            stabilization_passes: sumCounter(physicsStepStats, "stabilization_passes"),
            stabilizations_hitting_limit: sumCounter(physicsStepStats, "stabilizations_hitting_limit"),
            stabilization_candidate_pairs: sumCounter(physicsStepStats, "stabilization_candidate_pairs"),
            stabilization_exact_contacts: sumCounter(physicsStepStats, "stabilization_exact_contacts"),
            stabilization_active_bodies: sumCounter(physicsStepStats, "stabilization_active_bodies"),
          },
          projectile_lifecycle: {
            max_live_projectiles:
              liveProjectileCounts.length === 0 ? null : Math.max(...liveProjectileCounts),
            max_active_projectiles:
              activeProjectileCounts.length === 0 ? null : Math.max(...activeProjectileCounts),
            retired_on_contact: cumulativeCounterTotal(frames, "projectiles_retired_on_contact"),
            retired_out_of_bounds: cumulativeCounterTotal(
              frames,
              "projectiles_retired_out_of_bounds",
            ),
            evicted_by_cap: cumulativeCounterTotal(frames, "projectiles_evicted_by_cap"),
          },
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