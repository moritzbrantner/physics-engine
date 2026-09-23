// Presentation adapter only. The Rust entry point owns fixture selection, all physics and lifecycle.
// Deliberately expose no legacy stepping/counters: the fixture's unused event world is not telemetry.
const REQUIRED = [
  'approximate_reset_tower', 'approximate_step_velocity', 'approximate_shoot',
  'approximate_set_projectile_type', 'approximate_refresh_snapshot', 'approximate_snapshot_len',
  'approximate_body_sleeping', 'approximate_is_quiescent', 'approximate_grounded',
  'approximate_body_count', 'approximate_projectile_count', 'approximate_stat',
  'approximate_projectiles_out_of_bounds', 'approximate_projectiles_evicted',
  'approximate_total_contacts', 'approximate_error', 'approximate_position_stat',
];
export function createTowerRuntime(wasm) {
  for (const name of REQUIRED) {
    if (typeof wasm[name] !== 'function') throw new Error(`Tower WASM missing ${name}; reload the updated build`);
  }
  if (!(wasm.memory instanceof WebAssembly.Memory)) throw new Error('Tower WASM memory is unavailable');
  return Object.freeze({
    solver: 'fixed-step-f64',
    snapshotArray: Float64Array,
    orientationScale: 1,
    memory: wasm.memory,
    // Extra legacy UI arguments are intentionally not forwarded. Its baking/pass controls are disabled.
    sandbox_reset_tower_with_baking_options: rules => wasm.approximate_reset_tower(rules),
    sandbox_step_velocity: wasm.approximate_step_velocity,
    sandbox_shoot: wasm.approximate_shoot,
    sandbox_set_projectile_type: wasm.approximate_set_projectile_type,
    sandbox_refresh_render_snapshot: wasm.approximate_refresh_snapshot,
    sandbox_render_snapshot_len: wasm.approximate_snapshot_len,
    sandbox_render_snapshot_stride: () => 11,
    sandbox_is_quiescent: wasm.approximate_is_quiescent,
    sandbox_grounded: wasm.approximate_grounded,
    sandbox_body_count: wasm.approximate_body_count,
    sandbox_projectile_count: wasm.approximate_projectile_count,
    sandbox_active_projectile_count: wasm.approximate_projectile_count,
    sandbox_projectiles_retired_on_contact: () => wasm.approximate_stat(9),
    sandbox_projectiles_retired_out_of_bounds: wasm.approximate_projectiles_out_of_bounds,
    sandbox_projectiles_evicted_by_cap: wasm.approximate_projectiles_evicted,
    sandbox_last_collision_events: () => wasm.approximate_stat(4),
    sandbox_total_collisions: () => Number(wasm.approximate_total_contacts()),
    bodySleeping: wasm.approximate_body_sleeping,
    elapsed: () => wasm.approximate_stat(0),
    error: wasm.approximate_error,
    stepStats: () => ({
      fixed_position_passes: wasm.approximate_position_stat(0),
      fixed_position_bounds_tests: wasm.approximate_position_stat(1),
      fixed_position_contact_tests: wasm.approximate_position_stat(2),
      fixed_position_corrections: wasm.approximate_position_stat(3),
      fixed_substeps: wasm.approximate_stat(1),
      fixed_pair_tests: wasm.approximate_stat(2),
      fixed_narrow_tests: wasm.approximate_stat(3),
      fixed_contact_points: wasm.approximate_stat(4),
      fixed_impulse_iterations: wasm.approximate_stat(5),
      fixed_integrated_bodies: wasm.approximate_stat(6),
      fixed_woken_bodies: wasm.approximate_stat(7),
      fixed_swept_contacts: wasm.approximate_stat(8),
    }),
  });
}
