//! Execute the same public API acceptance fixtures natively and on WASM.
//! This diagnostic module is separate from the shipped demo.
#[path = "../../tests/rotating_interval.rs"]
mod contract;

#[unsafe(no_mangle)]
pub extern "C" fn rotating_interval_contract() -> i32 {
    contract::late_substep_failure_restores_the_requested_interval();
    contract::late_failure_restores_sleep_deadline_and_newly_parked_membership();
    contract::late_failure_restores_an_activated_parked_body_and_future_impacts();
    contract::successful_partition_matches_solver_commands_and_damps_once();
    contract::invalid_partitions_and_zero_intervals_do_not_change_motion();
    contract::fully_parked_intervals_do_not_copy_stationary_scene_state();
    contract::ballistic_membership_is_rejected_before_interval_work();
    contract::fixed_preparation_remains_equivalent_after_rollback_and_id_reuse();
    contract::interval_damping_preserves_externally_owned_angular_velocity();
    contract::damping_rounds_nearest_and_reports_a_motion_only_delta();
    0
}
