//! Execute the same public API acceptance fixtures natively and on WASM.
//! This diagnostic module is separate from the shipped demo.
#[path = "../../tests/rotating_interval.rs"]
mod contract;
#[path = "../../tests/rotating_mutations.rs"]
mod mutations;

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
    contract::zero_interval_retains_pose_and_applies_explicit_consumer_damping();
    mutations::descriptor_changes_keep_identity_policy_and_unrelated_state();
    mutations::equal_commands_preserve_parked_sleep_and_zero_interval_work();
    mutations::rejected_replacement_and_motion_leave_sleep_and_future_steps_unchanged();
    mutations::teleporting_support_wakes_only_its_dynamic_contact_island();
    mutations::new_geometry_admits_real_contacts_and_ignores_near_misses();
    mutations::intended_motion_wakes_target_and_honors_rotation_lock();
    mutations::fixed_preparation_and_kind_transitions_remain_equivalent();
    mutations::authority_changes_update_solver_partitions_without_cross_world_effects();
    mutations::overlap_only_edits_preserve_sleep_and_refresh_contact_eligibility();
    mutations::newly_eligible_layers_wake_a_dependency_beyond_the_replacement_bounds();
    mutations::material_edits_preserve_simulated_angular_state_and_body_policy();
    mutations::interval_event_totals_include_completed_discarded_contact_work();
    0
}
