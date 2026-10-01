#[path = "support/fixed_bound_workload.rs"]
mod fixed_bound_workload;

#[path = "support/translational_fixed_bounds.rs"]
mod fixed_bounds;

#[test]
fn fixed_bound_dependencies_and_lifecycle_keep_complete_physics() {
    fixed_bounds::run();
}

#[test]
fn actual_fixed_bound_work_partitions_every_query_and_repeats() {
    let first = fixed_bound_workload::run();
    let second = fixed_bound_workload::run();
    assert_eq!(first, second);
    for values in [first, second] {
        println!("FIXED_BOUND_CONTRACT {values:?}");
    }
}
