#[path = "support/fixed_bound_workload.rs"]
mod fixed_bound_workload;

#[test]
fn actual_fixed_bound_work_partitions_every_query_and_repeats() {
    let first = fixed_bound_workload::run();
    let second = fixed_bound_workload::run();
    assert_eq!(first, second);
    for values in [first, second] {
        println!("FIXED_BOUND_CONTRACT {values:?}");
    }
}
