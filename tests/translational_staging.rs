#[path = "support/translational_staging.rs"]
mod contract;

#[test]
fn retained_staging_lifecycle_and_failure_contract() {
    contract::run();
}
