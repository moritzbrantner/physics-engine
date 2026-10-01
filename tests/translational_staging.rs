#[path = "support/translational_staging.rs"]
mod contract;
#[path = "support/translational_report.rs"]
mod report;

#[test]
fn retained_staging_lifecycle_and_failure_contract() {
    contract::run();
}
