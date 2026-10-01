#[path = "support/translational_broad_phase.rs"]
mod contract;
#[path = "support/translational_report.rs"]
mod report;

#[test]
fn retained_broad_phase_ricochet_contract() {
    contract::run();
}
