#[path = "support/translational_broad_phase.rs"]
mod contract;

#[test]
fn retained_broad_phase_ricochet_contract() {
    contract::run();
}
