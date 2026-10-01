#[path = "support/fixed_contact_interval.rs"]
mod contract;

#[test]
fn finite_floor_keeps_its_actual_contact_when_reachable_corners_exceed_four() {
    assert!(contract::finite_floor_retains_actual_contacts() > 0.0);
}
#[test]
fn fixed_contact_intervals_preserve_floor_and_replay() {
    let first = contract::run();
    assert_eq!(first, contract::run());
    println!("FIXED_CONTACT_INTERVAL_NATIVE {first:?}");
}
