#[path = "support/fixed_contact_interval.rs"]
mod contract;
#[test]
fn fixed_contact_intervals_preserve_floor_and_replay() {
    let first = contract::run();
    assert_eq!(first, contract::run());
    println!("FIXED_CONTACT_INTERVAL_NATIVE {first:?}");
}
