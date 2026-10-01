#[path = "support/moving_support.rs"]
mod controls;

#[test]
fn moving_support_carry_departure_and_removal() {
    let first = controls::run();
    let second = controls::run();
    for (a, b) in first.iter().zip(&second) {
        for cadence in 0..2 {
            assert_eq!(a.values(cadence), b.values(cadence));
        }
    }
}
