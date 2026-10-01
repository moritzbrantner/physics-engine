#[path = "support/contact_wake_forces.rs"]
mod controls;
#[test]
fn contact_wake_has_first_substep_forces_and_no_duplicate_load() {
    assert_eq!(controls::run(), controls::run());
}
