#[path = "support/driven_parked_contact.rs"]
mod controls;
#[test]
fn external_contact_admission_restores_parked_response() {
    assert_eq!(controls::run(), controls::run());
}
