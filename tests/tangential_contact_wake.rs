#[path = "support/tangential_contact_wake.rs"]
mod fixture;
#[test]
fn admitted_external_tangent_motion_wakes_and_restores_loaded_response() {
    assert_eq!(fixture::run(), fixture::run());
}
