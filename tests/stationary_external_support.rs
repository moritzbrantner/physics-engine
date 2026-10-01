#[path = "support/stationary_external_support.rs"]
mod fixture;
#[test]
fn natural_external_support_parking_restarts_and_releases_real_dynamics() {
    assert_eq!(fixture::run(), fixture::run());
}
