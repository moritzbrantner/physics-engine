#[path = "support/box_fixture_oracle.rs"]
mod box_fixture_oracle;
#[path = "support/narrow_support.rs"]
mod controls;

#[test]
fn narrow_fixed_support_quality() {
    controls::run();
}
