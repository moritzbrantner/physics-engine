#[path = "support/narrow_support.rs"]
mod controls;
#[path = "support/obb_geometry.rs"]
mod obb_geometry;

#[test]
fn narrow_fixed_support_quality() {
    controls::run();
}
