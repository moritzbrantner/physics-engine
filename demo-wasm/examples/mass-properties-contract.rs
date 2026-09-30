//! Shared public mass-property acceptance fixtures, separate from the shipped demo.
#[path = "../../tests/mass_properties.rs"]
mod contract;

#[unsafe(no_mangle)]
pub extern "C" fn mass_properties_contract() -> i32 {
    contract::sphere_and_zero_skeleton_capsule_share_uniform_products();
    contract::capsule_tensor_matches_cylinder_and_hemisphere_mass_integration();
    contract::box_and_wedge_report_com_and_complete_symmetric_tensors();
    contract::scaling_and_mass_changes_have_the_declared_units();
    contract::invalid_mass_and_shape_products_fail_at_the_boundary();
    0
}
