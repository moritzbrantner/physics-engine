#[path = "support/primitive_contacts.rs"]
mod controls;

#[test]
fn independent_axial_primitive_contact_matrix() {
    assert_eq!(controls::run(), controls::run());
}
