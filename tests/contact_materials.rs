#[path = "support/contact_materials.rs"]
mod contract;

#[test]
fn floating_contact_material_contract() {
    contract::run();
    contract::run();
    assert_eq!(
        contract::reciprocal_current_contact_momentum(),
        contract::reciprocal_current_contact_momentum()
    );
}
