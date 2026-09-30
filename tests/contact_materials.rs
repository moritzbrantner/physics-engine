#[path = "support/contact_materials.rs"]
mod contract;

#[test]
fn floating_contact_material_contract() {
    contract::run();
    contract::run();
}
