//! Dedicated native/WASM regression; absent from the production Pages module.
#[path = "../../tests/support/dense_contact.rs"]
mod dense;
#[path = "../../tests/support/contact_materials.rs"]
mod materials;
#[unsafe(no_mangle)]
pub extern "C" fn dense_contact_contract() -> i32 {
    dense::run();
    materials::run();
    0
}
#[cfg(test)]
mod tests {
    #[test]
    fn native_dense_contact_contract() {
        super::dense::run();
        super::materials::run();
    }
}
