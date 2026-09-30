//! Dedicated native/WASM regression; absent from the production Pages module.
#[path = "../../tests/support/box_fixture_oracle.rs"]
mod box_fixture_oracle;
#[path = "../../tests/support/dense_contact.rs"]
mod dense;
#[path = "../../tests/support/contact_materials.rs"]
mod materials;
#[path = "../../tests/support/narrow_support.rs"]
mod narrow_support;
thread_local! {
    // Read-only acceptance measurements; this example is absent from production Pages.
    static NARROW_RESULTS: std::cell::RefCell<Option<[narrow_support::Measurements; 3]>> = const {
        std::cell::RefCell::new(None)
    };
}
#[unsafe(no_mangle)]
pub extern "C" fn dense_contact_contract() -> i32 {
    dense::run();
    materials::run();
    NARROW_RESULTS.with(|results| *results.borrow_mut() = Some(narrow_support::run()));
    0
}
#[unsafe(no_mangle)]
pub extern "C" fn narrow_support_metric(case: u32, cadence: u32, field: u32) -> f64 {
    NARROW_RESULTS.with(|results| {
        results
            .borrow()
            .as_ref()
            .and_then(|rows| rows.get(case as usize))
            .and_then(|row| row.values(cadence))
            .and_then(|values| values.get(field as usize).copied())
            .unwrap_or(f64::NAN)
    })
}
#[cfg(test)]
mod tests {
    #[test]
    fn native_dense_contact_contract() {
        super::dense::run();
        super::materials::run();
        super::narrow_support::run();
    }
}
