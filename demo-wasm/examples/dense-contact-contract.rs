//! Dedicated native/WASM regression; absent from the production Pages module.
#[path = "../../tests/support/box_fixture_oracle.rs"]
mod box_fixture_oracle;
#[path = "../../tests/support/contact_wake_forces.rs"]
mod contact_wake_forces;
#[path = "../../tests/support/dense_contact.rs"]
mod dense;
#[path = "../../tests/support/driven_parked_contact.rs"]
mod driven_parked_contact;
#[path = "../../tests/support/fixed_contact_interval.rs"]
mod fixed_contact_interval;
#[path = "../../tests/support/contact_materials.rs"]
mod materials;
#[path = "../../tests/support/moving_support.rs"]
mod moving_support;
#[path = "../../tests/support/narrow_support.rs"]
mod narrow_support;
#[path = "../../tests/support/primitive_contacts.rs"]
mod primitive_contacts;
#[path = "../../tests/support/stationary_external_support.rs"]
mod stationary_external_support;
#[path = "../../tests/support/tangential_contact_wake.rs"]
mod tangential_contact_wake;
thread_local! {
    static FIXED_INTERVAL_RESULTS: std::cell::Cell<Option<[f64; 5]>> = const {
        std::cell::Cell::new(None)
    };
    static STATIONARY_RESULTS: std::cell::Cell<Option<[[f64; 21]; 36]>> = const {
        std::cell::Cell::new(None)
    };
    static TANGENT_RESULTS: std::cell::Cell<Option<[[f64; 20]; 24]>> = const {
        std::cell::Cell::new(None)
    };
    static FORCE_RESULTS: std::cell::Cell<Option<[[f64; 17]; 16]>> = const {
        std::cell::Cell::new(None)
    };
    static DRIVEN_RESULTS: std::cell::Cell<Option<[[f64; 18]; 20]>> = const {
        std::cell::Cell::new(None)
    };
    // Read-only acceptance measurements; this example is absent from production Pages.
    static NARROW_RESULTS: std::cell::RefCell<Option<[narrow_support::Measurements; 3]>> = const {
        std::cell::RefCell::new(None)
    };
    static MOMENTUM_RESULTS: std::cell::Cell<Option<[f64; 4]>> = const {
        std::cell::Cell::new(None)
    };
    static PRIMITIVE_RESULTS: std::cell::Cell<Option<[f64; 10]>> = const {
        std::cell::Cell::new(None)
    };
    static MOVING_RESULTS: std::cell::RefCell<Option<[moving_support::Measurements; 6]>> = const {
        std::cell::RefCell::new(None)
    };
}
#[unsafe(no_mangle)]
pub extern "C" fn dense_contact_contract() -> i32 {
    FIXED_INTERVAL_RESULTS.with(|results| results.set(Some(fixed_contact_interval::run())));
    STATIONARY_RESULTS.with(|results| results.set(Some(stationary_external_support::run())));
    TANGENT_RESULTS.with(|results| results.set(Some(tangential_contact_wake::run())));
    FORCE_RESULTS.with(|results| results.set(Some(contact_wake_forces::run())));
    DRIVEN_RESULTS.with(|results| results.set(Some(driven_parked_contact::run())));
    dense::run();
    materials::run();
    MOMENTUM_RESULTS.with(|results| {
        results.set(Some(materials::reciprocal_current_contact_momentum()));
    });
    NARROW_RESULTS.with(|results| *results.borrow_mut() = Some(narrow_support::run()));
    PRIMITIVE_RESULTS.with(|results| results.set(Some(primitive_contacts::run())));
    MOVING_RESULTS.with(|results| *results.borrow_mut() = Some(moving_support::run()));
    0
}
#[unsafe(no_mangle)]
pub extern "C" fn fixed_contact_interval_metric(field: u32) -> f64 {
    FIXED_INTERVAL_RESULTS.with(|results| {
        results
            .get()
            .and_then(|values| values.get(field as usize).copied())
            .unwrap_or(f64::NAN)
    })
}
#[unsafe(no_mangle)]
pub extern "C" fn contact_momentum_metric(field: u32) -> f64 {
    MOMENTUM_RESULTS.with(|results| {
        results
            .get()
            .and_then(|values| values.get(field as usize).copied())
            .unwrap_or(f64::NAN)
    })
}
#[unsafe(no_mangle)]
pub extern "C" fn stationary_support_metric(case: u32, field: u32) -> f64 {
    STATIONARY_RESULTS.with(|results| {
        results
            .get()
            .and_then(|rows| rows.get(case as usize).copied())
            .and_then(|row| row.get(field as usize).copied())
            .unwrap_or(f64::NAN)
    })
}
#[unsafe(no_mangle)]
pub extern "C" fn tangential_contact_metric(case: u32, field: u32) -> f64 {
    TANGENT_RESULTS.with(|results| {
        results
            .get()
            .and_then(|rows| rows.get(case as usize).copied())
            .and_then(|row| row.get(field as usize).copied())
            .unwrap_or(f64::NAN)
    })
}
#[unsafe(no_mangle)]
pub extern "C" fn contact_wake_force_metric(case: u32, field: u32) -> f64 {
    FORCE_RESULTS.with(|results| {
        results
            .get()
            .and_then(|rows| rows.get(case as usize).copied())
            .and_then(|row| row.get(field as usize).copied())
            .unwrap_or(f64::NAN)
    })
}
#[unsafe(no_mangle)]
pub extern "C" fn driven_parked_metric(case: u32, field: u32) -> f64 {
    DRIVEN_RESULTS.with(|results| {
        results
            .get()
            .and_then(|rows| rows.get(case as usize).copied())
            .and_then(|row| row.get(field as usize).copied())
            .unwrap_or(f64::NAN)
    })
}
#[unsafe(no_mangle)]
pub extern "C" fn moving_support_metric(case: u32, cadence: u32, field: u32) -> f64 {
    MOVING_RESULTS.with(|results| {
        results
            .borrow()
            .as_ref()
            .and_then(|rows| rows.get(case as usize))
            .and_then(|row| row.values(cadence))
            .and_then(|values| values.get(field as usize).copied())
            .unwrap_or(f64::NAN)
    })
}
#[unsafe(no_mangle)]
pub extern "C" fn primitive_contact_metric(field: u32) -> f64 {
    PRIMITIVE_RESULTS.with(|results| {
        results
            .get()
            .and_then(|values| values.get(field as usize).copied())
            .unwrap_or(f64::NAN)
    })
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
        super::contact_wake_forces::run();
        super::tangential_contact_wake::run();
        super::stationary_external_support::run();
        super::driven_parked_contact::run();
        super::dense::run();
        super::materials::run();
        super::materials::reciprocal_current_contact_momentum();
        super::narrow_support::run();
        super::primitive_contacts::run();
        super::moving_support::run();
    }
}
