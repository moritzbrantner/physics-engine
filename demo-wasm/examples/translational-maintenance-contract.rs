//! Native/WASM retained-storage contract; absent from the production Pages module.
#[path = "../../tests/support/translational_broad_phase.rs"]
mod broad_phase;
#[path = "../../tests/support/fixed_bound_workload.rs"]
mod fixed_bound_workload;
#[path = "../../tests/support/translational_fixed_bounds.rs"]
mod fixed_bounds;
#[path = "../../tests/support/translational_staging.rs"]
mod staging;

thread_local! {
    static FIXED_RESULTS: std::cell::Cell<Option<[f64; 15]>> = const {
        std::cell::Cell::new(None)
    };
}

#[unsafe(no_mangle)]
pub extern "C" fn translational_maintenance_contract() -> i32 {
    staging::run();
    broad_phase::run();
    fixed_bounds::run();
    FIXED_RESULTS.with(|results| results.set(Some(fixed_bound_workload::run())));
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn fixed_bound_metric(field: u32) -> f64 {
    FIXED_RESULTS.with(|results| {
        results
            .get()
            .and_then(|values| values.get(field as usize).copied())
            .unwrap_or(f64::NAN)
    })
}

#[cfg(test)]
mod tests {
    #[test]
    fn native_translational_maintenance_contract() {
        super::staging::run();
        super::broad_phase::run();
        super::fixed_bounds::run();
        super::fixed_bound_workload::run();
    }
}
