//! Native/WASM retained-storage contract; absent from the production Pages module.
#[path = "../../tests/support/translational_staging.rs"]
mod staging;

#[unsafe(no_mangle)]
pub extern "C" fn translational_maintenance_contract() -> i32 {
    staging::run();
    0
}

#[cfg(test)]
mod tests {
    #[test]
    fn native_translational_maintenance_contract() {
        super::staging::run();
    }
}
