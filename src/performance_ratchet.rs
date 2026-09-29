//! Test-only structured evidence; no instrumentation or interface is added to production builds.

use std::cell::Cell;

thread_local! {
    pub(crate) static PROJECTION_DOTS: Cell<u64> = const { Cell::new(0) };
}

pub(crate) fn record(
    scenario: &str,
    work: &[(&str, u64)],
    correctness: &[(&str, u64)],
    timing: &[(&str, f64)],
) {
    fn fields<T: std::fmt::Display>(values: &[(&str, T)]) -> String {
        values
            .iter()
            .map(|(key, value)| {
                assert!(key.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'_'));
                format!("{key:?}:{value}")
            })
            .collect::<Vec<_>>()
            .join(",")
    }
    assert!(
        scenario
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || b"/-_".contains(&b))
    );
    assert!(
        timing
            .iter()
            .all(|(_, value)| value.is_finite() && *value >= 0.0)
    );
    println!(
        "PERFORMANCE_RATCHET {{\"scenario\":{scenario:?},\"work\":{{{}}},\"correctness\":{{{}}},\"timing\":{{{}}}}}",
        fields(work),
        fields(correctness),
        fields(timing)
    );
}
