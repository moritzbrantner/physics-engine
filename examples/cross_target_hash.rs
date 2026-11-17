//! WASM half of the cross-target state hash (#576): the fixture of `tests/cross_target_hash.rs`,
//! which panics (traps in WASM) unless its hash equals the pinned native one.
//! `scripts/cross-target-wasm.sh` builds it for wasm32-unknown-unknown and runs it in Node;
//! natively, `cargo run --example cross_target_hash`.
#[path = "../tests/support/cross_target.rs"]
mod fixture;

fn main() {
    let hash = fixture::run();
    assert_eq!(hash, fixture::PINNED, "{hash:#018x}");
}
