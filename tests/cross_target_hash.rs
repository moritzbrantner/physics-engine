//! Native half of the cross-target state hash (#576): the fixture's hash is pinned, and
//! `scripts/cross-target-wasm.sh` checks the same pinned value on wasm32. A change that moves the
//! hash on purpose (solver behaviour) updates `PINNED` in `tests/support/cross_target.rs`; the
//! wasm32 run must then agree with it.
#[path = "support/cross_target.rs"]
mod fixture;

#[test]
fn state_hash_is_pinned() {
    let hash = fixture::run();
    assert_eq!(
        hash,
        fixture::PINNED,
        "cross-target state hash moved: {hash:#018x}"
    );
}
