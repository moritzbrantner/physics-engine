# Conservative floating candidate bounds

Floating-world collision admission must not rely on physical `contact_slop` to
cover floating-point rounding. Cached pose bounds and swept endpoint bounds now
carry separate numerical guards. These widen broad-phase candidates only: they
do not change narrow-phase distance, contact depth, correction, solver budgets or
search tolerances.

## Reproduced rejection

At base `af9af5beb37d3954b4932aec82f6056e4aa50fc3`, a rotated box with half extents
`[1,2,3]`, position `[-11,4,3]` and normalized quaternion `[0.2,0.3,0.1,0.9]` has a
raw lower Y bound `0.9473684210526319`. Independently rotating local corner
`[-1,-2,3]` gives Y `0.9473684210526314`. Different arithmetic grouping loses several
rounding units of enclosure.

A dynamic sphere of radius 1e-6 at that corner minus `Y * radius`, with its Y
coordinate moved one representable value upward, is admitted by the unpruned
current-contact query. With a valid `contact_slop = 1e-20`, the floating world
rejects the pair before narrow phase and reports zero contacts. The minimized
public fixture fails natively and traps in the old WASM acceptance module.

Ranked probes distinguished raw rounding from an incorrect wedge fixture or a
large extent-formula error. A second probe removed only displacement padding:
a small rotated collider moving by about 1.67e9 units had an endpoint surface
coordinate `1666666666.6666586`, beyond the swept upper bound
`1666666666.6666584`. Endpoint addition needs a guard at the displacement scale.

## Numerical policy

Cached pose bounds expand by `64 * EPSILON * (1 + max_abs_position + max_extent)`.
The 64-ULP-scale budget matches existing ray pruning and covers the bounded
quaternion rotation, weighted-extent and translation expressions. The unit term
keeps the guard nonzero for small coordinates; the position/extent terms scale
with input magnitude. This is an explicit numerical admission allowance, not
physical penetration slop and not a formal exact-arithmetic interval proof.

Swept rows retain contact-slop and angular-motion padding, and add
`8 * EPSILON * max_abs_displacement` for endpoint addition. Existing query pruning
retains its own scale-aware search guards. Invalid/nonfinite state still follows
the existing checked input/search/atomic-step error paths. No saturating cast,
exact-reference fallback, allocation or new caller parameter is introduced.

The shared public native/WASM fixture checks repaired corner admission and a
nearby separated candidate that reaches narrow phase without receiving contact,
impulse or correction. Native geometry gates cover 1536 independently rotated
surface-extrema cases over all four shapes and scales 1e-6, 1 and 1e6, plus 24
real swept-row endpoint cases with both displacement signs. These are bounded
regressions, not a general angular CCD or dense-contact quality claim.

```sh
cargo test --locked --test floating_conservative_bounds
cargo test --locked --lib bounds_tests
cargo test --manifest-path demo-wasm/Cargo.toml --locked --example shape-query-contract
```

The complete Pages build runs the same public fixture in a separate query
acceptance WASM module. Production demo exports remain unchanged. The capability
ledger in #183 remains a following slice; it must consume these repaired bounds
rather than advertise the earlier unpadded helper as conservative.

Shared convention sourceRevision:
`e6acb5310afaf15c0cba24f87108f5f4ad1bedc3`.
