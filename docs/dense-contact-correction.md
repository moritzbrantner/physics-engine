# Admitted-contact position correction

The current-main dense impact fixture reproduces box penetration on tick 122 with 128 awake dynamic crates, a fixed floor, unchanged sphere/arrow/rigid projectile waves, 1/60-second ticks and a four-substep/eight-velocity/two-position-pass budget. Independent full OBB SAT observes a peak overlap of 4.752737337377873 against the existing 1.8-unit screening limit. Both repeated release-WASM runs fail identically. Unforced 32/64/128 stacks and the smaller impact scenes pass the original policy.

Deletion minimization reduces the failure to two tilted 36-unit crates and one rotation-locked CCD arrow. The exact poses, velocities, orientations, angular velocities, materials and IDs live in [the shared regression fixture](../tests/support/dense_contact.rs). It requires no floor or warm-start history. A single 1/60-second tick takes the initial 0.024016-unit overlap to 6.334306581237058 in the fixed-only comparison mode. Removing either crate or the projectile removes the failing pair.

## Cause and rejected alternatives

The arrow changes a crate's velocity and spin after its neighbor contact was clipped. Only the near corners fall inside contact slop; an opposite corner can close during integration without a matching constraint. Sweeps hold orientations fixed and the bounded velocity solver alone does not remove the resulting dynamic overlap. Forcing all eight velocity passes reproduces the same minimized failure, ruling out early convergence as its cause. Locking both crate rotations removes that particular failure but changes the physical system and is not a repair.

Unconditionally retaining the clipped footprint fixes the minimized case but fails an unforced 32-box control. Spreading four retained points over the footprint fixes that control but fails 64/128-box impact controls. Keeping every clipped vertex still fails the full 128-box impact scene. Expanding only after swept impact/angular evidence also fails a smaller impact control. These candidates were rejected, with unchanged populations, schedules and thresholds. The production clipper, contact-point bound and CCD kernels remain unchanged.

## Explicit policy and bounded work

`Config::position_correction` defaults to `PositionCorrection::FixedColliders`, preserving the previous comparison policy. `AdmittedContacts` additionally corrects residual overlap between awake movable pairs that already generated constraints during this substep. Tower Stability and the diagnostic capacity fixture explicitly select that policy.

The historical `fixed_position_iterations` field supplies one shared position-pass budget. Dynamic and fixed correction run within each pass; no extra velocity passes, substeps, chronological events or simulation time are added. Zero still disables all position correction. The selected tower budget remains four substeps, at most eight velocity passes and two position passes per substep.

Dynamic pairs follow stable BodyId order from admitted constraints, are visited once per pair per pass, and regenerate their current manifold after preceding corrections. This traverses admitted rows rather than scanning every body pair. No new contact is admitted by this stage. A later contact discovered after integration still requires the next substep's regular contact/CCD path. Sleeping, fixed, external and sensor bodies are excluded from dynamic correction; layer and one-way response restrictions remain authoritative. Wake probes and impulses occur before this stage under their existing real-contact policy.

Correction translates along the fresh contact normal, leaves contact slop and distributes displacement by allowed inverse mass. With reciprocal response this preserves the pair's mass-weighted center. It leaves linear/angular velocity, orientation and elapsed time unchanged, so it adds no kinetic correction energy. It does not promise conservation of angular momentum about an external origin, exact restitution, rotational positional correction or a global overlap bound. Material corrections reset quiet timers, rather than force-sleeping bodies that remain under correction.

The existing transaction journal has already captured each integrated awake endpoint; a later returned error rolls back these position changes too. `PositionReport` exposes dynamic pair visits, contact tests, corrections and exact geometry work. Discarded position geometry is included in failed-step diagnostics. Scratch clipping is reused. Public snapshot queries and ordinary geometry cache admission remain unchanged.

## Acceptance and limits

The shared public-API regression executes natively and as a separate release-WASM driver. It checks independent OBB penetration at the full-tick boundary and each equivalent physical substep, real CCD retirement, unchanged work ceilings and complete requested time. A focused position-stage control verifies unequal-mass center preservation and unchanged velocities, orientation and time. Checkpoint continuation preserves the explicit policy; format 2/algorithm 2 reject older products rather than silently changing their continuation.

The same `Fixture` also powers [the diagnostic viewer](../experiments/solver-budget/inspect.html). Build the diagnostic release-WASM module, serve the checkout with `python3 -m http.server 4191`, and open `/experiments/solver-budget/inspect.html`. Its policy selector runs the fixed-only adverse comparison and the repaired mode through the same Rust export. Reset, step, run/pause and 300-tick batches never derive authoritative state in JavaScript. Rust observations supply the quality checks; canvas only projects body snapshots.

Run the original and 20-second native matrix explicitly:

```sh
cargo test --release --locked --manifest-path experiments/solver-budget/Cargo.toml \
  dense_contact_original_and_long_trace_quality -- --ignored --nocapture
```

Release-WASM controls use the existing diagnostic crate and runner, with `COUNTS=32,64,128`, `SCENES=0,1`, `PROFILES=4s-8v-2p` and two repetitions. Thresholds remain floor 0.5, pair penetration 1.8 and unforced displacement 1.8. Sleeping is disabled on capacity scenes. Native and WASM must repeat within their own build/target; their trajectories need not have identical bits. [The retained evidence summary](dense-contact-quality-2026-09-30.json) records 24 native and 24 release-WASM traces, original adverse controls, rejected candidates, phases, work and reported memory. Every repaired run completes and repeats. Native peak overlap is 1.482849 on the 128-box impact case; WASM is 0.852552. All other repaired bounds pass, including every longer trace. These are separate supported build/target observations, not a cross-target bit-identity claim.

The repaired 20-second WASM impact p95 is approximately 2.02/6.73/17.24 ms at 32/64/128 boxes. Unforced 128-box active p95 is approximately 21.29 ms; startup p95 is approximately 21.65 ms. The explicit sleeping controls reach the quiescent path; warmed sleeping p95 is about 0.0004 ms, with the sleep-transition outlier retained. Sleeping bodies are never used as active capacity evidence. The 128-box repair does not establish a 16.67-ms physics or rendered-frame budget.

For comparable first-300-tick windows, passing mixed-impact controls increase from approximately 1.61 to 2.34 ms (32) and 5.90 to 7.40 ms (64) at worst-repetition active p95. The failing original 128-box impact run is excluded as a performance denominator. This is a correctness repair with visible cost, not an optimization claim. Timing is advisory on this shared host. Reported derived payload peaks at approximately 2.00 MB for the 128-box impact case; it is a documented subset, not retained world memory or RSS.

The clean code producer is `9a0c00846b1f8a20896c8c09371bf4775fe4cd3a`. Its existing 31 repository ratchet scenarios retain identical work and correctness against epoch 8; no thresholds or epoch changed. Those controls protect their named legacy/default workloads and do not substitute for the new independent overlap gates. Browser controls reproduce comparison failure on tick 122 and the repaired 300-tick pass, exercise reset/step/run/pause, and load the real 44-body Pages tower without page errors. Pages native/WASM volley, near-miss, immediate-arrow and mixed-burst checks also pass.

The existing Performance Evidence job also runs and blocks on the full original 300-tick 32/64/128 stack/impact matrix, not only the minimized regression. Its validator rejects missing populations/repetitions, incomplete/error steps, physical failures, changed budgets/thresholds and replay mismatches. Historical failing controls remain in the retained evidence rather than usable capacity.

This addresses one impact/contact-coverage failure class under #193. General dense rubble, moving supports, all primitive/material/mass combinations and consumer adoption beyond the Pages tower remain separate acceptance work. Passing this slice does not complete #193 or the full numerical migration. Resolved convention sourceRevision: `e6acb5310afaf15c0cba24f87108f5f4ad1bedc3`.
