# Atomic rotating intervals

`RotatingWorld3d::advance_interval` partitions one rational physical interval into a
positive number of equal substeps using the existing integer rotating solver. This
is a compatibility API; new CPU physics state continues to use `numeric::Scalar`.
It is the engine prerequisite for the persistent rotating adapter in
[ecs-lab#133](https://github.com/moritzbrantner/ecs-lab/issues/133).

The command validates the numerator, denominator, partition multiplication and
angular damping before stepping. It currently accepts rigid-box worlds only and
rejects any ballistic membership before mutation. Ballistic motion and retirement
are intentionally outside this transaction. A zero interval retains the existing
no-op physical and sleep-history contract.

On success, the caller's reusable report vector contains every completed substep
in order. Angular damping is applied once, after the complete interval, using the
existing integer milli scale and truncation toward zero. The last report includes
any damping changes in its sorted `changed_body_ids`. Caller-owned angular velocity on externally driven bodies is preserved. Damping
visits the retained active dynamic partition, counting externally driven entries
that it skips; it does not visit fixed geometry or parked dynamics.

On any returned solver error, the vector is cleared and all interval changes to
body motion, sleep deadlines/candidate membership, boundary obligations and parked
membership are restored. The existing `step` command keeps its documented behavior;
this complete-interval guarantee is opt-in. Panic recovery is not part of either
command contract.

The transaction records a first before-image only for a touched body. Motion
images contain kind, position, linear velocity and angular state, rather than a
complete rigid body. Sleep images contain the body's sleep/candidate flags,
optional stability deadline and pending-boundary flag. Parking images exist only
for actual park/unpark transitions and retain the original dynamic representation
when necessary. Temporary sleeping proxies are restored independently and are not
mistaken for persistent park transitions. No complete world, scene, body map or
sleep map is copied for rollback.

On error, retained broad-phase geometry and current-contact geometry are
invalidated instead of copied. Their work counters remain cumulative. Contact
response scratch has no authoritative contact history and is prepared by the
existing solver on reuse. Only touched parked-index entries are restored.

`RotatingIntervalWork3d` counts completed substeps, journal entries, damping visits
and changes. Four diagnostic contact counters include discarded substeps and wake
probes. These counters are a subset of total work: they exclude current-contact
cache geometry tests, solver scratch allocation, fixed-geometry preparation and
allocator/tree overhead. Journal-entry counts are not RSS or allocated-byte
measurements. Complete-call timing is required for performance claims.

The public acceptance fixtures cover a second-substep overflow after a successful
first step, restoring a newly parked body and its deadline, restoring an admitted
wake and subsequent impacts, successful partition equivalence, once-per-interval
damping, invalid/zero intervals, ballistic rejection, and a fully parked 128-body
zero-journal/zero-contact-work interval, and fixed-preparation parity after rollback,
support removal, ID reuse and external motion authority. The separate Pages acceptance driver runs
the same fixtures three times on WASM and is not shipped in the demo module.

This slice does not implement the persistent ECS adapter, ECS lifecycle deltas,
its playground/tower routing, consumer parity or consumer performance acceptance.
Those remain requirements of ecs-lab#133.
