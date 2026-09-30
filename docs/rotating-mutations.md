# Explicit rotating-world mutations

The legacy rotating-box compatibility world exposes two commands for consumers retaining an engine instance. They operate on the existing integer body API; new floating-state consumers continue to use the separate floating world. These commands supply the missing mutation boundary for [ECS Lab #133](https://github.com/moritzbrantner/ecs-lab/issues/133), alongside [atomic intervals](rotating-intervals.md). They do not implement that consumer's persistent lifetime or migrate its numerical representation.

## Descriptor changes

`RotatingWorld3d::replace_box(replacement)` updates one existing body in place and returns whether its descriptor changed. Use it for an actual teleport, shape/material/mass change, fixed/dynamic transition, collision eligibility or motion-authority change. Construct the validated `RigidBox3d` first. A missing ID fails; replacement never creates a new member. The ID, interaction category and category-pair policy remain registered throughout the command.

An equal descriptor returns `false` before geometry queries, wake work or cache changes. A changed descriptor validates its stationary conservative bounds and resolves old/new contact dependencies before committing anything. A returned geometry/contact error preserves authoritative bodies, parked membership and sleep state. Query caches may accumulate derived work during validation. The commit moves one descriptor into its existing body slot and updates the solver partitions and affected contact-cache generation. It does not remove/reinsert bodies or clone the world for rollback.

Actual support changes wake dependent parked dynamics. New solid geometry admits exact OBB contacts before activation; a separated shape or overlap-only body is not sufficient wake evidence. Traversal follows dynamic dependencies and does not pass through a shared fixed floor to unrelated islands. Editing the floor itself can affect every body it supports. Existing sleep-timer invalidation on an actual parked-body activation remains in force; this API does not redefine that policy.

Genuine fixed geometry is unregistered/reprepared when its shape, kind or solver participation changes. A material-only edit retains its existing prepared geometry. Runtime and prepare-at-load modes produce the same physical/query results after fixed geometry moves, becomes dynamic, becomes fixed again, is removed or reuses an ID.

## Intended motion

Use a clone of the single authoritative body with `RigidBox3d::with_body` to edit its translational/material descriptor while retaining angular state and policies. The builder validates identity, half extents, mass and fixed-body spin without normalizing an already simulated quaternion again. `with_motion_authority` changes ownership without reconstructing physical state; an unchanged authority preserves sleep policy, external ownership disables sleep, and returning to physics selects normal sleep.

`RotatingWorld3d::set_motion(id, linear_velocity, angular_velocity)` validates and applies both components together, returning whether effective motion changed. A missing body fails. A fixed body rejects nonzero linear or angular velocity before wake/mutation; zero is a no-op. Rotation locks suppress supplied angular velocity. Equal effective motion preserves a parked body and its sleep history. Changed motion activates the target and stores both components in authoritative physics state. Later contact admission controls propagation to other bodies.

These APIs deliberately do not accept a full-world synchronization snapshot. Consumers should retain identity/metadata mappings, submit only intended changes, let physics own simulated pose and velocity, and treat converted output as an observable view. A normal unchanged tick calls neither mutation command. Full output conversion may still be required by a consumer's export contract.

## Acceptance and work limits

`tests/rotating_mutations.rs` exercises descriptor/authority changes, identity/category retention, unrelated bodies/worlds, equal commands on 128 parked bodies, malformed/missing/fixed motion commands, failed bounds with future replay, support removal, real new contacts, near misses, overlap-only eligibility, rotation locks, fixed preparation and kind/ID transitions. The same public fixtures execute in the existing separate WASM interval-contract module, leaving production demo exports unchanged.

Equal commands require a body lookup/comparison and retain no additional physical state. Actual changes can pay for contact queries, dynamic-island traversal and derived cache maintenance; new-contact admission scans borrowed active dynamic membership and queries the parked bounds index. It uses replacement eligibility, including changed layers, rather than a current-body graph with old filters. This is not a claim that all mutation work is constant or that those queries perform no scans. The parked original-body store remains the existing sleep representation, not an additional ECS pose mirror. The unchanged-frame test verifies zero motion, sleep and parked before-images in the following four-substep interval. Complete consumer construction, mutation, conversion/writeback and memory measurements remain part of ECS Lab #133.
