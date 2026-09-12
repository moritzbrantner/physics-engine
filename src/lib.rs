//! Reusable deterministic physics simulation primitives.
//!
//! The engine owns deterministic translational rigid-body stepping plus engine-local rotational and
//! contact geometry foundations: gravity, fixed/dynamic bodies, continuous collision detection,
//! time-of-impact resolution, restitution, deterministic spatial queries, physics-native collider
//! geometry, fixed-point orientation/angular velocity, box inertia, angular impulse evidence, exact
//! quantized OBB SAT contact seeds, conservative rotational sweep bounds, canonical rotating-box
//! free-flight sampling, deterministic balanced-AABB-BVH rotational broad-phase pairing, explicit
//! sampled rotating contact/re-contact search, shared first/re-contact frontiers, deterministic
//! OBB/frontier response, Coulomb-limited OBB friction, bounded repeated sampled-event advancement,
//! and a rotating-box world that consumes persistent contact tails deterministically. The original
//! `World` solver remains translational and AABB-only. The public `RotatingWorld3d` defaults to an
//! ECS-backed entity/component world that runs the stabilized rotating solver as a physics system; the raw
//! solver resource remains available as `PhysicsWorld3dKernel` for deliberately lower-level integrations.
//!
//! Rendering, game loops and non-physics components remain consumer-owned.

#![forbid(unsafe_code)]

mod angular;
mod body;
mod collider;
mod collision;
mod ecs_world;
mod math;
mod obb_friction;
mod obb_response;
mod oriented_box;
mod query;
mod repeated_rotating_events;
mod rigid_box;
mod rigid_box_free_flight;
mod rotating_broad_phase;
mod rotating_contact_frontier;
mod rotating_contact_response;
mod rotating_contact_search;
mod rotating_recontact_search;
mod rotating_world;
mod rotational_sweep;
mod stabilized_rotating_world;
mod wide_ratio;
mod world;

pub use angular::{
    ANGULAR_VELOCITY_SCALE, AngularError3d, AngularState3d, AngularVelocity3d, BoxInertia3d,
    ORIENTATION_SCALE, Orientation3d, box_inertia, contact_angular_impulse, integrate_orientation,
};
pub use body::{BodyId, BodyKind, MATERIAL_SCALE, Material, RigidBody};
pub use collider::{Collider, ColliderContact, ColliderError, ColliderShape, collider_contact};
pub use collision::{
    ContactNormal, SUBTICKS_PER_TICK, SweepHit, TimeOfImpact, overlap_aabb, swept_aabb,
};
pub use ecs_world::EcsRotatingWorld3d as RotatingWorld3d;
pub use math::Vec3i;
pub use obb_friction::resolve_obb_contact;
pub use obb_response::{ObbContactResponse3d, ObbContactResponseError3d, ObbResolvedContact3d};
pub use oriented_box::{
    ObbAxisFeature3d, ObbContactSeed3d, OrientedBox3d, OrientedBoxError3d, obb_contact_seed,
    oriented_box_vertices,
};
pub use query::{Aabb, QueryError, QueryHit, Ray};
pub use repeated_rotating_events::{
    MAX_REPEATED_ROTATING_EVENTS, RepeatedRotatingEventAdvance3d, RepeatedRotatingEventConfig3d,
    RepeatedRotatingEventError3d, RotatingResolvedEvent3d, advance_repeated_rotating_events,
};
pub use rigid_box::{RigidBox3d, RigidBoxError3d};
pub use rigid_box_free_flight::{
    RigidBoxFreeFlightConfig3d, RigidBoxFreeFlightError3d, rigid_box_free_flight_sweep_bounds,
    sample_rigid_box_free_flight,
};
pub use rotating_broad_phase::{
    RotatingBroadPhaseError3d, RotationalSweepPair3d, rotational_sweep_candidate_pairs,
};
pub use rotating_contact_frontier::{
    RotatingContactFrontier3d, RotatingContactFrontierError3d, earliest_rotating_contact_frontier,
    next_rotating_contact_frontier,
};
pub use rotating_contact_response::{
    RotatingContactResponse3d, RotatingContactResponseError3d, resolve_rotating_contact_frontier,
};
pub use rotating_contact_search::{
    RotatingContactSearchConfig3d, RotatingContactSearchError3d, RotatingContactSearchHit3d,
    SampledContactTime3d, sampled_rotating_contact_search,
};
pub use rotating_recontact_search::sampled_rotating_recontact_search;
pub use rotating_world::{
    RotatingWorldConfig3d, RotatingWorldError3d, RotatingWorldStepReport3d,
    RotatingWorldStepStats3d,
};
pub use rotational_sweep::{
    RotationalSweepBounds3d, RotationalSweepError3d, rotational_sweep_bounds,
};
pub use stabilized_rotating_world::RotatingWorld3d as PhysicsWorld3dKernel;
pub use world::{CollisionEvent, PhysicsError, StepReport, StepStats, World, WorldConfig};
