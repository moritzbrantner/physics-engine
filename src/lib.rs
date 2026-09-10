//! Reusable deterministic physics simulation primitives.
//!
//! The engine currently owns translational 3D rigid-body semantics: deterministic world state,
//! gravity, fixed/dynamic bodies, continuous collision detection, time-of-impact resolution,
//! restitution, deterministic spatial queries, and physics-native collider geometry. Rendering,
//! ECS storage, game loops and scene ownership belong to consumers.

#![forbid(unsafe_code)]

mod body;
mod collider;
mod collision;
mod math;
mod query;
mod world;

pub use body::{BodyId, BodyKind, MATERIAL_SCALE, Material, RigidBody};
pub use collider::{Collider, ColliderContact, ColliderError, ColliderShape, collider_contact};
pub use collision::{
    ContactNormal, SUBTICKS_PER_TICK, SweepHit, TimeOfImpact, overlap_aabb, swept_aabb,
};
pub use math::Vec3i;
pub use query::{Aabb, QueryError, QueryHit, Ray};
pub use world::{CollisionEvent, PhysicsError, StepReport, StepStats, World, WorldConfig};
