//! Reusable deterministic physics simulation primitives.
//!
//! The first engine slice deliberately owns only translational 3D AABB rigid-body semantics:
//! deterministic world state, gravity, fixed/dynamic bodies, continuous collision detection,
//! time-of-impact resolution and restitution. Rendering, ECS storage, game loops and scene ownership
//! belong to consumers.

#![forbid(unsafe_code)]

mod body;
mod collision;
mod math;
mod world;

pub use body::{BodyId, BodyKind, MATERIAL_SCALE, Material, RigidBody};
pub use collision::{
    ContactNormal, SUBTICKS_PER_TICK, SweepHit, TimeOfImpact, overlap_aabb, swept_aabb,
};
pub use math::Vec3i;
pub use world::{CollisionEvent, PhysicsError, StepReport, StepStats, World, WorldConfig};
