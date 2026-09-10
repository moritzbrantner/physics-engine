//! Reusable deterministic physics simulation primitives.
//!
//! The engine owns deterministic translational rigid-body stepping plus engine-local rotational and
//! contact geometry foundations: gravity, fixed/dynamic bodies, continuous collision detection,
//! time-of-impact resolution, restitution, deterministic spatial queries, physics-native collider
//! geometry, fixed-point orientation/angular velocity, box inertia, angular impulse evidence, and
//! exact quantized OBB SAT contact seeds. The current `World` solver remains translational and
//! AABB-only. Rendering, ECS storage, game loops and scene ownership belong to consumers.

#![forbid(unsafe_code)]

mod angular;
mod body;
mod collider;
mod collision;
mod math;
mod oriented_box;
mod query;
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
pub use math::Vec3i;
pub use oriented_box::{
    ObbAxisFeature3d, ObbContactSeed3d, OrientedBox3d, OrientedBoxError3d, obb_contact_seed,
    oriented_box_vertices,
};
pub use query::{Aabb, QueryError, QueryHit, Ray};
pub use world::{CollisionEvent, PhysicsError, StepReport, StepStats, World, WorldConfig};
