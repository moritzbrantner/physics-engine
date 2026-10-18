//! Physics engine for the owner's games, measured against Rapier.
//!
//! [`approximate::World`] is the only world. Physical state is `f32`; elapsed time is `f64`.
//! Rendering, game loops and gameplay rules remain consumer-owned.

#![forbid(unsafe_code)]

mod body;
pub mod numeric;
#[doc(hidden)]
pub mod work;

pub use body::{BodyId, CollisionLayers3d};

pub mod approximate;
