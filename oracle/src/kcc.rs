//! Shared character drivers for the parity scenarios. Each scenario keeps its geometry, inputs
//! and checks; the drivers own the consumer patterns both sides repeat:
//!
//! - [`rapier::Characters`]: MMORPG/ARPG's Rapier call pattern. Static colliders, then one
//!   kinematic capsule per character, `detect_collisions`; per tick an optional world rebuild
//!   (the consumers' pattern; without it, bare Rapier) and, per character in id order,
//!   `move_shape` with its own body excluded, `set_translation` and `detect_collisions`.
//! - [`engine::Characters`]: the same characters on physics-engine, one `move_characters`
//!   batch per tick with reused move and result buffers.

use crate::{Obs, Trace};

/// The consumers' fixed tick (60 Hz).
pub const DT: f32 = 1.0 / 60.0;
/// Gravity, m/s².
pub const GRAVITY: f32 = 9.81;
/// Character controller offset (skin), metres.
pub const OFFSET: f32 = 0.0025;
/// Snap-to-ground distance beyond the offset, metres.
pub const SNAP: f32 = 0.05;
/// The consumers' unit capsule: half segment and radius.
pub const HALF: f32 = 0.5;
pub const RADIUS: f32 = 0.3;

/// A trace recorded as `ticks` ticks of `lanes` lanes with `per_lane` observations each,
/// followed by `extra` per-tick observations.
#[derive(Clone, Copy)]
pub struct Lanes<'a> {
    obs: &'a [Obs],
    lanes: usize,
    per_lane: usize,
    per_tick: usize,
}

impl<'a> Lanes<'a> {
    /// `trace` in this layout, or an error when its length does not fit it.
    pub fn new(
        trace: &'a Trace,
        ticks: u32,
        lanes: usize,
        per_lane: usize,
        extra: usize,
    ) -> Result<Self, String> {
        let per_tick = lanes * per_lane + extra;
        if trace.0.len() != ticks as usize * per_tick {
            return Err(format!(
                "trace has {} observations, expected {} ticks of {per_tick}",
                trace.0.len(),
                ticks
            ));
        }
        Ok(Self {
            obs: &trace.0,
            lanes,
            per_lane,
            per_tick,
        })
    }

    /// Observation `k` of `lane` at `tick`.
    pub fn obs(&self, tick: usize, lane: usize, k: usize) -> Obs {
        self.obs[tick * self.per_tick + lane * self.per_lane + k]
    }

    /// Per-tick observation `k` after the lanes at `tick`.
    pub fn extra(&self, tick: usize, k: usize) -> Obs {
        self.obs[tick * self.per_tick + self.lanes * self.per_lane + k]
    }

    /// The position `lane` recorded first at `tick` (NaN if it recorded none).
    pub fn position(&self, tick: usize, lane: usize) -> [f32; 3] {
        match self.obs(tick, lane, 0) {
            Obs::Position(p) => p,
            _ => [f32::NAN; 3],
        }
    }

    /// Whether observation `k` of `lane` at `tick` is `Grounded(true)`.
    pub fn grounded(&self, tick: usize, lane: usize, k: usize) -> bool {
        matches!(self.obs(tick, lane, k), Obs::Grounded(true))
    }
}

/// A trace of one position per lane and tick.
pub fn positions(trace: &Trace, ticks: u32, lanes: usize) -> Result<Lanes<'_>, String> {
    Lanes::new(trace, ticks, lanes, 1, 0)
}

#[cfg(feature = "engine")]
pub mod engine {
    use physics_engine::{
        BodyId,
        approximate::{
            Body, BodyStorage, CharacterMove, CharacterMoveResult, CharacterSettings, Config,
            Shape, Vector, World,
        },
    };

    pub fn v(a: [f32; 3]) -> Vector {
        Vector(a[0], a[1], a[2])
    }

    /// A world without gravity holding `statics`, added in order.
    pub fn world(storage: BodyStorage, statics: impl IntoIterator<Item = Body>) -> World {
        let mut world = World::new(Config {
            body_storage: storage,
            gravity: Vector::ZERO,
            ..Config::default()
        })
        .unwrap();
        for body in statics {
            world.add_body(body).unwrap();
        }
        world
    }

    /// An immovable body keyed `key`.
    pub fn fixed(key: u64, shape: Shape, at: [f32; 3]) -> Body {
        Body::keyed(key, shape, v(at), 0.0)
    }

    /// Externally moved capsules keyed from `first_key`, one per start.
    pub fn capsules(
        first_key: u64,
        shape: Shape,
        starts: impl IntoIterator<Item = [f32; 3]>,
    ) -> impl Iterator<Item = Body> {
        starts.into_iter().enumerate().map(move |(i, p)| {
            let mut body = Body::keyed(first_key + i as u64, shape, v(p), 1.0);
            body.external = true;
            body
        })
    }

    /// Characters in id order, moved in one `move_characters` batch per tick.
    pub struct Characters {
        pub world: World,
        pub ids: Vec<BodyId>,
        pub moves: Vec<CharacterMove>,
        pub results: Vec<CharacterMoveResult>,
    }

    impl Characters {
        /// Adds `bodies` to `world` as characters with `settings`.
        pub fn new(
            world: World,
            bodies: impl IntoIterator<Item = Body>,
            settings: CharacterSettings,
        ) -> Self {
            let mut characters = Self {
                world,
                ids: Vec::new(),
                moves: Vec::new(),
                results: Vec::new(),
            };
            for body in bodies {
                characters.add(body, settings);
            }
            characters
        }

        /// Adds one character.
        pub fn add(&mut self, body: Body, settings: CharacterSettings) -> BodyId {
            let id = self.world.add_body(body).unwrap();
            self.world.add_character(id, settings).unwrap();
            self.ids.push(id);
            self.moves.reserve(1);
            self.results.reserve(1);
            id
        }

        /// Moves every character by `desired(lane)` in one batch.
        pub fn move_all(&mut self, mut desired: impl FnMut(usize) -> Vector) {
            self.move_each(|lane, body| CharacterMove {
                body,
                desired: desired(lane),
                exclude: None,
                autostep: None,
            });
        }

        /// Moves every character by its `CharacterMove` in one batch.
        pub fn move_each(&mut self, mut each: impl FnMut(usize, BodyId) -> CharacterMove) {
            self.moves.clear();
            for (lane, &body) in self.ids.iter().enumerate() {
                self.moves.push(each(lane, body));
            }
            self.world
                .move_characters(&self.moves, &mut self.results)
                .unwrap();
        }

        /// A body's position.
        pub fn position(&self, body: BodyId) -> [f32; 3] {
            let p = self.world.body(body).unwrap().position;
            [p.0, p.1, p.2]
        }
    }
}

#[cfg(feature = "rapier")]
pub mod rapier {
    use super::{DT, OFFSET, SNAP};
    use rapier3d::control::{
        CharacterAutostep, CharacterLength, EffectiveCharacterMovement,
        KinematicCharacterController,
    };
    use rapier3d::parry::query::ShapeCastHit;
    use rapier3d::parry::query::{DefaultQueryDispatcher, QueryDispatcher, ShapeCastOptions};
    use rapier3d::prelude::*;

    pub fn v(a: [f32; 3]) -> Vector {
        Vector::new(a[0], a[1], a[2])
    }

    /// The consumers' controller: `offset`, 45° climb and slide, snapping `snap` beyond the
    /// offset, autostep `(max_height, min_width)` without dynamic bodies.
    pub fn mirror_controller(
        offset: f32,
        snap: Option<f32>,
        autostep: Option<(f32, f32)>,
    ) -> KinematicCharacterController {
        KinematicCharacterController {
            up: Vector::Y,
            offset: CharacterLength::Absolute(offset),
            slide: true,
            autostep: autostep.map(|(height, width)| CharacterAutostep {
                max_height: CharacterLength::Absolute(height),
                min_width: CharacterLength::Absolute(width),
                include_dynamic_bodies: false,
            }),
            max_slope_climb_angle: core::f32::consts::FRAC_PI_4,
            min_slope_slide_angle: core::f32::consts::FRAC_PI_4,
            snap_to_ground: snap.map(|s| CharacterLength::Absolute(offset + s)),
            normal_nudge_factor: KinematicCharacterController::default().normal_nudge_factor,
        }
    }

    /// A static cuboid collider.
    pub fn cuboid((c, h): ([f32; 3], [f32; 3])) -> ColliderBuilder {
        ColliderBuilder::cuboid(h[0], h[1], h[2]).translation(v(c))
    }

    /// The consumers' ground: a halfspace through the origin, facing +Y.
    pub fn halfspace() -> ColliderBuilder {
        ColliderBuilder::new(SharedShape::halfspace(Vector::Y))
    }

    /// The consumers' downward support cast of `shape` from `at`: target distance `OFFSET`,
    /// reach `OFFSET + SNAP`.
    pub fn support_cast(
        queries: &QueryPipeline<'_>,
        shape: &Capsule,
        at: Vector,
    ) -> Option<(ColliderHandle, ShapeCastHit)> {
        queries.cast_shape(
            &Pose::from_translation(at),
            -Vector::Y,
            shape,
            ShapeCastOptions {
                target_distance: OFFSET,
                stop_at_penetration: false,
                max_time_of_impact: OFFSET + SNAP,
                compute_impact_geometry_on_penetration: true,
            },
        )
    }

    /// Whether a support hit is walkable (45°).
    pub fn walkable(hit: &ShapeCastHit) -> bool {
        hit.normal1.dot(Vector::Y) >= core::f32::consts::FRAC_PI_4.cos()
    }

    /// Kinematic capsule characters over static colliders, moved one by one in id order.
    pub struct Characters {
        pub world: PhysicsWorld,
        /// Static colliders inserted before the characters, in order, on every build.
        pub statics: Vec<ColliderBuilder>,
        /// Static colliders inserted after the characters (ARPG's floor).
        pub trailing: Vec<ColliderBuilder>,
        /// Handles of `statics`, then `trailing`, in the current world.
        pub static_handles: Vec<ColliderHandle>,
        pub shape: Capsule,
        pub controller: KinematicCharacterController,
        pub bodies: Vec<RigidBodyHandle>,
        /// Game positions; the source of truth across rebuilds.
        pub positions: Vec<Vector>,
        /// Rebuild the world every tick (MMORPG/ARPG); bare Rapier keeps it.
        pub rebuild: bool,
        pub dispatcher: Box<dyn QueryDispatcher>,
        /// Index into `static_handles` of a static every move ignores (a planar floor).
        pub exclude: Option<usize>,
        /// Keep characters on their plane: a move's vertical translation is dropped.
        pub planar: bool,
        /// The last [`Self::move_all`]'s movements, in id order.
        pub movements: Vec<EffectiveCharacterMovement>,
    }

    impl Characters {
        pub fn new(
            statics: Vec<ColliderBuilder>,
            starts: impl IntoIterator<Item = [f32; 3]>,
            shape: Capsule,
            controller: KinematicCharacterController,
            rebuild: bool,
        ) -> Self {
            let positions: Vec<Vector> = starts.into_iter().map(v).collect();
            let mut characters = Self {
                world: PhysicsWorld::new(),
                statics,
                trailing: Vec::new(),
                static_handles: Vec::new(),
                shape,
                controller,
                bodies: Vec::with_capacity(positions.len()),
                movements: Vec::with_capacity(positions.len()),
                positions,
                rebuild,
                dispatcher: Box::new(DefaultQueryDispatcher),
                exclude: None,
                planar: false,
            };
            characters.build();
            characters
        }

        /// Statics inserted after the characters; rebuilds the world.
        pub fn with_trailing(mut self, trailing: Vec<ColliderBuilder>) -> Self {
            self.trailing = trailing;
            self.build();
            self
        }

        /// A fresh world: statics, characters at their positions, trailing statics, then
        /// `detect_collisions`.
        pub fn build(&mut self) {
            let mut world = PhysicsWorld::new();
            world.gravity = Vector::ZERO;
            world.integration_parameters.dt = DT;
            self.static_handles.clear();
            for collider in &self.statics {
                let handle = world.colliders.insert(collider.clone().build());
                self.static_handles.push(handle);
            }
            self.bodies.clear();
            for &p in &self.positions {
                let body = world.bodies.insert(
                    RigidBodyBuilder::kinematic_position_based()
                        .translation(p)
                        .build(),
                );
                world.colliders.insert_with_parent(
                    ColliderBuilder::new(SharedShape::new(self.shape)).build(),
                    body,
                    &mut world.bodies,
                );
                self.bodies.push(body);
            }
            for collider in &self.trailing {
                let handle = world.colliders.insert(collider.clone().build());
                self.static_handles.push(handle);
            }
            world.detect_collisions(&(), &());
            self.world = world;
        }

        /// The consumers' tick start: a rebuild when they rebuild.
        pub fn start_tick(&mut self) {
            if self.rebuild {
                self.build();
            }
        }

        /// A query pipeline over the current world.
        pub fn queries<'a>(&'a self, filter: QueryFilter<'a>) -> QueryPipeline<'a> {
            let w = &self.world;
            w.broad_phase
                .as_query_pipeline(&*self.dispatcher, &w.bodies, &w.colliders, filter)
        }

        /// The support cast from `at` through static geometry only.
        pub fn support(&self, at: Vector) -> Option<(ColliderHandle, ShapeCastHit)> {
            support_cast(&self.queries(QueryFilter::only_fixed()), &self.shape, at)
        }

        /// `move_shape` of `lane` from its position by `desired`, with `controller`.
        pub fn move_with(
            &self,
            lane: usize,
            controller: &KinematicCharacterController,
            desired: Vector,
        ) -> EffectiveCharacterMovement {
            let mut filter = QueryFilter::default().exclude_rigid_body(self.bodies[lane]);
            if let Some(i) = self.exclude {
                filter = filter.exclude_collider(self.static_handles[i]);
            }
            controller.move_shape(
                DT,
                &self.queries(filter),
                &self.shape,
                &Pose::from_translation(self.positions[lane]),
                desired,
                |_| {},
            )
        }

        /// `move_shape` of `lane` with the driver's controller.
        pub fn move_shape(&self, lane: usize, desired: Vector) -> EffectiveCharacterMovement {
            self.move_with(lane, &self.controller, desired)
        }

        /// Where `movement` takes `lane` (on its plane when `planar`).
        pub fn end(&self, lane: usize, movement: &EffectiveCharacterMovement) -> Vector {
            let t = movement.translation;
            let from = self.positions[lane];
            if self.planar {
                from + Vector::new(t.x, 0.0, t.z)
            } else {
                from + t
            }
        }

        /// Moves `lane`'s body to `to`, then `detect_collisions`.
        pub fn commit(&mut self, lane: usize, to: Vector) {
            self.positions[lane] = to;
            self.world.bodies[self.bodies[lane]].set_translation(to, false);
            self.world.detect_collisions(&(), &());
        }

        /// Starts the tick and moves every character by `desired(lane)`, in id order.
        pub fn move_all(&mut self, mut desired: impl FnMut(usize) -> Vector) {
            self.start_tick();
            self.movements.clear();
            for lane in 0..self.bodies.len() {
                let movement = self.move_shape(lane, desired(lane));
                let to = self.end(lane, &movement);
                self.commit(lane, to);
                self.movements.push(movement);
            }
        }

        /// `lane`'s position as an array.
        pub fn position(&self, lane: usize) -> [f32; 3] {
            let p = self.positions[lane];
            [p.x, p.y, p.z]
        }
    }
}
