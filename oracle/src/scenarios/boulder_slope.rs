//! The Redbrand boulder trap (#519, mmorpg#299): a dynamic sphere rolls down heightfield
//! terrain and strikes a real Greyhaven rock-hull proxy at the foot of the slope. The ball
//! is 0.5 m in radius with MMORPG's debris material and density (C18: density 600, friction
//! 0.8, restitution 0.15, linear damping 0.2, angular 0.8), at 60 Hz with 2 substeps × 4
//! passes and sleep off.
//!
//! Both sides use MMORPG's retained response lane (`step_authored`): the terrain and the rock
//! stay in the world, and the ball is added with its state, stepped once and removed each
//! tick. The raw Rapier variant keeps the ball in one world.
use crate::{Obs, Scenario, Trace};

const TICKS: u32 = 600;
const DT: f32 = 1.0 / 60.0;
const GRAVITY: f32 = 9.0;
const RADIUS: f32 = 0.5;
const DENSITY: f32 = 600.0;
const FRICTION: f32 = 0.8;
const RESTITUTION: f32 = 0.15;
const LINEAR_DAMPING: f32 = 0.2;
const ANGULAR_DAMPING: f32 = 0.8;
/// Terrain: COLUMNS × ROWS vertices CELL metres apart from ORIGIN; it falls along +X with
/// GRADE (and gentle ripples across it) until FOOT, then lies flat at 0.
const COLUMNS: usize = 41;
const ROWS: usize = 25;
const CELL: f32 = 1.0;
const ORIGIN: [f32; 2] = [0.0, -12.0];
const GRADE: f32 = 0.35;
const FOOT: f32 = 14.0;
/// The rock proxy's centre (its local frame spans ±1.1 m vertically) beside the ball's line.
const ROCK: [f32; 3] = [19.5, 1.1, 0.7];
const START_X: f32 = 1.5;
/// Acceptance (#519): no deeper than Rapier's deepest contact plus this; every tick's path
/// point and the rest point within the debris oracle's tolerance of Rapier's: 0.5 m or a
/// quarter of Rapier's travel from the start.
#[cfg_attr(not(feature = "rapier"), allow(dead_code))]
const DEPTH_TOLERANCE: f32 = 0.005;
const REST_TOLERANCE: f32 = 0.5;

fn height_at_vertex(i: usize, j: usize) -> f32 {
    let x = ORIGIN[0] + CELL * i as f32;
    let z = ORIGIN[1] + CELL * j as f32;
    let ripple = 0.04 * (1.3 * x).sin() * (0.4 * z).cos();
    if x < FOOT {
        GRADE * (FOOT - x) + ripple
    } else {
        0.0
    }
}

/// Heights row-major: Z rows, X columns.
fn heights() -> Vec<f32> {
    (0..COLUMNS * ROWS)
        .map(|k| height_at_vertex(k % COLUMNS, k / COLUMNS))
        .collect()
}

/// Ground height under (x, z): the terrain with parry's B–C diagonal.
fn ground(x: f32, z: f32) -> f32 {
    let (fx, fz) = ((x - ORIGIN[0]) / CELL, (z - ORIGIN[1]) / CELL);
    if !(0.0..=(COLUMNS - 1) as f32).contains(&fx) || !(0.0..=(ROWS - 1) as f32).contains(&fz) {
        return f32::NEG_INFINITY;
    }
    let (i, j) = (
        (fx.floor() as usize).min(COLUMNS - 2),
        (fz.floor() as usize).min(ROWS - 2),
    );
    let (u, v) = (fx - i as f32, fz - j as f32);
    let (a, b, c, d) = (
        height_at_vertex(i, j),
        height_at_vertex(i + 1, j),
        height_at_vertex(i, j + 1),
        height_at_vertex(i + 1, j + 1),
    );
    if u + v <= 1.0 {
        a + u * (b - a) + v * (c - a)
    } else {
        d + (1.0 - u) * (c - d) + (1.0 - v) * (b - d)
    }
}

fn start() -> [f32; 3] {
    [START_X, ground(START_X, 0.0) + RADIUS + 0.05, 0.0]
}

#[cfg(feature = "engine")]
fn mass() -> f32 {
    DENSITY * 4.0 / 3.0 * core::f32::consts::PI * RADIUS.powi(3)
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "boulder-slope",
        capabilities: "C17, C18",
        ticks: TICKS,
        #[cfg(feature = "engine")]
        engine: |storage| Box::new(engine::Sim::new(storage)),
        #[cfg(feature = "rapier")]
        rapier: || Box::new(rapier::Sim::new(true)),
        #[cfg(feature = "rapier")]
        rapier_raw: Some(|| Box::new(rapier::Sim::new(false))),
        check,
    }
}

/// A tick's ball position and velocity.
type State = ([f32; 3], [f32; 3]);

fn states(trace: &Trace) -> Result<Vec<State>, String> {
    trace
        .0
        .as_chunks::<3>()
        .0
        .iter()
        .map(|o| match (o[0], o[1], o[2]) {
            (Obs::Position(p), Obs::Velocity(v), Obs::Rotation(_)) => Ok((p, v)),
            _ => Err("malformed boulder observation".to_string()),
        })
        .collect()
}

fn distance(a: [f32; 3], b: [f32; 3]) -> f32 {
    a.into_iter()
        .zip(b)
        .map(|(x, y)| (x - y).powi(2))
        .sum::<f32>()
        .sqrt()
}

fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    crate::finite(engine)?;
    crate::finite(rapier)?;
    let expected = TICKS as usize * 3;
    if engine.0.len() != expected || rapier.0.len() != expected {
        return Err("boulder trace has the wrong observation count".into());
    }
    let (ours, theirs) = (states(engine)?, states(rapier)?);
    // Depth below Rapier's deepest contact with terrain or rock, plus the tolerance.
    #[cfg(feature = "rapier")]
    {
        let deepest = |s: &[State]| {
            s.iter()
                .map(|(p, _)| rapier::separation(*p))
                .fold(0.0_f32, f32::min)
        };
        let (reference, depth) = (deepest(&theirs), deepest(&ours));
        if depth < reference - DEPTH_TOLERANCE {
            return Err(format!(
                "ball penetrates terrain or rock by {} m, Rapier by {} m",
                -depth, -reference
            ));
        }
        for (name, s) in [("engine", &ours), ("Rapier", &theirs)] {
            let closest = s
                .iter()
                .map(|(p, _)| rapier::rock_separation(*p))
                .fold(f32::INFINITY, f32::min);
            if closest > 0.02 {
                return Err(format!(
                    "{name} ball never strikes the rock (closest {closest} m)"
                ));
            }
        }
    }
    let begin = start();
    for (name, s) in [("engine", &ours), ("Rapier", &theirs)] {
        for (tick, (p, _)) in s.iter().enumerate() {
            if p[1] - RADIUS < ground(p[0], p[2]) - 0.05 {
                return Err(format!(
                    "{name} ball is under the terrain at tick {tick}: {p:?}"
                ));
            }
            // The rock's -X face is at least 1.4 m from its centre: the ball never passes it.
            if (p[2] - ROCK[2]).abs() < 0.8 && p[0] > ROCK[0] - 1.0 {
                return Err(format!(
                    "{name} ball passes into the rock at tick {tick}: {p:?}"
                ));
            }
        }
        // Rolls downhill: two seconds in, it has run well down the slope.
        let (p, v) = s[120];
        if p[0] < begin[0] + 2.5 || p[1] > begin[1] - 0.8 || v[0] < 1.5 {
            return Err(format!("{name} ball does not roll downhill: {p:?} {v:?}"));
        }
    }
    // Path and rest point near Rapier's.
    for (tick, ((p, _), (r, _))) in ours.iter().zip(&theirs).enumerate() {
        let travel = distance(*r, begin);
        let off = distance(*p, *r);
        if off > REST_TOLERANCE.max(0.25 * travel) {
            return Err(format!(
                "tick {tick}: ball is {off:.3} m from Rapier's (travel {travel:.3} m): \
                 {p:?} vs {r:?}"
            ));
        }
    }
    let ((p, v), (r, w)) = (ours[TICKS as usize - 1], theirs[TICKS as usize - 1]);
    // Stops or deflects like Rapier: the same side of the rock, the same lateral direction
    // and, after the strike, no longer heading into it.
    if (p[2] - ROCK[2]).signum() != (r[2] - ROCK[2]).signum() {
        return Err(format!(
            "ball deflects to the other side of the rock: {p:?} vs {r:?}"
        ));
    }
    if v[0] > 0.5 || w[0] > 0.5 {
        return Err(format!("ball still heads into the rock: {v:?} vs {w:?}"));
    }
    Ok(())
}

#[cfg(feature = "engine")]
mod engine {
    use super::*;
    use physics_engine::approximate::{
        Body, BodyStorage, Config, ConvexHullDesc, HeightfieldDesc, Material, Shape, Vector as V,
        World,
    };
    pub struct Sim {
        world: World,
        ball: Option<Body>,
        statics: usize,
    }
    fn v(p: [f32; 3]) -> V {
        V(p[0], p[1], p[2])
    }
    impl Sim {
        pub fn new(storage: BodyStorage) -> Self {
            let mut world = World::new(Config {
                body_storage: storage,
                gravity: V(0.0, -GRAVITY, 0.0),
                substeps: 2,
                velocity_iterations: 4,
                ..Config::default()
            })
            .unwrap();
            // Fixed geometry keeps the engine's default material, as in MMORPG.
            world
                .add_heightfield(
                    HeightfieldDesc {
                        origin: V(ORIGIN[0], 0.0, ORIGIN[1]),
                        cell_size: (CELL, CELL),
                        columns: COLUMNS,
                        rows: ROWS,
                        heights: heights(),
                        removed_cells: None,
                        material: Material::default(),
                    },
                    Some(1),
                )
                .unwrap();
            world
                .add_convex_hull(
                    ConvexHullDesc {
                        position: v(ROCK),
                        vertices: crate::greyhaven_fixture::vertices()
                            .into_iter()
                            .map(v)
                            .collect(),
                        material: Material::default(),
                    },
                    Some(2),
                )
                .unwrap();
            let mut ball = Body::new(Shape::Sphere(RADIUS), v(start()), mass());
            ball.friction = FRICTION;
            ball.restitution = RESTITUTION;
            ball.linear_damping = LINEAR_DAMPING;
            ball.angular_damping = ANGULAR_DAMPING;
            ball.sleep_allowed = false;
            let statics = world.body_count();
            Self {
                world,
                ball: Some(ball),
                statics,
            }
        }
    }
    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            // MMORPG's retained response lane: static geometry stays, the ball is transient.
            let id = self.world.add_body(self.ball.take().unwrap()).unwrap();
            self.world
                .step(f64::from(DT))
                .unwrap_or_else(|e| panic!("boulder tick {tick}: {e:?}"));
            let body = self.world.remove_body(id).unwrap();
            assert_eq!(self.world.body_count(), self.statics);
            let (p, u, q) = (body.position, body.velocity, body.orientation);
            trace.0.push(Obs::Position([p.0, p.1, p.2]));
            trace.0.push(Obs::Velocity([u.0, u.1, u.2]));
            trace.0.push(Obs::Rotation([q.0, q.1, q.2, q.3]));
            self.ball = Some(body);
        }
    }
}

#[cfg(feature = "rapier")]
mod rapier {
    use super::*;
    use rapier3d::parry::shape::HeightField;
    use rapier3d::parry::utils::Array2;
    use rapier3d::prelude::*;

    fn v(p: [f32; 3]) -> Vector {
        Vector::new(p[0], p[1], p[2])
    }

    fn terrain() -> (Vector, SharedShape) {
        // parry: rows along Z, columns along X, column-major storage, centred on its pose.
        let h = heights();
        let data = (0..COLUMNS * ROWS)
            .map(|k| h[(k % ROWS) * COLUMNS + k / ROWS])
            .collect();
        let size = [CELL * (COLUMNS - 1) as f32, CELL * (ROWS - 1) as f32];
        let field = HeightField::new(
            Array2::new(ROWS, COLUMNS, data),
            Vector::new(size[0], 1.0, size[1]),
        );
        (
            Vector::new(ORIGIN[0] + size[0] * 0.5, 0.0, ORIGIN[1] + size[1] * 0.5),
            SharedShape::new(field),
        )
    }

    fn rock() -> SharedShape {
        SharedShape::convex_hull(
            &crate::greyhaven_fixture::vertices()
                .into_iter()
                .map(v)
                .collect::<Vec<_>>(),
        )
        .unwrap()
    }

    fn signed(target: &Pose, shape: &dyn Shape, p: [f32; 3]) -> f32 {
        rapier3d::parry::query::contact(
            &Pose::from_translation(v(p)),
            &Ball::new(RADIUS),
            target,
            shape,
            1.0,
        )
        .unwrap()
        .map_or(1.0, |c| c.dist)
    }

    /// The ball's separation from the rock.
    pub fn rock_separation(p: [f32; 3]) -> f32 {
        signed(&Pose::from_translation(v(ROCK)), rock().as_ref(), p)
    }

    /// The ball's least separation from the terrain or the rock (negative when overlapping).
    pub fn separation(p: [f32; 3]) -> f32 {
        let (centre, field) = terrain();
        signed(&Pose::from_translation(centre), field.as_ref(), p)
            .min(rock_separation(p))
            .min(0.0)
    }

    pub struct Sim {
        world: PhysicsWorld,
        ball: RigidBodyHandle,
        rebuild: bool,
    }

    fn add_ball(
        world: &mut PhysicsWorld,
        pose: Pose,
        velocity: Vector,
        spin: Vector,
    ) -> RigidBodyHandle {
        let id = world.bodies.insert(
            RigidBodyBuilder::dynamic()
                .pose(pose)
                .linvel(velocity)
                .angvel(spin)
                .linear_damping(LINEAR_DAMPING)
                .angular_damping(ANGULAR_DAMPING)
                .can_sleep(false)
                .build(),
        );
        world.colliders.insert_with_parent(
            ColliderBuilder::ball(RADIUS)
                .density(DENSITY)
                .friction(FRICTION)
                .restitution(RESTITUTION)
                .build(),
            id,
            &mut world.bodies,
        );
        id
    }

    impl Sim {
        pub fn new(rebuild: bool) -> Self {
            let mut world = PhysicsWorld::new();
            world.gravity = Vector::new(0.0, -GRAVITY, 0.0);
            world.integration_parameters.dt = DT;
            let (centre, field) = terrain();
            world
                .colliders
                .insert(ColliderBuilder::new(field).translation(centre).build());
            world
                .colliders
                .insert(ColliderBuilder::new(rock()).translation(v(ROCK)).build());
            let ball = add_ball(
                &mut world,
                Pose::from_translation(v(start())),
                Vector::ZERO,
                Vector::ZERO,
            );
            Self {
                world,
                ball,
                rebuild,
            }
        }
    }

    impl crate::Sim for Sim {
        fn tick(&mut self, _: u32, trace: &mut Trace) {
            if self.rebuild {
                // The retained lane: the ball is re-added from its state each tick.
                let body = &self.world.bodies[self.ball];
                let (pose, velocity, spin) = (*body.position(), body.linvel(), body.angvel());
                self.world.remove_body(self.ball).unwrap();
                self.ball = add_ball(&mut self.world, pose, velocity, spin);
            }
            self.world.step();
            let body = &self.world.bodies[self.ball];
            trace.0.push(Obs::Position(body.translation().to_array()));
            trace.0.push(Obs::Velocity(body.linvel().to_array()));
            trace
                .0
                .push(Obs::Rotation(body.rotation().normalize().to_array()));
        }
    }
}

#[cfg(all(test, feature = "rapier"))]
mod tests {
    use super::*;
    #[test]
    fn rapier_reference_rolls_down_and_strikes_the_rock() {
        let s = scenario();
        let mut trace = Trace::default();
        s.run(&mut rapier::Sim::new(true), &mut trace);
        check(&trace, &trace).unwrap();
    }
}
