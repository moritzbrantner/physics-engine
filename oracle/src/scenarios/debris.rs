//! C18 slice B (MMORPG tumbling debris, mmorpg#288, #366): 16 breakable props break at once and
//! throw their pieces, MMORPG's `throw_debris` and `step_debris` copied: crates break into six
//! panels, barrels into six staves and two lids, explosive barrels throw theirs harder (14/12
//! units per tick instead of 4/5). Half the breaks stand on the floor plane over removed
//! terrain cells, half on a heightfield hill. The pieces tumble with restitution and linear
//! and angular damping, collide with each other, and settle by the game's rule: from their
//! eighth tick, a piece slower than 0.6 units/tick turning slower than 0.03 rad/tick becomes a
//! fixed box. Every collider uses friction 0.8 and restitution 0.15 on both sides, so Rapier's
//! averaged and the engine's larger/smaller coefficients agree.
//!
//! Rapier rebuilds the world per tick as MMORPG does; the raw variant keeps one world and
//! turns settled pieces into fixed bodies. The engine steps once per tick with sleep off, and
//! replaces a settled piece by a fixed box.

use crate::{Obs, Scenario, Trace};

const TICKS: u32 = 150;
const DT: f32 = 1.0 / 30.0;
const GRAVITY: f32 = 9.0;
/// Metres per game unit, metres per second per unit/tick, rad/s per rad/tick.
const UNIT: f32 = 0.01;
const SPEED: f32 = UNIT * 30.0;
const SPIN: f32 = 30.0;
const FRICTION: f32 = 0.8;
const RESTITUTION: f32 = 0.15;
const DENSITY: f32 = 600.0;
const LINEAR_DAMPING: f32 = 0.2;
const ANGULAR_DAMPING: f32 = 0.8;
/// MMORPG's settle rule: from this age a piece slower than both rest limits settles.
const SETTLE_AFTER: u32 = 8;
const REST_SPEED: f32 = 0.6 * SPEED;
const REST_SPIN: f32 = 0.03 * SPIN;
/// Terrain: N × N vertices, CELL metres apart, a hill on the +X half, flat (removed) cells
/// elsewhere over the floor plane.
const N: usize = 17;
const CELL: f32 = 2.0;
const ORIGIN: f32 = -16.0;
const HILL: [f32; 2] = [8.0, 0.0];
const HILL_RADIUS: f32 = 7.0;
const HILL_HEIGHT: f32 = 1.5;
const BREAKS: usize = 16;
/// Acceptance (owner, #366): at least this share of the rest points lie within
/// `REST_TOLERANCE` of Rapier's, or a quarter of Rapier's throw; every one within `REST_CAP`
/// (measured worst about 1.0 m at 2 substeps × 4 passes). No piece sinks into the ground
/// deeper than Rapier's deepest piece does in the same scenario.
const REST_SHARE: f32 = 0.9;
const REST_TOLERANCE: f32 = 0.5;
const REST_CAP: f32 = 1.5;

#[derive(Clone, Copy, PartialEq)]
enum Kind {
    Crate,
    Barrel,
    ExplosiveBarrel,
}

/// A piece in the prop's frame (units): centre offset, half extents and its own yaw.
type Piece = ([i32; 3], [i32; 3], u16);

const CRATE_PIECES: [Piece; 6] = [
    ([0, 32, 0], [33, 3, 33], 0),
    ([0, -32, 0], [33, 3, 33], 0),
    ([32, 0, 0], [3, 29, 33], 0),
    ([-32, 0, 0], [3, 29, 33], 0),
    ([0, 0, 32], [29, 29, 3], 0),
    ([0, 0, -32], [29, 29, 3], 0),
];
const BARREL_PIECES: [Piece; 8] = [
    ([0, 0, 27], [9, 40, 3], 0),
    ([23, 0, 13], [9, 40, 3], 10_923),
    ([23, 0, -13], [9, 40, 3], 21_845),
    ([0, 0, -27], [9, 40, 3], 32_768),
    ([-23, 0, -13], [9, 40, 3], 43_691),
    ([-23, 0, 13], [9, 40, 3], 54_613),
    ([0, 42, 0], [22, 3, 22], 0),
    ([0, -42, 0], [22, 3, 22], 0),
];

fn kind(b: usize) -> Kind {
    [
        Kind::Crate,
        Kind::Barrel,
        Kind::Crate,
        Kind::ExplosiveBarrel,
    ][b % 4]
}

fn pieces(kind: Kind) -> &'static [Piece] {
    match kind {
        Kind::Crate => &CRATE_PIECES,
        Kind::Barrel | Kind::ExplosiveBarrel => &BARREL_PIECES,
    }
}

fn height_at_vertex(i: usize, j: usize) -> f32 {
    let (x, z) = (ORIGIN + CELL * i as f32, ORIGIN + CELL * j as f32);
    let r2 = (x - HILL[0]).powi(2) + (z - HILL[1]).powi(2);
    (HILL_HEIGHT * (1.0 - r2 / (HILL_RADIUS * HILL_RADIUS))).max(0.0)
}

/// Heights row-major: Z rows, X columns.
fn heights() -> Vec<f32> {
    (0..N * N).map(|k| height_at_vertex(k % N, k / N)).collect()
}

/// Cell (i, j) is removed when it is flat at 0, where the floor plane lies (MMORPG's pattern).
fn removed(i: usize, j: usize) -> bool {
    [(i, j), (i + 1, j), (i, j + 1), (i + 1, j + 1)]
        .into_iter()
        .all(|(a, b)| height_at_vertex(a, b) == 0.0)
}

/// Removed cells row-major, `(N - 1)²`.
#[cfg_attr(not(feature = "engine"), allow(dead_code))]
fn removed_cells() -> Vec<bool> {
    (0..(N - 1) * (N - 1))
        .map(|k| removed(k % (N - 1), k / (N - 1)))
        .collect()
}

/// Ground height (terrain with parry's B–C diagonal, or the floor plane at 0 under removed
/// cells; the clamped terrain is never below 0).
fn ground(x: f32, z: f32) -> f32 {
    let (fx, fz) = ((x - ORIGIN) / CELL, (z - ORIGIN) / CELL);
    if !(0.0..=(N - 1) as f32).contains(&fx) || !(0.0..=(N - 1) as f32).contains(&fz) {
        return 0.0;
    }
    let (i, j) = (
        (fx.floor() as usize).min(N - 2),
        (fz.floor() as usize).min(N - 2),
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

/// A piece's initial state, in metres and seconds.
#[derive(Clone, Copy)]
struct Start {
    position: [f32; 3],
    rotation: [f32; 4],
    velocity: [f32; 3],
    spin: [f32; 3],
    half: [f32; 3],
}

impl Start {
    #[cfg(feature = "engine")]
    fn mass(&self) -> f32 {
        DENSITY * 8.0 * self.half[0] * self.half[1] * self.half[2]
    }
}

fn radians(yaw: u16) -> f32 {
    f32::from(yaw) / 65_536.0 * core::f32::consts::TAU
}

/// MMORPG's `throw_debris` for every break: the prop stands with its lowest footprint point
/// on the ground, faces `facing` and is broken by a player 1.5 m away.
fn starts() -> Vec<Start> {
    let mut out = Vec::new();
    for b in 0..BREAKS {
        let k = b % 8;
        let (x, z) = if b < 8 {
            (-13.0 + 6.0 * (k % 2) as f32, -9.0 + 6.0 * (k / 2) as f32)
        } else {
            (
                HILL[0] - 2.5 + 5.0 * (k % 2) as f32,
                HILL[1] - 4.5 + 3.0 * (k / 2) as f32,
            )
        };
        let kind = kind(b);
        let half_height = if kind == Kind::Crate { 0.35 } else { 0.45 };
        let footprint = (-2..=2)
            .flat_map(|i| (-2..=2).map(move |j| (i as f32 * 0.2, j as f32 * 0.2)))
            .map(|(dx, dz)| ground(x + dx, z + dz))
            .fold(0.0, f32::max);
        let centre = [x, footprint + half_height, z];
        let facing = (b as u16).wrapping_mul(9_973);
        let killer = b as f32 * 2.3;
        let away = [-killer.cos() * 3.0, -killer.sin() * 3.0];
        let (outward, upward) = if kind == Kind::ExplosiveBarrel {
            (14.0, 12.0)
        } else {
            (4.0, 5.0)
        };
        for (index, &(offset, half, yaw)) in pieces(kind).iter().enumerate() {
            let [ox, oy, oz] = offset.map(|axis| axis as f32);
            let (sin, cos) = radians(facing).sin_cos();
            let [px, pz] = [ox * cos + oz * sin, oz * cos - ox * sin];
            let length = px.hypot(pz);
            let (dx, dz) = if length > 0.0 {
                (px / length, pz / length)
            } else {
                (0.0, 0.0)
            };
            let i = index as f32;
            let lift = if oy > 0.0 { 1.4 } else { 1.0 };
            let (s, c) = (radians(facing.wrapping_add(yaw)) / 2.0).sin_cos();
            out.push(Start {
                position: [
                    centre[0] + px * UNIT,
                    centre[1] + oy * UNIT,
                    centre[2] + pz * UNIT,
                ],
                rotation: [0.0, s, 0.0, c],
                velocity: [
                    (dx * outward + away[0]) * SPEED,
                    (upward * lift + (i % 3.0)) * SPEED,
                    (dz * outward + away[1]) * SPEED,
                ],
                spin: [
                    (0.12 * ((i % 3.0) - 1.0) + 0.05) * SPIN,
                    0.08 * ((i % 2.0) * 2.0 - 1.0) * SPIN,
                    0.12 * (((i + 1.0) % 3.0) - 1.0) * SPIN,
                ],
                half: half.map(|axis| axis as f32 * UNIT),
            });
        }
    }
    out
}

/// MMORPG's settle rule for a piece of `age` ticks.
fn settles(age: u32, velocity: [f32; 3], spin: [f32; 3]) -> bool {
    let length = |v: [f32; 3]| (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    age >= SETTLE_AFTER && length(velocity) < REST_SPEED && length(spin) < REST_SPIN
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "debris",
        capabilities: "C18",
        ticks: TICKS,
        #[cfg(feature = "engine")]
        engine: || Box::new(engine::Sim::new()),
        #[cfg(feature = "rapier")]
        rapier: || Box::new(rapier::Sim::new(true)),
        #[cfg(feature = "rapier")]
        rapier_raw: Some(|| Box::new(rapier::Sim::new(false))),
        check,
    }
}

fn rotate(q: [f32; 4], v: [f32; 3]) -> [f32; 3] {
    let [x, y, z, w] = q;
    let u = [x, y, z];
    let cross = |a: [f32; 3], b: [f32; 3]| {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    };
    let t = cross(u, v).map(|c| 2.0 * c);
    let tt = cross(u, t);
    [
        v[0] + w * t[0] + tt[0],
        v[1] + w * t[1] + tt[1],
        v[2] + w * t[2] + tt[2],
    ]
}

/// How far a box sinks into the ground: its deepest corner below the surface, and the deepest
/// terrain point (vertices and points along the cell edges) inside it.
fn sinking(p: [f32; 3], q: [f32; 4], half: [f32; 3]) -> f32 {
    let mut deepest: f32 = 0.0;
    for k in 0..8 {
        let sign = |bit: usize| if k & bit == 0 { -1.0 } else { 1.0 };
        let offset = rotate(q, [sign(1) * half[0], sign(2) * half[1], sign(4) * half[2]]);
        let c = [p[0] + offset[0], p[1] + offset[1], p[2] + offset[2]];
        deepest = deepest.max(ground(c[0], c[2]) - c[1]);
    }
    let inverse = [-q[0], -q[1], -q[2], q[3]];
    let reach = half[0].max(half[1]).max(half[2]) * 1.8;
    let cell = |v: f32| ((v - ORIGIN) / CELL).floor().clamp(0.0, (N - 1) as f32) as usize;
    let vertex = |i: usize, j: usize| {
        [
            ORIGIN + CELL * i as f32,
            height_at_vertex(i, j),
            ORIGIN + CELL * j as f32,
        ]
    };
    for i in cell(p[0] - reach)..=(cell(p[0] + reach) + 1).min(N - 1) {
        for j in cell(p[2] - reach)..=(cell(p[2] + reach) + 1).min(N - 1) {
            let a = vertex(i, j);
            let mut ends = vec![];
            if i + 1 < N {
                ends.push(vertex(i + 1, j));
            }
            if j + 1 < N {
                ends.push(vertex(i, j + 1));
            }
            if i > 0 && j + 1 < N {
                ends.push(vertex(i - 1, j + 1));
            }
            for b in ends {
                if removed(i.min(N - 2), j.min(N - 2)) && a[1] == 0.0 && b[1] == 0.0 {
                    continue;
                }
                for s in 0..=64 {
                    let t = s as f32 / 64.0;
                    let w = [
                        a[0] + (b[0] - a[0]) * t - p[0],
                        a[1] + (b[1] - a[1]) * t - p[1],
                        a[2] + (b[2] - a[2]) * t - p[2],
                    ];
                    let l = rotate(inverse, w);
                    let depth = (0..3)
                        .map(|k| half[k] - l[k].abs())
                        .fold(f32::INFINITY, f32::min);
                    deepest = deepest.max(depth);
                }
            }
        }
    }
    deepest
}

/// Per tick and piece: position, rotation and whether it has settled.
fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    let starts = starts();
    let per_tick = 3 * starts.len();
    if engine.0.len() != rapier.0.len() || engine.0.len() != per_tick * TICKS as usize {
        return Err("trace lengths differ".into());
    }
    let at = |t: &Trace, tick: usize, k: usize| {
        let base = tick * per_tick + 3 * k;
        match (t.0[base], t.0[base + 1], t.0[base + 2]) {
            (Obs::Position(p), Obs::Rotation(q), Obs::Grounded(rest)) => (p, q, rest),
            _ => ([f32::NAN; 3], [f32::NAN; 4], false),
        }
    };
    let deepest = |t: &Trace| {
        let mut worst = (0.0, 0, 0);
        for (k, start) in starts.iter().enumerate() {
            for tick in 0..TICKS as usize {
                let (p, q, _) = at(t, tick, k);
                let sink = sinking(p, q, start.half);
                if sink > worst.0 {
                    worst = (sink, k, tick);
                }
            }
        }
        worst
    };
    let (sink, k, tick) = deepest(engine);
    let (rapier_sink, _, _) = deepest(rapier);
    if sink > rapier_sink {
        return Err(format!(
            "piece {k} sinks {sink:.4} m into the ground at tick {tick}, deeper than Rapier's \
             {rapier_sink:.4} m"
        ));
    }
    let mut near = 0;
    for (k, start) in starts.iter().enumerate() {
        let last = TICKS as usize - 1;
        let (p, _, rest) = at(engine, last, k);
        if !rest {
            return Err(format!("piece {k} still moves at tick {last}: {p:?}"));
        }
        let (r, _, _) = at(rapier, last, k);
        let distance = |a: [f32; 3], b: [f32; 3]| {
            ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
        };
        let throw = distance(r, start.position);
        let off = distance(p, r);
        if off > REST_CAP {
            return Err(format!(
                "piece {k} rests {off:.3} m from Rapier's rest point (throw {throw:.3} m): \
                 {p:?} vs {r:?}"
            ));
        }
        near += usize::from(off <= REST_TOLERANCE.max(0.25 * throw));
    }
    if (near as f32) < REST_SHARE * starts.len() as f32 {
        return Err(format!(
            "{near} of {} pieces rest near Rapier's rest points",
            starts.len()
        ));
    }
    Ok(())
}

#[cfg(feature = "engine")]
mod engine {
    use super::*;
    use physics_engine::{
        BodyId,
        approximate::{Body, Config, HeightfieldDesc, Material, Quaternion, Shape, Vector, World},
    };

    fn v(a: [f32; 3]) -> Vector {
        Vector(a[0], a[1], a[2])
    }

    struct Piece {
        id: BodyId,
        key: u64,
        half: [f32; 3],
        resting: bool,
    }

    pub struct Sim {
        world: World,
        pieces: Vec<Piece>,
    }

    fn material(body: &mut Body) {
        body.friction = FRICTION;
        body.restitution = RESTITUTION;
    }

    impl Sim {
        pub fn new() -> Self {
            let mut world = World::new(Config {
                body_storage: crate::engine_storage(),
                gravity: Vector(0.0, -GRAVITY, 0.0),
                // Two substeps of four passes stop a landing plank's corners well within
                // Rapier's sinking. Slop and sleep are the metre defaults (#318).
                substeps: 2,
                velocity_iterations: 4,
                length_unit: 1.0,
                ..Config::default()
            })
            .unwrap();
            let mut floor = Body::keyed(1, Shape::HalfSpace, Vector::ZERO, 0.0);
            material(&mut floor);
            world.add_body(floor).unwrap();
            let terrain = world
                .add_heightfield(
                    HeightfieldDesc {
                        origin: Vector(ORIGIN, 0.0, ORIGIN),
                        cell_size: (CELL, CELL),
                        columns: N,
                        rows: N,
                        heights: heights(),
                        removed_cells: Some(removed_cells()),
                        material: Material {
                            friction: FRICTION,
                            restitution: RESTITUTION,
                            ..Material::default()
                        },
                    },
                    Some(2),
                )
                .unwrap();
            let _ = terrain;
            let pieces = starts()
                .iter()
                .enumerate()
                .map(|(k, s)| {
                    let key = 16 + k as u64;
                    let mut body = Body::keyed(key, Shape::Box(v(s.half)), v(s.position), s.mass());
                    let [x, y, z, w] = s.rotation;
                    body.orientation = Quaternion(x, y, z, w);
                    body.velocity = v(s.velocity);
                    body.angular_velocity = v(s.spin);
                    body.linear_damping = LINEAR_DAMPING;
                    body.angular_damping = ANGULAR_DAMPING;
                    body.sleep_allowed = false;
                    material(&mut body);
                    Piece {
                        id: world.add_body(body).unwrap(),
                        key,
                        half: s.half,
                        resting: false,
                    }
                })
                .collect();
            Self { world, pieces }
        }
    }

    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            if self.pieces.iter().any(|p| !p.resting) {
                self.world.step(f64::from(DT)).unwrap();
            }
            for piece in &mut self.pieces {
                let body = self.world.body(piece.id).unwrap();
                let (position, orientation) = (body.position, body.orientation);
                if !piece.resting {
                    let w = body.angular_velocity;
                    let u = body.velocity;
                    if settles(tick + 1, [u.0, u.1, u.2], [w.0, w.1, w.2]) {
                        // The game's settle rule: the piece becomes a fixed box.
                        self.world.remove_body(piece.id);
                        let mut fixed =
                            Body::keyed(piece.key, Shape::Box(v(piece.half)), position, 0.0);
                        fixed.orientation = orientation;
                        material(&mut fixed);
                        piece.id = self.world.add_body(fixed).unwrap();
                        piece.resting = true;
                    }
                }
                let q = orientation;
                trace
                    .0
                    .push(Obs::Position([position.0, position.1, position.2]));
                trace.0.push(Obs::Rotation([q.0, q.1, q.2, q.3]));
                trace.0.push(Obs::Grounded(piece.resting));
            }
        }
    }
}

#[cfg(feature = "rapier")]
mod rapier {
    use super::*;
    use rapier3d::parry::shape::{HeightField, HeightFieldCellStatus};
    use rapier3d::parry::utils::Array2;
    use rapier3d::prelude::*;

    fn v(a: [f32; 3]) -> Vector {
        Vector::new(a[0], a[1], a[2])
    }

    struct Piece {
        position: Vector,
        rotation: Rotation,
        velocity: Vector,
        spin: Vector,
        half: [f32; 3],
        resting: bool,
        handle: Option<RigidBodyHandle>,
    }

    pub struct Sim {
        world: PhysicsWorld,
        terrain: SharedShape,
        pieces: Vec<Piece>,
        rebuild: bool,
    }

    fn terrain() -> SharedShape {
        // parry: rows along Z, columns along X, column-major storage.
        let h = heights();
        let data = (0..N * N).map(|k| h[(k % N) * N + k / N]).collect();
        let size = CELL * (N - 1) as f32;
        let mut field = HeightField::new(Array2::new(N, N, data), Vector::new(size, 1.0, size));
        for j in 0..N - 1 {
            for i in 0..N - 1 {
                if removed(i, j) {
                    field.set_cell_status(j, i, HeightFieldCellStatus::CELL_REMOVED);
                }
            }
        }
        SharedShape::new(field)
    }

    fn statics(world: &mut PhysicsWorld, terrain: &SharedShape) {
        world.gravity = Vector::new(0.0, -GRAVITY, 0.0);
        world.integration_parameters.dt = DT;
        world.colliders.insert(
            ColliderBuilder::new(SharedShape::halfspace(Vector::Y))
                .friction(FRICTION)
                .restitution(RESTITUTION)
                .build(),
        );
        let centre = ORIGIN + CELL * (N - 1) as f32 * 0.5;
        world.colliders.insert(
            ColliderBuilder::new(terrain.clone())
                .translation(Vector::new(centre, 0.0, centre))
                .friction(FRICTION)
                .restitution(RESTITUTION)
                .build(),
        );
    }

    /// MMORPG's `step_dynamic` insertion: a settled piece is a fixed collider.
    fn insert(world: &mut PhysicsWorld, p: &Piece) -> Option<RigidBodyHandle> {
        let collider = ColliderBuilder::cuboid(p.half[0], p.half[1], p.half[2])
            .friction(FRICTION)
            .restitution(RESTITUTION)
            .density(DENSITY);
        let pose = Pose::from_parts(p.position, p.rotation);
        if p.resting {
            world.colliders.insert(collider.position(pose).build());
            return None;
        }
        let body = world.bodies.insert(
            RigidBodyBuilder::dynamic()
                .pose(pose)
                .linvel(p.velocity)
                .angvel(p.spin)
                .linear_damping(LINEAR_DAMPING)
                .angular_damping(ANGULAR_DAMPING)
                .build(),
        );
        world
            .colliders
            .insert_with_parent(collider.build(), body, &mut world.bodies);
        Some(body)
    }

    impl Sim {
        pub fn new(rebuild: bool) -> Self {
            let terrain = terrain();
            let mut world = PhysicsWorld::new();
            statics(&mut world, &terrain);
            let mut pieces: Vec<_> = starts()
                .into_iter()
                .map(|s| Piece {
                    position: v(s.position),
                    rotation: Rotation::from_xyzw(
                        s.rotation[0],
                        s.rotation[1],
                        s.rotation[2],
                        s.rotation[3],
                    ),
                    velocity: v(s.velocity),
                    spin: v(s.spin),
                    half: s.half,
                    resting: false,
                    handle: None,
                })
                .collect();
            if !rebuild {
                for p in &mut pieces {
                    p.handle = insert(&mut world, p);
                }
            }
            Self {
                world,
                terrain,
                pieces,
                rebuild,
            }
        }
    }

    impl crate::Sim for Sim {
        fn tick(&mut self, tick: u32, trace: &mut Trace) {
            if self.pieces.iter().any(|p| !p.resting) {
                if self.rebuild {
                    // MMORPG's pattern: a fresh world per tick, settled pieces fixed.
                    let mut world = PhysicsWorld::new();
                    statics(&mut world, &self.terrain);
                    for p in &mut self.pieces {
                        p.handle = insert(&mut world, p);
                    }
                    world.step();
                    self.world = world;
                } else {
                    self.world.step();
                }
            }
            for p in &mut self.pieces {
                if !p.resting
                    && let Some(body) = p.handle.and_then(|h| self.world.bodies.get_mut(h))
                {
                    p.position = body.translation();
                    p.rotation = body.rotation().normalize();
                    p.velocity = body.linvel();
                    p.spin = body.angvel();
                    if settles(tick + 1, p.velocity.to_array(), p.spin.to_array()) {
                        p.resting = true;
                        p.velocity = Vector::ZERO;
                        p.spin = Vector::ZERO;
                        if !self.rebuild {
                            body.set_body_type(RigidBodyType::Fixed, true);
                        }
                    }
                }
                let q = p.rotation;
                trace
                    .0
                    .push(Obs::Position([p.position.x, p.position.y, p.position.z]));
                trace.0.push(Obs::Rotation([q.x, q.y, q.z, q.w]));
                trace.0.push(Obs::Grounded(p.resting));
            }
        }
    }
}
