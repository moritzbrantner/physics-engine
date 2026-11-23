//! C30 contact impulses for collision damage (#575, moenarch-games/shooter#75): what each
//! contact did over a step, per pair and compound child, at 60 Hz with the shooter's solver
//! settings. The engine side reads `Report::contact_impulses`
//! (`Config::report_contact_impulses`); Rapier sums `ContactData::impulse` over each pair's
//! solver manifolds, one collider per compound part.
//!
//! - **Rest** (lane 0): a 10 kg crate resting on the floor: its weight's share per step.
//! - **Drop** (lane 1): a 10 kg crate hitting the floor at 5 m/s.
//! - **Landing** (lane 2): a three-footed compound with an off-centre deck dropped onto its ball
//!   feet: one contact per foot, each carrying its own share.
//! - **Debris** (lane 3): a 5 kg ball skimming the floor at 8 m/s into a robot's front armour
//!   plate (a ball resting against the plate needs no force from it, where a leaning block's
//!   share is indeterminate).
//! - **Bullet** (lane 4): a 50 g CCD bullet at 150 m/s into the same plate.
//!
//! Observations: per tick, per contact slot (a pair of labels and children), the summed
//! normal impulse with its impulse-weighted normal (from the lower label toward the higher)
//! and point (Rapier's: its witness on the lower label's surface); each lane's other contacts summed into one more slot. The debris and bullet hit
//! a plate standing 28 cm off the chassis, further than they travel in a tick: Rapier's
//! speculative contacts otherwise give part of a plate hit to the chassis behind it.
//!
//! Invariants against Rapier, per slot: the total impulse within 10% (+0.02 N·s; the debris
//! hit is 34.3 N·s against Rapier's 36.6, the inelastic two-body value 33.1); the running
//! total within Rapier's of the tick before and after (10% of the slot's peak, 3% of the
//! total, +0.02 N·s), as the engine's speculative contacts can stop a body one tick earlier;
//! an impact's peak tick within one of Rapier's; at the peak and while both report at least a fifth of
//! the slot's peak, normals within 0.95 and points on the same contact plane within 2 cm; at
//! rest (the last 20 ticks) every tick within 3% (+0.01 N·s).

use crate::{Obs, Scenario, Trace};

const TICKS: u32 = 90;
const DT: f32 = 1.0 / 60.0;
const GRAVITY: f32 = 9.81;
const SPACING: f32 = 4.0;
const FLOOR: u32 = 1;

/// A part of a body: a box (half extents) or a ball (radius), its centre in the body's frame
/// and its mass.
#[derive(Clone, Copy)]
enum Part {
    Box([f32; 3]),
    Ball(f32),
}
struct Spec {
    label: u32,
    parts: Vec<(Part, [f32; 3], f32)>,
    /// The authoring origin's start position, relative to the lane.
    origin: [f32; 3],
    velocity: [f32; 3],
    ccd: bool,
}
impl Spec {
    fn compound(&self) -> bool {
        self.parts.len() > 1
    }
}

/// A contact slot: labels (lower first) and children.
type Slot = (u32, u32, Option<u32>, Option<u32>);
/// One tick of a slot: impulse, normal and point (or their impulse-weighted sums).
type Sample = (f32, [f32; 3], [f32; 3]);

/// A wide, low chassis (child 0) that does not rock when hit, with a front armour plate (child 1)
/// standing 28 cm off it.
fn robot(label: u32) -> Spec {
    Spec {
        label,
        parts: vec![
            (Part::Box([0.4, 0.3, 0.4]), [0.0; 3], 60.0),
            (Part::Box([0.3, 0.25, 0.02]), [0.0, 0.0, 0.7], 4.0),
        ],
        origin: [0.0, 0.301, 0.0],
        velocity: [0.0; 3],
        ccd: false,
    }
}

/// Each lane's bodies and contact slots.
fn lanes() -> Vec<(Vec<Spec>, Vec<Slot>)> {
    let crate_box = |label, y, vy| Spec {
        label,
        parts: vec![(Part::Box([0.5, 0.5, 0.5]), [0.0; 3], 10.0)],
        origin: [0.0, y, 0.0],
        velocity: [0.0, vy, 0.0],
        ccd: false,
    };
    vec![
        (
            vec![crate_box(10, 0.501, 0.0)],
            vec![(FLOOR, 10, None, None)],
        ),
        (
            vec![crate_box(20, 0.8, -5.0)],
            vec![(FLOOR, 20, None, None)],
        ),
        (
            vec![Spec {
                label: 30,
                parts: vec![
                    (Part::Ball(0.1), [-0.5, 0.0, -0.3], 2.0),
                    (Part::Ball(0.1), [0.5, 0.0, -0.3], 2.0),
                    (Part::Ball(0.1), [0.0, 0.0, 0.4], 2.0),
                    // Off-centre: the feet carry different loads.
                    (Part::Box([0.6, 0.05, 0.45]), [0.15, 0.15, 0.0], 6.0),
                ],
                origin: [0.0, 0.4, 0.0],
                velocity: [0.0; 3],
                ccd: false,
            }],
            vec![
                (FLOOR, 30, None, Some(0)),
                (FLOOR, 30, None, Some(1)),
                (FLOOR, 30, None, Some(2)),
            ],
        ),
        (
            vec![
                robot(40),
                Spec {
                    label: 41,
                    parts: vec![(Part::Ball(0.15), [0.0; 3], 5.0)],
                    origin: [0.0, 0.16, 1.5],
                    velocity: [0.0, 0.0, -8.0],
                    ccd: false,
                },
            ],
            vec![
                (FLOOR, 40, None, Some(0)),
                (40, 41, Some(1), None),
                (40, 41, Some(0), None),
                (FLOOR, 41, None, None),
            ],
        ),
        (
            vec![
                robot(50),
                Spec {
                    label: 51,
                    parts: vec![(Part::Ball(0.02), [0.0; 3], 0.05)],
                    origin: [0.0, 0.5, 3.0],
                    velocity: [0.0, 0.0, -150.0],
                    ccd: true,
                },
            ],
            vec![(FLOOR, 50, None, Some(0)), (50, 51, Some(1), None)],
        ),
    ]
}

/// The lane a label belongs to (the floor belongs to none).
fn lane_of(label: u32) -> usize {
    label as usize / 10 - 1
}

/// One step's contacts, as (slot, impulse, normal from the lower label, point), into one
/// observation per slot: each lane's slots, then its other contacts.
struct Sums {
    slots: Vec<Vec<Slot>>,
    /// Per lane and slot (the last: other): impulse, weighted normal, weighted point.
    sums: Vec<Vec<Sample>>,
}
impl Sums {
    fn new() -> Self {
        let slots: Vec<Vec<Slot>> = lanes().into_iter().map(|(_, s)| s).collect();
        let sums = slots
            .iter()
            .map(|s| vec![Default::default(); s.len() + 1])
            .collect();
        Self { slots, sums }
    }
    fn add(&mut self, slot: Slot, impulse: f32, normal: [f32; 3], point: [f32; 3]) {
        let label = if slot.0 == FLOOR { slot.1 } else { slot.0 };
        let lane = lane_of(label);
        let k = self.slots[lane]
            .iter()
            .position(|s| *s == slot)
            .unwrap_or(self.slots[lane].len());
        let s = &mut self.sums[lane][k];
        s.0 += impulse;
        for i in 0..3 {
            s.1[i] += normal[i] * impulse;
            s.2[i] += point[i] * impulse;
        }
    }
    fn flush(&mut self, trace: &mut Trace) {
        for lane in &mut self.sums {
            for s in lane.iter_mut() {
                let (impulse, n, p) = std::mem::take(s);
                let (normal, point) = if impulse > 0.0 {
                    let l = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt().max(1e-12);
                    (n.map(|x| x / l), p.map(|x| x / impulse))
                } else {
                    ([0.0; 3], [0.0; 3])
                };
                trace.0.push(Obs::Impulse {
                    impulse,
                    normal,
                    point,
                });
            }
        }
    }
}

pub fn scenario() -> Scenario {
    Scenario {
        name: "collision-damage",
        capabilities: "C30",
        ticks: TICKS,
        #[cfg(feature = "engine")]
        engine: |storage| Box::new(engine::Sim::new(storage)),
        #[cfg(feature = "rapier")]
        rapier: || Box::new(rapier::Sim::new()),
        #[cfg(feature = "rapier")]
        rapier_raw: None,
        #[cfg(feature = "rapier")]
        bare_gated: true,
        check,
    }
}

fn check(engine: &Trace, rapier: &Trace) -> Result<(), String> {
    let names: Vec<(usize, String)> = lanes()
        .iter()
        .enumerate()
        .flat_map(|(lane, (_, slots))| {
            slots
                .iter()
                .map(|s| format!("{s:?}"))
                .chain(["other".to_string()])
                .map(move |name| (lane, name))
                .collect::<Vec<_>>()
        })
        .collect();
    let count = names.len();
    let len = count * TICKS as usize;
    if engine.0.len() != len || rapier.0.len() != len {
        return Err(format!(
            "trace lengths differ: {} / {} / {len}",
            engine.0.len(),
            rapier.0.len()
        ));
    }
    let series = |trace: &Trace, k: usize| -> Vec<Sample> {
        (0..TICKS as usize)
            .map(|t| match trace.0[t * count + k] {
                Obs::Impulse {
                    impulse,
                    normal,
                    point,
                } => (impulse, normal, point),
                _ => (f32::NAN, [0.0; 3], [0.0; 3]),
            })
            .collect()
    };
    for (k, (lane, name)) in names.iter().enumerate() {
        let (e, r) = (series(engine, k), series(rapier, k));
        let at = |tick: usize| format!("lane {lane} slot {name} tick {tick}");
        let total = |s: &[Sample]| s.iter().map(|x| x.0).sum::<f32>();
        let (te, tr) = (total(&e), total(&r));
        if (te - tr).abs() > 0.1 * tr + 0.02 {
            return Err(format!(
                "lane {lane} slot {name}: total impulse {te:.4} N·s, Rapier {tr:.4}"
            ));
        }
        let single = r.iter().map(|x| x.0).fold(0.0, f32::max);
        // Running totals: the engine's lies within Rapier's of the tick before and after (its
        // speculative contacts can stop a body at the surface one tick before Rapier's do).
        let running = |s: &[Sample]| {
            s.iter()
                .scan(0.0, |sum, x| {
                    *sum += x.0;
                    Some(*sum)
                })
                .collect::<Vec<f32>>()
        };
        let (ce, cr) = (running(&e), running(&r));
        let peak = |s: &[Sample]| {
            (0..s.len()).fold(0, |best, t| if s[t].0 > s[best].0 { t } else { best })
        };
        let (pe, pr) = (peak(&e), peak(&r));
        // An impact slot: unloaded at the start (a support's largest step is not an impact).
        let impact = r[..3].iter().all(|x| x.0 == 0.0);
        if single > 0.0 && impact {
            let ((_, ne, xe), (_, nr, xr)) = (e[pe], r[pr]);
            if pe.abs_diff(pr) > 1 || !same_plane((ne, xe), (nr, xr)) {
                return Err(format!(
                    "lane {lane} slot {name}: peak at tick {pe} along {ne:?} at {xe:?}, \
                     Rapier tick {pr} along {nr:?} at {xr:?}"
                ));
            }
        }
        for t in 0..e.len() {
            let tolerance = 0.1 * single + 0.03 * cr[t] + 0.02;
            let lo = if t == 0 { 0.0 } else { cr[t - 1] };
            let hi = cr[(t + 1).min(cr.len() - 1)];
            if ce[t] < lo - tolerance || ce[t] > hi + tolerance {
                return Err(format!(
                    "{}: {:.4} N·s so far, Rapier {lo:.4}..{hi:.4} over the neighbouring ticks",
                    at(t),
                    ce[t]
                ));
            }
            let ((ie, ne, xe), (ir, nr, xr)) = (e[t], r[t]);
            if single > 0.0
                && ie >= 0.2 * single
                && ir >= 0.2 * single
                && !same_plane((ne, xe), (nr, xr))
            {
                return Err(format!(
                    "{}: normal {ne:?} at {xe:?}, Rapier {nr:?} at {xr:?}",
                    at(t)
                ));
            }
            if t + 20 >= e.len() && (ie - ir).abs() > 0.03 * ir + 0.01 {
                return Err(format!(
                    "{}: resting impulse {ie:.4} N·s, Rapier {ir:.4}",
                    at(t)
                ));
            }
        }
    }
    Ok(())
}

fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
/// Normals within 0.95 and points on the same contact plane within 2 cm. A resting face's load
/// point is indeterminate within its footprint, so the points are compared along the normal.
fn same_plane((ne, xe): ([f32; 3], [f32; 3]), (nr, xr): ([f32; 3], [f32; 3])) -> bool {
    dot(ne, nr) >= 0.95 && dot([xe[0] - xr[0], xe[1] - xr[1], xe[2] - xr[2]], nr).abs() <= 0.02
}

#[cfg(feature = "engine")]
mod engine {
    use super::*;
    use physics_engine::{
        BodyId,
        approximate::{
            Body, BodyStorage, CompoundChild, Config, PreparedCompound, Quaternion, Shape, Vector,
            World,
        },
    };
    use std::collections::HashMap;

    use crate::kcc::engine::v;

    pub struct Sim {
        world: World,
        labels: HashMap<BodyId, u32>,
        sums: Sums,
    }

    fn shape(part: Part) -> Shape {
        match part {
            Part::Box(h) => Shape::Box(v(h)),
            Part::Ball(r) => Shape::Sphere(r),
        }
    }

    impl Sim {
        pub fn new(storage: BodyStorage) -> Self {
            let mut world = World::new(Config {
                body_storage: storage,
                gravity: Vector(0.0, -GRAVITY, 0.0),
                substeps: 2,
                velocity_iterations: 4,
                report_contact_impulses: true,
                ..Config::default()
            })
            .unwrap();
            let mut labels = HashMap::new();
            let floor = world
                .add_body(Body::keyed(
                    FLOOR.into(),
                    Shape::HalfSpace,
                    Vector::ZERO,
                    0.0,
                ))
                .unwrap();
            labels.insert(floor, FLOOR);
            for (lane, (specs, _)) in lanes().into_iter().enumerate() {
                let x = lane as f32 * SPACING;
                for spec in specs {
                    let origin = v(spec.origin) + Vector(x, 0.0, 0.0);
                    let id = if spec.compound() {
                        let children: Vec<_> = spec
                            .parts
                            .iter()
                            .map(|&(part, at, mass)| CompoundChild::new(shape(part), v(at), mass))
                            .collect();
                        let c = PreparedCompound::new(&children).unwrap();
                        let (position, orientation) = c.body_pose(origin, Quaternion::IDENTITY);
                        let mut body =
                            Body::keyed(spec.label.into(), Shape::Sphere(1.0), position, 1.0);
                        body.orientation = orientation;
                        body.velocity = v(spec.velocity);
                        body.sleep_allowed = false;
                        world.add_compound(&c, body).unwrap()
                    } else {
                        let (part, _, mass) = spec.parts[0];
                        let mut body = Body::keyed(spec.label.into(), shape(part), origin, mass);
                        body.velocity = v(spec.velocity);
                        body.ccd = spec.ccd;
                        body.sleep_allowed = false;
                        world.add_body(body).unwrap()
                    };
                    labels.insert(id, spec.label);
                }
            }
            Self {
                world,
                labels,
                sums: Sums::new(),
            }
        }
    }

    impl crate::Sim for Sim {
        fn tick(&mut self, _tick: u32, trace: &mut Trace) {
            let report = self.world.step(f64::from(DT)).unwrap();
            for c in &report.contact_impulses {
                let (a, b) = (self.labels[&c.a], self.labels[&c.b]);
                let n = [c.normal.0, c.normal.1, c.normal.2];
                let p = [c.point.0, c.point.1, c.point.2];
                self.sums
                    .add((a, b, c.child_a, c.child_b), c.normal_impulse, n, p);
            }
            self.sums.flush(trace);
        }
    }
}

#[cfg(feature = "rapier")]
mod rapier {
    use super::*;
    use rapier3d::prelude::*;
    use std::collections::HashMap;

    use crate::kcc::rapier::v;

    pub struct Sim {
        world: PhysicsWorld,
        /// Each collider's label and, for a compound part, its index.
        parts: HashMap<ColliderHandle, (u32, Option<u32>)>,
        sums: Sums,
    }

    impl Sim {
        pub fn new() -> Self {
            let mut world = PhysicsWorld::new();
            world.gravity = Vector::new(0.0, -GRAVITY, 0.0);
            world.integration_parameters.dt = DT;
            let mut parts = HashMap::new();
            let floor = world
                .colliders
                .insert(ColliderBuilder::new(SharedShape::halfspace(Vector::Y)).build());
            parts.insert(floor, (FLOOR, None));
            for (lane, (specs, _)) in lanes().into_iter().enumerate() {
                let x = lane as f32 * SPACING;
                for spec in specs {
                    let handle = world.bodies.insert(
                        RigidBodyBuilder::dynamic()
                            .translation(v(spec.origin) + Vector::new(x, 0.0, 0.0))
                            .linvel(v(spec.velocity))
                            .ccd_enabled(spec.ccd)
                            .can_sleep(false)
                            .build(),
                    );
                    for (k, &(part, at, mass)) in spec.parts.iter().enumerate() {
                        let builder = match part {
                            Part::Box(h) => ColliderBuilder::cuboid(h[0], h[1], h[2]),
                            Part::Ball(r) => ColliderBuilder::ball(r),
                        };
                        let collider = world.colliders.insert_with_parent(
                            builder.translation(v(at)).mass(mass).build(),
                            handle,
                            &mut world.bodies,
                        );
                        let child = spec.compound().then_some(k as u32);
                        parts.insert(collider, (spec.label, child));
                    }
                }
            }
            Self {
                world,
                parts,
                sums: Sums::new(),
            }
        }
    }

    impl crate::Sim for Sim {
        fn tick(&mut self, _tick: u32, trace: &mut Trace) {
            // The narrow phase generates the step's contacts at the poses it starts from.
            let poses: HashMap<ColliderHandle, Pose> = self
                .parts
                .keys()
                .map(|&c| (c, *self.world.colliders[c].position()))
                .collect();
            self.world.step();
            for pair in self.world.narrow_phase.contact_pairs() {
                let (mut first, mut second) =
                    (self.parts[&pair.collider1], self.parts[&pair.collider2]);
                // The normal points from collider 1 toward collider 2; slots from the lower label.
                let flip = first.0 > second.0;
                if flip {
                    std::mem::swap(&mut first, &mut second);
                }
                let (c1, c2) = (poses[&pair.collider1], poses[&pair.collider2]);
                for m in pair.solver_manifolds() {
                    let n = if flip { -m.data.normal } else { m.data.normal };
                    for point in &m.points {
                        let impulse = point.data.impulse;
                        if impulse <= 0.0 {
                            continue;
                        }
                        // The witness on the lower label's surface: Rapier lets a fast body sink
                        // up to 10 cm before its contact acts, which moves a midpoint.
                        let p = if flip {
                            c2.transform_point(point.local_p2)
                        } else {
                            c1.transform_point(point.local_p1)
                        };
                        self.sums.add(
                            (first.0, second.0, first.1, second.1),
                            impulse,
                            [n.x, n.y, n.z],
                            [p.x, p.y, p.z],
                        );
                    }
                }
            }
            self.sums.flush(trace);
        }
    }
}
