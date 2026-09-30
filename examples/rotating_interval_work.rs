//! Complete engine-call comparison; initialization and parity checks are outside timed regions.
use std::{hint::black_box, time::Instant};

use physics_engine::{
    AngularState3d, AngularVelocity3d, BodyId, Material, Orientation3d, RigidBody, RigidBox3d,
    RotatingIntervalConfig3d, RotatingIntervalWork3d, RotatingWorld3d, RotatingWorldConfig3d,
    RotatingWorldStepReport3d, Vec3i,
};

const CALLS: u32 = 256;
const CONFIG: RotatingIntervalConfig3d = RotatingIntervalConfig3d {
    timestep_numerator: 1,
    timestep_denominator: 60,
    substeps: 4,
    angular_damping_milli: 1000,
};

fn rigid(body: RigidBody) -> RigidBox3d {
    RigidBox3d::new(
        body,
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .unwrap()
}

fn scene(name: &str) -> RotatingWorld3d {
    let mut world = RotatingWorld3d::new(RotatingWorldConfig3d {
        gravity: Vec3i::ZERO,
        ..Default::default()
    });
    if name == "contacts-32" {
        let half = Vec3i::new(1024, 1024, 1024);
        for island in 0..32 {
            let y = island * 10_000;
            let first = u64::try_from(island).unwrap() * 3 + 1;
            for body in [
                RigidBody::dynamic(
                    BodyId(first),
                    Vec3i::new(0, y, 0),
                    Vec3i::new(20_000, 0, 0),
                    half,
                ),
                RigidBody::fixed(BodyId(first + 1), Vec3i::new(4096, y, 0), half),
                RigidBody::fixed(BodyId(first + 2), Vec3i::new(-4096, y, 0), half),
            ] {
                world
                    .add_box(rigid(body.with_material(Material::new(1000))).with_rotation_locked())
                    .unwrap();
            }
        }
    } else {
        let (count, active) = match name {
            "quiet-128" => (128, 0),
            "active-4" => (4, 4),
            "sparse-512" => (512, 4),
            "active-512" => (512, 512),
            _ => unreachable!(),
        };
        for index in 0..count {
            let id = BodyId(u64::try_from(index).unwrap() + 1);
            let position = Vec3i::new(index * 10_000, 0, 0);
            let half = Vec3i::new(100, 100, 100);
            let body = if index < active || name == "quiet-128" {
                RigidBody::dynamic(
                    id,
                    position,
                    Vec3i::new(if index < active { 600 } else { 0 }, 0, 0),
                    half,
                )
            } else {
                RigidBody::fixed(id, position, half)
            };
            world.add_box(rigid(body)).unwrap();
        }
        if name == "quiet-128" {
            world.step(1, 1).unwrap();
        }
    }
    world
}

fn commands(world: &mut RotatingWorld3d, reports: &mut Vec<RotatingWorldStepReport3d>) {
    reports.clear();
    for _ in 0..CONFIG.substeps {
        reports.push(world.step(1, 240).unwrap());
    }
}

fn timed(
    world: &mut RotatingWorld3d,
    reports: &mut Vec<RotatingWorldStepReport3d>,
    atomic: bool,
) -> (u128, RotatingIntervalWork3d) {
    let mut work = RotatingIntervalWork3d::default();
    let started = Instant::now();
    for _ in 0..CALLS {
        if atomic {
            work = black_box(world.advance_interval(CONFIG, reports).unwrap());
        } else {
            commands(world, reports);
        }
        black_box(&*reports);
    }
    (started.elapsed().as_nanos(), work)
}

fn main() {
    for name in [
        "quiet-128",
        "active-4",
        "sparse-512",
        "active-512",
        "contacts-32",
    ] {
        for trial in 0..3 {
            let mut control = scene(name);
            let mut atomic = control.clone();
            let mut control_reports = Vec::new();
            let mut atomic_reports = Vec::new();
            for _ in 0..64 {
                commands(&mut control, &mut control_reports);
                atomic
                    .advance_interval(CONFIG, &mut atomic_reports)
                    .unwrap();
            }
            let (control_ns, atomic_ns, work) = if trial % 2 == 0 {
                let (control_ns, _) = timed(&mut control, &mut control_reports, false);
                let (atomic_ns, work) = timed(&mut atomic, &mut atomic_reports, true);
                (control_ns, atomic_ns, work)
            } else {
                let (atomic_ns, work) = timed(&mut atomic, &mut atomic_reports, true);
                let (control_ns, _) = timed(&mut control, &mut control_reports, false);
                (control_ns, atomic_ns, work)
            };
            assert_eq!(
                atomic.boxes().collect::<Vec<_>>(),
                control.boxes().collect::<Vec<_>>()
            );
            for body in atomic.boxes() {
                assert_eq!(
                    atomic.is_sleeping(body.body().id()),
                    control.is_sleeping(body.body().id())
                );
            }
            assert_eq!(atomic_reports, control_reports);
            println!(
                "{{\"scene\":\"{name}\",\"trial\":{trial},\"calls\":{CALLS},\"control_ns\":{control_ns},\"atomic_ns\":{atomic_ns},\"motion_entries\":{},\"sleep_entries\":{},\"parked_entries\":{},\"contact_work\":{:?},\"parity\":true}}",
                work.motion_before_images,
                work.sleep_before_images,
                work.parked_before_images,
                work.contact_work
            );
        }
    }
}
