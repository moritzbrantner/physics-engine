//! Successful whole-step cost/trace controls usable unchanged on the prior API.
use physics_engine::approximate::Real;
use std::{
    fs::{self, File},
    io::{BufWriter, Write},
    time::Instant,
};

use physics_engine::{
    BodyId,
    approximate::{Body, Config, Shape, Vector as V, World},
};

fn supported(count: u64) -> World {
    let mut world = World::new(Config {
        gravity: V(0.0, -10.0, 0.0),
        substeps: 2,
        sleep_speed: 0.1,
        sleep_seconds: 0.05,
        ..Config::default()
    })
    .unwrap();
    world
        .add_body(Body::new(
            BodyId(0),
            Shape::Box(V(2_000.0, 0.1, 10.0)),
            V(0.0, -0.1, 0.0),
            0.0,
        ))
        .unwrap();
    for index in 0..count {
        world
            .add_body(Body::new(
                BodyId(index + 1),
                Shape::Box(V(0.5, 0.5, 0.5)),
                V(index as Real * 3.0, 0.5, 0.0),
                1.0,
            ))
            .unwrap();
    }
    for _ in 0..100 {
        world.step(0.01).unwrap();
    }
    assert!(world.is_quiescent());
    world
}

fn fixture(name: &str, count: u64) -> World {
    if name != "free" && name != "stack" {
        return supported(count);
    }
    let mut world = World::new(Config {
        gravity: if name == "stack" {
            V(0.0, -10.0, 0.0)
        } else {
            V::ZERO
        },
        substeps: 4,
        fixed_position_iterations: 2,
        ..Config::default()
    })
    .unwrap();
    if name == "stack" {
        world
            .add_body(Body::new(
                BodyId(0),
                Shape::Box(V(50.0, 0.1, 50.0)),
                V(0.0, -0.1, 0.0),
                0.0,
            ))
            .unwrap();
    }
    for index in 0..count {
        let position = if name == "stack" {
            V(
                (index % 4) as Real * 1.01,
                ((index / 4) % 4) as Real * 1.01 + 0.5,
                (index / 16) as Real * 1.01,
            )
        } else {
            V(index as Real * 3.0, 10.0, 0.0)
        };
        let mut body = Body::new(
            BodyId(index + 1),
            Shape::Box(V(0.5, 0.5, 0.5)),
            position,
            1.0,
        );
        body.sleep_allowed = false;
        if name == "free" {
            body.velocity = V(0.1, 0.0, 0.0);
        }
        world.add_body(body).unwrap();
    }
    world
}

#[test]
#[ignore = "advisory whole-call before/after transaction matrix"]
fn successful_floating_step_matrix() {
    let trace_directory = std::env::var_os("PHYSICS_TRANSACTION_TRACE_DIR");
    if let Some(directory) = &trace_directory {
        fs::create_dir_all(directory).unwrap();
    }
    for (name, count) in [
        ("free", 128),
        ("free", 512),
        ("quiet", 128),
        ("quiet", 512),
        ("local", 128),
        ("local", 512),
        ("stack", 48),
        ("impact", 48),
    ] {
        for trial in 0..3 {
            let mut world = fixture(name, count);
            if name == "impact" {
                let mut projectile =
                    Body::new(BodyId(10_000), Shape::Sphere(0.2), V(-4.0, 0.5, 0.0), 0.5);
                projectile.velocity = V(40.0, 0.0, 0.0);
                projectile.ccd = true;
                projectile.retire_on_impact = true;
                world.add_body(projectile).unwrap();
            }
            let mut trace = trace_directory.as_ref().map(|directory| {
                BufWriter::new(
                    File::create(
                        std::path::Path::new(directory).join(format!("{name}-{count}-{trial}.bin")),
                    )
                    .unwrap(),
                )
            });
            let mut milliseconds = 0.0;
            let mut checksum = 0xcbf2_9ce4_8422_2325_u64;
            let mut contacts = 0;
            let mut retired = 0;
            for tick in 0..120 {
                if name == "local" {
                    let speed = if tick % 2 == 0 { 0.2 } else { -0.2 };
                    world.set_velocity(BodyId(1), V(speed, 0.0, 0.0)).unwrap();
                }
                let start = Instant::now();
                let report = world.step(0.01).unwrap();
                milliseconds += start.elapsed().as_secs_f64() * 1_000.0;
                contacts += report.contact_points;
                retired += report.retired.len();
                // Exact round-trip float Debug text, body/private physical fields,
                // ordering, time and semantic report values; no scratch capacities.
                let state = format!(
                    "{:?}|{:?}|{:?}",
                    world.bodies().collect::<Vec<_>>(),
                    world.elapsed_seconds(),
                    (
                        report.substeps,
                        report.contact_points,
                        report.woken_bodies,
                        report.swept_contacts,
                        report.integrated_bodies,
                        report.retired
                    )
                );
                let length = (state.len() as u64).to_be_bytes();
                for byte in length.iter().chain(state.as_bytes()) {
                    checksum ^= u64::from(*byte);
                    checksum = checksum.wrapping_mul(0x0000_0100_0000_01b3);
                }
                if let Some(trace) = &mut trace {
                    trace.write_all(&length).unwrap();
                    trace.write_all(state.as_bytes()).unwrap();
                }
            }
            if let Some(trace) = &mut trace {
                trace.flush().unwrap();
            }
            println!(
                "FLOATING_TRANSACTION_MATRIX {{\"mode\":\"{name}\",\"bodies\":{count},\"trial\":{trial},\"completed_ticks\":120,\"whole_step_ms\":{milliseconds},\"contact_points\":{contacts},\"retired_bodies\":{retired},\"trace_fnv1a64\":\"{checksum:016x}\"}}"
            );
        }
    }
}
