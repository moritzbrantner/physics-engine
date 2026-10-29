//! One character move (or a few in a row) on Rapier and physics-engine from the same pose.
//!
//! ```text
//! cargo run --release -p physics-oracle --bin probe -- \
//!     --floor --box 9.8,0.1025,0,0.2,0.1025,0.8 --autostep 0.25,0.15 \
//!     --at 10.2855,0.8126,0 --desired -0.0333,-0.0027,0 [--repeat 10]
//! ```
//!
//! Geometry: `--floor` (y = 0), `--box cx,cy,cz,hx,hy,hz`, `--wedge cx,cy,cz,hx,hy,hz` (rises
//! toward -X), each repeatable. Character: `--capsule half,radius` (0.5,0.3), `--offset`
//! (0.0025), `--slope degrees` (45), `--snap metres|none` (0.05), `--autostep height,width`,
//! `--stay` (the engine leaves an embedded start in place, as Rapier does).
//! `--repeat N` continues each side from its own result with the same desired motion.

use physics_oracle::probe::{Controller, Geometry, engine, rapier};
use std::process::ExitCode;

fn numbers<const N: usize>(text: &str) -> Result<[f32; N], String> {
    let values: Vec<f32> = text
        .split(',')
        .map(|v| v.trim().parse::<f32>().map_err(|e| format!("{text}: {e}")))
        .collect::<Result<_, _>>()?;
    values
        .try_into()
        .map_err(|_| format!("{text}: expected {N} comma-separated numbers"))
}

/// Geometry, controller, start, desired motion and number of moves.
type Probe = (Geometry, Controller, [f32; 3], [f32; 3], usize);

fn parse() -> Result<Probe, String> {
    let mut geometry = Geometry::default();
    let mut c = Controller::default();
    let (mut at, mut desired, mut repeat) = (None, None, 1);
    let mut args = std::env::args().skip(1);
    let value = |args: &mut dyn Iterator<Item = String>, flag: &str| {
        args.next().ok_or(format!("{flag} needs a value"))
    };
    while let Some(flag) = args.next() {
        match flag.as_str() {
            "--floor" => geometry.floor = true,
            "--box" | "--wedge" => {
                let [cx, cy, cz, hx, hy, hz] = numbers::<6>(&value(&mut args, &flag)?)?;
                let shape = ([cx, cy, cz], [hx, hy, hz]);
                if flag == "--box" {
                    geometry.boxes.push(shape);
                } else {
                    geometry.wedges.push(shape);
                }
            }
            "--capsule" => {
                [c.half_segment, c.radius] = numbers::<2>(&value(&mut args, &flag)?)?;
            }
            "--offset" => [c.offset] = numbers::<1>(&value(&mut args, &flag)?)?,
            "--slope" => {
                let [degrees] = numbers::<1>(&value(&mut args, &flag)?)?;
                c.max_slope = degrees.to_radians();
            }
            "--snap" => {
                let v = value(&mut args, &flag)?;
                c.snap = if v == "none" {
                    None
                } else {
                    Some(numbers::<1>(&v)?[0])
                };
            }
            "--autostep" => {
                let [h, w] = numbers::<2>(&value(&mut args, &flag)?)?;
                c.autostep = Some((h, w));
            }
            "--stay" => c.stay = true,
            "--at" => at = Some(numbers::<3>(&value(&mut args, &flag)?)?),
            "--desired" => desired = Some(numbers::<3>(&value(&mut args, &flag)?)?),
            "--repeat" => {
                repeat = value(&mut args, &flag)?
                    .parse()
                    .map_err(|e| format!("--repeat: {e}"))?
            }
            other => return Err(format!("unknown argument {other} (see the source header)")),
        }
    }
    Ok((
        geometry,
        c,
        at.ok_or("--at is required")?,
        desired.ok_or("--desired is required")?,
        repeat,
    ))
}

fn main() -> ExitCode {
    let (geometry, c, at, desired, repeat) = match parse() {
        Ok(parsed) => parsed,
        Err(error) => {
            eprintln!("{error}");
            return ExitCode::FAILURE;
        }
    };
    let (mut r_at, mut e_at) = (at, at);
    println!("move  side    translation (x, y, z)                    grounded  position");
    for k in 0..repeat {
        let (r, e) = (
            rapier(&geometry, c, r_at, desired),
            engine(&geometry, c, e_at, desired),
        );
        for (side, o, p) in [("rapier", r, &mut r_at), ("engine", e, &mut e_at)] {
            for (a, t) in p.iter_mut().zip(o.translation) {
                *a += t;
            }
            println!(
                "{k:<5} {side:<7} ({:>11.7}, {:>11.7}, {:>11.7})  {:<8}  ({:.5}, {:.5}, {:.5})",
                o.translation[0], o.translation[1], o.translation[2], o.grounded, p[0], p[1], p[2]
            );
        }
    }
    ExitCode::SUCCESS
}
