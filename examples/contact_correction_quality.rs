//! Held-out stability acceptance, separate from the tower timing matrix.
//! Exits nonzero on failed quality. --report-only retains identical failed records for research.
use physics_engine::{
    BodyId,
    approximate::{Body, Config, Quaternion, Shape, SoftContact, Vector as V, World},
};
use std::{env, fs, process};
// JSON has no NaN/infinity. A numerical failure must remain a readable failed record.
fn json_number(value: f64) -> String {
    if value.is_finite() {
        value.to_string()
    } else {
        "null".into()
    }
}
fn main() {
    let args = env::args().skip(1).collect::<Vec<_>>();
    if args.is_empty() || args.len() > 2 || args.get(1).is_some_and(|s| s != "--report-only") {
        eprintln!(
            "usage: cargo run --release --example contact_correction_quality -- result.json [--report-only]"
        );
        process::exit(2);
    }
    let mut records = Vec::new();
    let mut accepted = true;
    let mut all_policies_passed = true;
    for (name, masses, friction) in [
        ("equal", [2.0, 2.0, 2.0, 2.0], 0.6),
        ("mixed", [0.5, 4.0, 1.0, 2.0], 0.6),
        ("sliding", [2.0, 1.0, 4.0, 0.5], 0.15),
    ] {
        for (mode, soft_contact) in [
            ("baumgarte", None),
            (
                "soft",
                Some(SoftContact {
                    relaxation_iterations: 0,
                    ..SoftContact::default()
                }),
            ),
            ("relaxed", Some(SoftContact::default())),
        ] {
            let mut w = World::new(Config {
                soft_contact,
                ..Config::default()
            })
            .unwrap();
            w.add_body(Body::new(
                BodyId(1),
                Shape::Box(V(500.0, 16.0, 500.0)),
                V(0.0, -16.0, 0.0),
                0.0,
            ))
            .unwrap();
            for (n, mass) in masses.into_iter().enumerate() {
                let mut b = Body::new(
                    BodyId(n as u64 + 10),
                    Shape::Box(V(18.0, 18.0, 18.0)),
                    V(0.0, 18.0 + n as f64 * 38.0, 0.0),
                    mass,
                );
                b.friction = friction;
                if n == 3 {
                    b.orientation = Quaternion(0.0, 0.0, 0.025_f64.sin(), 0.025_f64.cos());
                }
                w.add_body(b).unwrap();
            }
            let mut peak = 0.0_f64;
            let mut energy_peak = 0.0_f64;
            let mut bounded = true;
            let mut failure = None;
            let mut ticks = 0;
            for t in 0..1200 {
                if t == 240 {
                    let point = w.body(BodyId(13)).unwrap().position + V(0.0, 8.0, 0.0);
                    w.apply_impulse(BodyId(13), V(50.0, 0.0, 0.0), point)
                        .unwrap();
                }
                match w.step(1.0 / 60.0) {
                    Ok(r) => bounded &= r.impulse_iterations <= 40,
                    Err(e) => {
                        failure = Some(format!("{e:?}"));
                        break;
                    }
                }
                ticks += 1;
                for b in w.bodies().filter(|b| b.mass > 0.0) {
                    let h = b.shape.half_extents();
                    let axis = b.orientation.axes();
                    let extent =
                        axis[0].1.abs() * h.0 + axis[1].1.abs() * h.1 + axis[2].1.abs() * h.2;
                    peak = peak.max((extent - b.position.1).max(0.0));
                }
                energy_peak = energy_peak.max(w.bodies().map(Body::kinetic_energy).sum());
            }
            let energy = w.bodies().map(Body::kinetic_energy).sum::<f64>();
            let passed = failure.is_none()
                && ticks == 1200
                && bounded
                && peak <= 0.5
                && energy.is_finite()
                && energy_peak.is_finite()
                && (w.elapsed_seconds() - 20.0).abs() < 1e-9;
            all_policies_passed &= passed;
            if mode == "relaxed" {
                accepted &= passed;
            }
            let peak = json_number(peak);
            let energy_peak = json_number(energy_peak);
            let energy = json_number(energy);
            records.push(format!("{{\"scene\":\"{name}\",\"mode\":\"{mode}\",\"masses\":{masses:?},\"friction\":{friction},\"passed\":{passed},\"completed_ticks\":{ticks},\"failure\":{},\"max_floor_penetration\":{peak},\"energy_peak\":{energy_peak},\"final_energy\":{energy},\"quiescent\":{},\"bounded_iterations\":{bounded}}}",failure.map_or("null".into(),|s|format!("\"{s}\"")),w.is_quiescent()));
        }
    }
    fs::write(&args[0],format!("{{\"kind\":\"contact-correction-heldout-v1\",\"complete\":true,\"passed\":{accepted},\"candidate_policy\":\"relaxed\",\"all_policies_passed\":{all_policies_passed},\"floor_penetration_limit\":0.5,\"total_ticks\":1200,\"impulse_tick\":240,\"note\":\"All policies retain the same floor limit. Promotion gates the selected relaxed candidate; failures in baseline and soft-only controls stay explicit. Energy units are mass*scene_units^2/s^2, not SI joules. Reporting mode does not relabel failures.\",\"records\":[{}]}}\n",records.join(",\n"))).expect("write quality report");
    if !accepted {
        eprintln!(
            "Held-out quality failed; inspect {}. Do not promote the correction policy.",
            args[0]
        );
        if args.len() == 1 {
            process::exit(1);
        }
    }
}
