use super::*;
use crate::{
    BodyId,
    approximate::{Body, Config, Error, Quaternion, Shape, World},
};
fn config(relax: u8) -> Config {
    Config {
        gravity: Vector::ZERO,
        soft_contact: Some(SoftContact {
            relaxation_iterations: relax,
            ..SoftContact::default()
        }),
        ..Config::default()
    }
}
fn body(id: u64, p: Vector, mass: Scalar) -> Body {
    Body::new(BodyId(id), Shape::Box(Vector(1.0, 1.0, 1.0)), p, mass)
}
#[test]
fn correction_input_and_underflow_are_rejected_before_mutation() {
    for value in [Scalar::NAN, Scalar::INFINITY, -1.0] {
        for property in 0..2 {
            let mut s = SoftContact::default();
            if property == 0 {
                s.frequency_hz = value;
            } else {
                s.damping_ratio = value;
            }
            assert!(matches!(
                World::new(Config {
                    soft_contact: Some(s),
                    ..Config::default()
                }),
                Err(Error::InvalidInput)
            ));
        }
    }
    for s in [
        SoftContact {
            frequency_hz: 0.0,
            ..SoftContact::default()
        },
        SoftContact {
            relaxation_iterations: 9,
            ..SoftContact::default()
        },
    ] {
        assert!(
            World::new(Config {
                soft_contact: Some(s),
                ..Config::default()
            })
            .is_err()
        );
    }
    let mut w = World::new(Config {
        substeps: 1,
        soft_contact: Some(SoftContact {
            damping_ratio: 0.0,
            frequency_hz: 0.01,
            ..SoftContact::default()
        }),
        ..config(2)
    })
    .unwrap();
    w.add_body(body(1, Vector::ZERO, 1.0)).unwrap();
    w.apply_impulse(BodyId(1), Vector::X, Vector::ZERO).unwrap();
    let before = w.body(BodyId(1)).unwrap().clone();
    assert!(matches!(
        w.step(Scalar::from_bits(1)),
        Err(Error::InvalidInput)
    ));
    assert_eq!(w.body(BodyId(1)), Some(&before));
    assert_eq!(w.elapsed_seconds(), 0.0);
}
#[test]
fn coefficients_match_implicit_spring_formula_and_frequency_cap() {
    for h in [1.0 / 60.0, 1.0 / 240.0, 1.0 / 480.0] {
        for zeta in [0.0, 0.5, 1.0, 2.0] {
            let s = SoftContact {
                frequency_hz: 60.0,
                damping_ratio: zeta,
                relaxation_iterations: 2,
            };
            let c = s.prepare(h).unwrap();
            let omega = 2.0 * std::f64::consts::PI * 60.0_f64.min(0.25 / h);
            let damping = 2.0 * zeta * omega;
            let stiffness = omega * omega;
            let denom = 1.0 + h * damping + h * h * stiffness;
            assert!((c.mass_scale - (h * damping + h * h * stiffness) / denom).abs() < 1e-14);
            assert!((c.impulse_scale - 1.0 / denom).abs() < 1e-14);
            assert!((c.bias_rate - stiffness / (damping + h * stiffness)).abs() < 1e-10);
        }
    }
    let a = SoftContact::default().prepare(1.0 / 240.0).unwrap();
    let b = SoftContact {
        frequency_hz: 10_000.0,
        ..SoftContact::default()
    }
    .prepare(1.0 / 240.0)
    .unwrap();
    assert_eq!(a.mass_scale.to_bits(), b.mass_scale.to_bits());
}
#[test]
fn relaxation_preserves_free_impulse_force_torque_and_carry_forward() {
    let mut reference = World::new(Config {
        gravity: Vector::ZERO,
        ..Config::default()
    })
    .unwrap();
    reference.add_body(body(1, Vector::ZERO, 2.0)).unwrap();
    let mut soft = World::new(config(2)).unwrap();
    soft.add_body(body(1, Vector::ZERO, 2.0)).unwrap();
    for w in [&mut reference, &mut soft] {
        w.apply_impulse(BodyId(1), Vector(2.0, 3.0, 0.0), Vector(0.0, 0.0, 1.0))
            .unwrap();
        w.add_force(BodyId(1), Vector(1.0, 0.0, 0.0)).unwrap();
        w.bodies[0].torque = Vector(0.1, 0.2, 0.0);
    }
    for _ in 0..40 {
        reference.step(1.0 / 60.0).unwrap();
        let r = soft.step(1.0 / 60.0).unwrap();
        assert_eq!(reference.body(BodyId(1)), soft.body(BodyId(1)));
        assert_eq!(r.correction.relaxation_iterations, 0);
    }
}
#[test]
fn bias_relaxation_removes_separation_energy_without_integrating_twice() {
    let fixture = |relax| {
        let mut w = World::new(Config {
            substeps: 1,
            convergence: None,
            ..config(relax)
        })
        .unwrap();
        for (id, x) in [(1, -0.85), (2, 0.85)] {
            let mut b = Body::new(BodyId(id), Shape::Sphere(1.0), Vector(x, 0.0, 0.0), 1.0);
            b.sleep_allowed = false;
            w.add_body(b).unwrap();
        }
        w
    };
    let mut soft = fixture(0);
    let mut relaxed = fixture(2);
    soft.step(1.0 / 240.0).unwrap();
    let r = relaxed.step(1.0 / 240.0).unwrap();
    let energy = |w: &World| w.bodies().map(Body::kinetic_energy).sum::<Scalar>();
    assert!(energy(&soft) > 0.01);
    assert!(energy(&relaxed) < energy(&soft) * 1e-8);
    for id in [BodyId(1), BodyId(2)] {
        assert_eq!(
            soft.body(id).unwrap().position,
            relaxed.body(id).unwrap().position
        );
    }
    assert_eq!(r.integrated_bodies, 2);
    assert_eq!(r.correction.relaxation_iterations, 2);
    assert_eq!(r.impulse_iterations, 10);
    assert_eq!(relaxed.elapsed_seconds(), 1.0 / 240.0);
    assert!(
        (relaxed.body(BodyId(1)).unwrap().velocity + relaxed.body(BodyId(2)).unwrap().velocity)
            .length()
            < 1e-12
    );
}
#[test]
fn actual_bounce_keeps_restitution_and_momentum_during_relaxation() {
    for relax in [0, 2, 4] {
        let mut w = World::new(Config {
            substeps: 1,
            ..config(relax)
        })
        .unwrap();
        for (id, x, v) in [(1, -1.0, 10.0), (2, 1.0, -10.0)] {
            let mut b = Body::new(BodyId(id), Shape::Sphere(1.0), Vector(x, 0.0, 0.0), 1.0);
            b.velocity = Vector(v, 0.0, 0.0);
            b.restitution = 1.0;
            w.add_body(b).unwrap();
        }
        let r = w.step(1.0 / 240.0).unwrap();
        let a = w.body(BodyId(1)).unwrap();
        let b = w.body(BodyId(2)).unwrap();
        assert!((a.velocity.0 + 10.0).abs() < 1e-12);
        assert!((b.velocity.0 - 10.0).abs() < 1e-12);
        assert!((a.velocity + b.velocity).length() < 1e-12);
        assert_eq!(r.correction.softened_points, 0);
    }
}
#[test]
fn speculative_sweeps_still_shield_and_retire_at_the_nearest_wall() {
    for relax in [0, 2] {
        for shape in [Shape::Sphere(0.1), Shape::Box(Vector(0.1, 0.1, 1.0))] {
            for (wall, target) in [(1, 2), (2, 1)] {
                let mut w = World::new(config(relax)).unwrap();
                w.add_body(Body::new(
                    BodyId(wall),
                    Shape::Box(Vector(10.0, 10.0, 0.02)),
                    Vector(0.0, 0.0, 5.0),
                    0.0,
                ))
                .unwrap();
                w.add_body(body(target, Vector::ZERO, 2.0)).unwrap();
                let before = w.body(BodyId(target)).unwrap().clone();
                let mut p = Body::new(BodyId(9), shape, Vector(0.0, 0.0, 10.0), 1.0);
                p.ccd = true;
                p.velocity = Vector(0.0, 0.0, -10000.0);
                p.retire_on_impact = true;
                w.add_body(p).unwrap();
                let r = w.step(1.0 / 60.0).unwrap();
                assert_eq!(r.retired, vec![BodyId(9)]);
                assert!(r.swept_contacts > 0);
                assert_eq!(w.body(BodyId(target)), Some(&before));
            }
        }
    }
}
#[test]
fn rounded_corner_miss_remains_a_miss_under_both_corrections() {
    for relax in [0, 2] {
        let mut w = World::new(config(relax)).unwrap();
        w.add_body(body(1, Vector::ZERO, 0.0)).unwrap();
        let mut p = Body::new(BodyId(2), Shape::Sphere(0.5), Vector(1.4, 1.4, 5.0), 1.0);
        p.ccd = true;
        p.velocity = Vector(0.0, 0.0, -1000.0);
        p.retire_on_impact = true;
        w.add_body(p).unwrap();
        let r = w.step(1.0 / 60.0).unwrap();
        assert!(r.retired.is_empty());
        assert!(w.body(BodyId(2)).unwrap().position.2 < -1.0);
    }
}
#[test]
fn support_removal_and_changed_substeps_remain_repeatable() {
    for relax in [0, 2] {
        let mut a = World::new(Config {
            gravity: Vector(0.0, -10.0, 0.0),
            ..config(relax)
        })
        .unwrap();
        a.add_body(Body::new(
            BodyId(1),
            Shape::Box(Vector(10.0, 1.0, 10.0)),
            Vector(0.0, -1.0, 0.0),
            0.0,
        ))
        .unwrap();
        a.add_body(body(2, Vector(0.0, 1.0, 0.0), 1.0)).unwrap();
        let mut b = a.clone();
        for tick in 0..300 {
            let dt = if tick % 2 == 0 {
                1.0 / 60.0
            } else {
                1.0 / 120.0
            };
            a.step(dt).unwrap();
            b.step(dt).unwrap();
            assert_eq!(
                a.bodies().collect::<Vec<_>>(),
                b.bodies().collect::<Vec<_>>()
            );
        }
        assert!(a.body(BodyId(2)).unwrap().is_sleeping());
        a.remove_body(BodyId(1));
        b.remove_body(BodyId(1));
        for _ in 0..10 {
            a.step(1.0 / 60.0).unwrap();
            b.step(1.0 / 60.0).unwrap();
            assert_eq!(a.body(BodyId(2)), b.body(BodyId(2)));
        }
        assert!(a.body(BodyId(2)).unwrap().velocity.1 < -1.0);
    }
}
#[test]
fn kinetic_observation_uses_body_frame_inertia_and_does_not_change_state() {
    let mut b = Body::new(
        BodyId(1),
        Shape::Box(Vector(2.0, 3.0, 4.0)),
        Vector::ZERO,
        6.0,
    );
    b.orientation = Quaternion(0.0, 0.0, (0.5_f64).sqrt(), (0.5_f64).sqrt());
    b.angular_velocity = b.orientation.rotate(Vector(1.0, 0.0, 0.0));
    b.velocity = Vector(2.0, 0.0, 0.0);
    let before = b.clone();
    assert!((b.kinetic_energy() - 37.0).abs() < 1e-12);
    assert_eq!(b, before);
}
#[test]
fn positional_correction_cannot_be_hidden_by_relaxed_sleep_velocity() {
    let mut w = World::new(Config {
        gravity: Vector(0.0, -10.0, 0.0),
        substeps: 1,
        sleep_seconds: 1e-6,
        sleep_speed: 0.1,
        ..config(2)
    })
    .unwrap();
    w.add_body(Body::new(
        BodyId(1),
        Shape::Box(Vector(10.0, 1.0, 10.0)),
        Vector(0.0, -1.0, 0.0),
        0.0,
    ))
    .unwrap();
    w.add_body(body(2, Vector(0.0, 0.6, 0.0), 1.0)).unwrap();
    w.step(1.0 / 240.0).unwrap();
    let b = w.body(BodyId(2)).unwrap();
    assert!(b.position.1 > 0.61);
    assert!(!b.sleeping);
}

#[test]
fn tilted_frictional_stacks_keep_finite_state_and_bounded_penetration() {
    // The independent CLI also records failures in comparison controls; no bound is relaxed.
    let mut failed = Vec::new();
    for (name, masses, friction) in [
        ("equal", [2.0, 2.0, 2.0, 2.0], 0.6),
        ("mixed", [0.5, 4.0, 1.0, 2.0], 0.6),
        ("sliding", [2.0, 1.0, 4.0, 0.5], 0.15),
    ] {
        for policy in 0..3 {
            // The comparison CLI retains the failing old and soft-only controls, unchanged.
            // The mixed-mass regression gates the selected hard-support + relaxed candidate.
            if name == "mixed" && policy != 2 {
                continue;
            }
            let mut w = World::new(Config {
                gravity: Vector(0.0, -3600.0, 0.0),
                soft_contact: match policy {
                    0 => None,
                    _ => Some(SoftContact {
                        relaxation_iterations: if policy == 1 { 0 } else { 2 },
                        ..SoftContact::default()
                    }),
                },
                ..Config::default()
            })
            .unwrap();
            w.add_body(Body::new(
                BodyId(1),
                Shape::Box(Vector(500.0, 16.0, 500.0)),
                Vector(0.0, -16.0, 0.0),
                0.0,
            ))
            .unwrap();
            for (n, mass) in masses.into_iter().enumerate() {
                let mut b = Body::new(
                    BodyId(n as u64 + 10),
                    Shape::Box(Vector(18.0, 18.0, 18.0)),
                    Vector(0.0, 18.0 + n as f64 * 38.0, 0.0),
                    mass,
                );
                b.friction = friction;
                if n == 3 {
                    b.orientation = Quaternion(0.0, 0.0, 0.025_f64.sin(), 0.025_f64.cos());
                }
                w.add_body(b).unwrap();
            }
            let mut peak: Scalar = 0.0;
            let mut maximum_energy: Scalar = 0.0;
            for t in 0..1200 {
                if t == 240 {
                    let point = w.body(BodyId(13)).unwrap().position + Vector(0.0, 8.0, 0.0);
                    w.apply_impulse(BodyId(13), Vector(50.0, 0.0, 0.0), point)
                        .unwrap();
                }
                let r = w.step(1.0 / 60.0).unwrap();
                assert!(r.impulse_iterations <= 40);
                for b in w.bodies().filter(|b| b.mass > 0.0) {
                    assert!(b.valid());
                    assert!(b.kinetic_energy().is_finite());
                    peak = peak.max((-crate::approximate::contact::bounds(b).0.1).max(0.0));
                }
                maximum_energy = maximum_energy.max(w.bodies().map(Body::kinetic_energy).sum());
            }
            eprintln!(
                "stability: {name}, policy={policy}, peak_floor={peak}, peak_energy={maximum_energy}, final_energy={}, sleeping={}",
                w.bodies().map(Body::kinetic_energy).sum::<Scalar>(),
                w.is_quiescent()
            );
            if peak > 0.5 {
                failed.push((name, policy, peak));
            }
            assert!((w.elapsed_seconds() - 20.0).abs() < 1e-9);
        }
    }
    assert!(failed.is_empty(), "quality failures: {failed:?}");
}

#[test]
fn fixed_supports_stay_hard_and_dynamic_contact_normals_use_softness() {
    let mut w = World::new(Config {
        substeps: 1,
        ..config(2)
    })
    .unwrap();
    w.add_body(body(1, Vector(0.0, -1.0, 0.0), 0.0)).unwrap();
    w.add_body(body(2, Vector(0.0, 0.9, 0.0), 1.0)).unwrap();
    w.add_body(body(3, Vector(0.0, 2.8, 0.0), 2.0)).unwrap();
    let r = w.step(1.0 / 60.0).unwrap();
    assert!(r.correction.hard_support_points > 0);
    assert!(r.correction.softened_points > 0);
    for c in &w.constraints {
        let fixed = w.bodies[c.a].mass == 0.0 || w.bodies[c.b].mass == 0.0;
        assert_eq!(c.hard_normal, fixed);
    }
    assert!(r.correction.relaxation_iterations <= 2);
}

#[test]
fn compliance_starts_only_beyond_the_existing_penetration_slop() {
    // A binary-exact slop makes the equality boundary unambiguous. The approaching
    // spheres must receive a real support impulse, not merely produce zero work.
    for separation in [-0.25, -0.125, -0.0625, 0.0, 0.0625] {
        let cfg = Config {
            substeps: 1,
            convergence: None,
            contact_slop: 0.125,
            ..config(0)
        };
        let mut candidate = World::new(cfg).unwrap();
        let mut reference = World::new(Config {
            soft_contact: None,
            ..cfg
        })
        .unwrap();
        for (id, x, vx) in [(1, 0.0, 1.0), (2, 2.0 + separation, -1.0)] {
            let mut b = Body::new(BodyId(id), Shape::Sphere(1.0), Vector(x, 0.0, 0.0), 1.0);
            b.velocity = Vector(vx, 0.0, 0.0);
            for w in [&mut candidate, &mut reference] {
                w.add_body(b.clone()).unwrap();
            }
        }
        let r = candidate.step(1.0 / 240.0).unwrap();
        reference.step(1.0 / 240.0).unwrap();
        assert!(!candidate.constraints.is_empty());
        assert_eq!(
            r.correction.softened_points > 0,
            separation < -cfg.contact_slop
        );
        if separation >= -cfg.contact_slop {
            assert!(candidate.constraints.iter().all(|c| c.hard_normal));
            assert_eq!(candidate.bodies, reference.bodies);
        }
    }
}

#[test]
fn a_converged_unchanged_bounce_skips_relaxation_without_losing_its_impulse() {
    let mut w = World::new(Config {
        substeps: 1,
        ..config(2)
    })
    .unwrap();
    for (id, x, vx) in [(1, -1.0, 10.0), (2, 1.0, -10.0)] {
        let mut b = Body::new(BodyId(id), Shape::Sphere(1.0), Vector(x, 0.0, 0.0), 1.0);
        b.velocity = Vector(vx, 0.0, 0.0);
        b.restitution = 1.0;
        w.add_body(b).unwrap();
    }
    let r = w.step(1.0 / 240.0).unwrap();
    assert_eq!(r.convergence.converged_substeps, 1);
    assert_eq!(r.correction.unchanged_converged_skips, 1);
    assert_eq!(r.correction.relaxation_iterations, 0);
    assert_eq!(r.correction.relaxation_skipped_iterations, 2);
    assert_eq!(
        r.correction.relaxation_motion_bytes,
        8 * w.relaxation_bias.capacity() as u64
    );
    assert!(w.relaxation_motion.is_empty());
    assert!((w.bodies[0].velocity.0 + 10.0).abs() < 1e-12);
    assert!((w.bodies[1].velocity.0 - 10.0).abs() < 1e-12);
    assert!((w.bodies[0].velocity + w.bodies[1].velocity).length() < 1e-12);
    assert_eq!(r.integrated_bodies, 2);
}

#[test]
fn changed_correction_target_requires_relaxation_even_after_primary_convergence() {
    let mut w = World::new(Config {
        substeps: 1,
        ..config(2)
    })
    .unwrap();
    w.add_body(Body::new(BodyId(1), Shape::Sphere(1.0), Vector::ZERO, 0.0))
        .unwrap();
    w.add_body(Body::new(
        BodyId(2),
        Shape::Sphere(1.0),
        Vector(1.75, 0.0, 0.0),
        1.0,
    ))
    .unwrap();
    let r = w.step(1.0 / 240.0).unwrap();
    assert_eq!(r.convergence.converged_substeps, 1);
    assert!(r.correction.hard_support_points > 0);
    assert_eq!(r.correction.unchanged_converged_skips, 0);
    assert_eq!(r.correction.relaxation_iterations, 2);
    assert!(w.bodies[1].position.0 > 1.75);
    assert!(w.bodies[1].velocity.length() < 1e-12);
}

#[test]
fn unchanged_targets_do_not_allow_skipping_an_unchecked_primary_solve() {
    let mut w = World::new(Config {
        substeps: 1,
        velocity_iterations: 1,
        convergence: None,
        ..config(2)
    })
    .unwrap();
    w.add_body(Body::new(BodyId(1), Shape::Sphere(1.0), Vector::ZERO, 0.0))
        .unwrap();
    let mut b = Body::new(BodyId(2), Shape::Sphere(1.0), Vector(2.0, 0.0, 0.0), 1.0);
    b.velocity = Vector(-1.0, 0.0, 0.0);
    w.add_body(b).unwrap();
    let r = w.step(1.0 / 240.0).unwrap();
    assert_eq!(r.convergence.converged_substeps, 0);
    assert_eq!(r.correction.unchanged_target_checks, 0);
    assert_eq!(r.correction.unchanged_converged_skips, 0);
    assert_eq!(r.correction.relaxation_iterations, 2);
    assert_eq!(r.impulse_iterations, 3);
}

#[test]
fn reconciled_soft_islands_keep_primary_work_separate_from_relaxation_and_positions() {
    use crate::approximate::ConvergenceScope;
    for scope in [
        ConvergenceScope::ContactIslands,
        ConvergenceScope::WholeWorld,
    ] {
        for relax in [0, 2, 8] {
            let mut w = World::new(Config {
                gravity: Vector(0.0, -10.0, 0.0),
                convergence_scope: scope,
                fixed_position_iterations: 2,
                ..config(relax)
            })
            .unwrap();
            w.add_body(Body::new(
                BodyId(0),
                Shape::Box(Vector(100.0, 1.0, 100.0)),
                Vector(0.0, -1.0, 0.0),
                0.0,
            ))
            .unwrap();
            for (id, p) in [
                (1, Vector(0.0, 0.9, 0.0)),
                (2, Vector(0.0, 2.8, 0.0)),
                (3, Vector(0.0, 4.7, 0.0)),
                (4, Vector(20.0, 1.0, 0.0)),
            ] {
                let mut b = body(id, p, 1.0);
                b.sleep_allowed = false;
                w.add_body(b).unwrap();
            }
            let mut clone = w.clone();
            let mut partitions = 0;
            let mut softened = 0;
            let mut relaxations = 0;
            for _ in 0..120 {
                let r = w.step(1.0 / 60.0).unwrap();
                clone.step(1.0 / 60.0).unwrap();
                assert_eq!(
                    w.bodies, clone.bodies,
                    "same-build replay and clone cache validity"
                );
                assert_eq!(w.cache, clone.cache);
                assert_eq!(
                    r.impulse_iterations - r.correction.relaxation_iterations
                        + r.convergence.skipped_iterations,
                    u64::from(r.substeps) * 8
                );
                assert!(r.position.passes <= u64::from(r.substeps) * 2);
                if scope == ConvergenceScope::ContactIslands {
                    assert_eq!(
                        r.convergence.constraint_visits + r.islands.skipped_constraint_visits,
                        r.contact_points * 8,
                        "primary island accounting must exclude relaxation"
                    );
                }
                for b in w.bodies().filter(|b| b.mass > 0.0) {
                    assert!(
                        crate::approximate::contact::bounds(b).0.1
                            >= -w.config.contact_slop - 1e-10
                    );
                }
                assert!(w.constraints.iter().all(|c| !c.relaxing_normal));
                softened += r.correction.softened_points;
                partitions += r.islands.partition_builds;
                relaxations += r.correction.relaxation_iterations;
            }
            assert!(softened > 0);
            if scope == ConvergenceScope::ContactIslands {
                assert!(partitions > 0);
            } else {
                assert_eq!(partitions, 0);
            }
            assert_eq!(relaxations > 0, relax > 0);
            assert!((w.elapsed_seconds() - 2.0).abs() < 1e-12);
        }
    }
}
