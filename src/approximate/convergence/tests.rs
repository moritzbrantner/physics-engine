use super::*;
use crate::{
    BodyId,
    approximate::{Config, Error, Shape, Vector as V, World},
};

fn sphere(id: u64, x: f64, mass: f64) -> Body {
    Body::new(BodyId(id), Shape::Sphere(1.0), V(x, 0.0, 0.0), mass)
}
fn row(a: usize, b: usize) -> Constraint {
    Constraint {
        a,
        b,
        n: V::X,
        ra: V::ZERO,
        rb: V::ZERO,
        t1: V::Y,
        t2: V::Z,
        normal_mass: 1.0,
        tangent_mass: [1.0; 2],
        bias: 0.0,
        #[cfg(feature = "experimental-soft-contact")]
        hard_normal: true,
        #[cfg(feature = "experimental-soft-contact")]
        relaxing_normal: false,
        #[cfg(feature = "experimental-soft-contact")]
        normal_coefficients: super::super::correction::Coefficients::RIGID,
        friction: 0.5,
        normal_impulse: 0.0,
        tangent_impulse: [0.0; 2],
        sep: 0.0,
        swept: false,
        response: [true, true],
        spin: false,
    }
}

#[test]
fn malformed_convergence_configuration_is_rejected() {
    for value in [f64::NAN, f64::INFINITY, -0.01] {
        for field in 0..3 {
            let mut c = Convergence::default();
            match field {
                0 => c.absolute_velocity = value,
                1 => c.absolute_impulse = value,
                _ => c.relative = value,
            }
            assert!(matches!(
                World::new(Config {
                    convergence: Some(c),
                    ..Config::default()
                }),
                Err(Error::InvalidInput)
            ));
        }
    }
    assert!(
        World::new(Config {
            convergence: Some(Convergence {
                relative: 0.02,
                ..Convergence::default()
            }),
            ..Config::default()
        })
        .is_err()
    );
    assert!(
        World::new(Config {
            convergence: Some(Convergence {
                absolute_velocity: 0.0,
                absolute_impulse: 0.0,
                relative: 0.0
            }),
            ..Config::default()
        })
        .is_ok()
    );
}

#[test]
fn small_impulse_cannot_hide_a_large_velocity_error_on_a_light_body() {
    let b = vec![sphere(1, 0.0, 0.0), sphere(2, 2.0, 1e-6)];
    let mut c = row(0, 1);
    c.normal_mass = 1e-6;
    c.bias = 0.01;
    // 1e-8 is below the absolute impulse tolerance, but its velocity effect is 0.01.
    assert!(c.bias * c.normal_mass < Convergence::default().absolute_impulse);
    let mut stats = ConvergenceStats::default();
    assert!(projected_residual(&b, &[c], Convergence::default(), &mut stats).is_none());
    assert_eq!(stats.residual_checks, 1);
    assert_eq!(stats.residual_constraint_visits, 1);
}

#[test]
fn residual_respects_separating_normals_and_saturated_sliding_friction() {
    let mut b = vec![sphere(1, 0.0, 0.0), sphere(2, 2.0, 1.0)];
    let mut c = row(0, 1);
    b[1].velocity = V(10.0, 0.0, 0.0);
    assert_eq!(
        projected_residual(
            &b,
            std::slice::from_ref(&c),
            Convergence::default(),
            &mut ConvergenceStats::default()
        ),
        Some(0.0)
    );
    b[1].velocity = V(0.0, 10.0, 0.0);
    c.normal_impulse = 2.0;
    c.tangent_impulse[0] = -1.0; // Exactly the friction disk boundary; continued sliding is legal.
    assert_eq!(
        projected_residual(
            &b,
            std::slice::from_ref(&c),
            Convergence::default(),
            &mut ConvergenceStats::default()
        ),
        Some(0.0)
    );
    c.tangent_impulse[0] = 0.0; // Unsatisfied friction, not converged.
    assert!(
        projected_residual(
            &b,
            &[c],
            Convergence::default(),
            &mut ConvergenceStats::default()
        )
        .is_none()
    );
}

#[test]
fn final_residual_catches_an_earlier_constraint_disturbed_by_a_later_contact() {
    let mut b = vec![
        sphere(1, -2.0, 0.0),
        sphere(2, 0.0, 1.0),
        sphere(3, 2.0, 1.0),
    ];
    b[2].velocity = V(-2.0, 0.0, 0.0);
    let a = row(0, 1);
    let mut c = row(1, 2);
    c.normal_mass = 0.5;
    // First row is already satisfied before the second row changes the common body.
    assert_eq!(
        projected_residual(
            &b,
            std::slice::from_ref(&a),
            Convergence::default(),
            &mut ConvergenceStats::default()
        ),
        Some(0.0)
    );
    let mut report = Report::default();
    let prepared: Vec<_> = b
        .iter()
        .map(|b| PreparedResponse::new(b, &mut report))
        .collect();
    solve::<true, false>(
        &mut b,
        &prepared,
        std::slice::from_mut(&mut c),
        1,
        Convergence::default(),
        &mut report,
    );
    assert!(b[1].velocity.0 < -0.5);
    assert!(
        projected_residual(
            &b,
            &[a],
            Convergence::default(),
            &mut ConvergenceStats::default()
        )
        .is_none()
    );
}

#[test]
fn two_passes_solve_an_isolated_elastic_impact_without_changing_momentum_or_carry_forward() {
    let mut w = World::new(Config {
        gravity: V::ZERO,
        substeps: 1,
        ..Config::default()
    })
    .unwrap();
    for (id, x, velocity) in [(1, -1.0, 10.0), (2, 1.0, -10.0)] {
        let mut b = sphere(id, x, 1.0);
        b.velocity = V(velocity, 0.0, 0.0);
        b.restitution = 1.0;
        w.add_body(b).unwrap();
    }
    let mut fixed = w.clone();
    fixed.config.convergence = None;
    let r = w.step(1.0 / 60.0).unwrap();
    let f = fixed.step(1.0 / 60.0).unwrap();
    assert_eq!(r.impulse_iterations, 2);
    assert_eq!(r.convergence.converged_substeps, 1);
    assert_eq!(r.convergence.skipped_iterations, 6);
    assert_eq!(r.convergence.max_exit_velocity_residual, 0.0);
    assert_eq!(f.impulse_iterations, 8);
    assert_eq!(w.bodies, fixed.bodies);
    assert_eq!(w.bodies[0].velocity + w.bodies[1].velocity, V::ZERO);
    for _ in 0..20 {
        w.step(1.0 / 60.0).unwrap();
        fixed.step(1.0 / 60.0).unwrap();
    }
    assert_eq!(w.bodies, fixed.bodies);
    assert!(w.bodies[0].position.0 < -1.0);
}

#[test]
fn empty_contacts_skip_passes_without_dropping_forces_impulses_or_time() {
    let mut w = World::new(Config {
        gravity: V(0.0, -10.0, 0.0),
        ..Config::default()
    })
    .unwrap();
    w.add_body(sphere(1, 0.0, 2.0)).unwrap();
    w.add_force(BodyId(1), V(4.0, 0.0, 0.0)).unwrap();
    w.apply_impulse(BodyId(1), V(2.0, 0.0, 0.0), V::ZERO)
        .unwrap();
    let mut fixed = w.clone();
    fixed.config.convergence = None;
    for _ in 0..3 {
        let r = w.step(1.0 / 60.0).unwrap();
        fixed.step(1.0 / 60.0).unwrap();
        assert_eq!(r.impulse_iterations, 0);
        assert_eq!(r.convergence.empty_substeps, 4);
        assert_eq!(r.convergence.skipped_iterations, 32);
        assert_eq!(w.bodies, fixed.bodies);
        assert_eq!(w.elapsed, fixed.elapsed);
    }
    let r = w.step(0.0).unwrap();
    assert_eq!(r.convergence, ConvergenceStats::default());
}

#[test]
fn one_pass_budget_remains_one_and_coupled_chain_reaches_the_ceiling() {
    let bodies = vec![
        sphere(1, 0.0, 0.0),
        sphere(2, 2.0, 1.0),
        sphere(3, 4.0, 1.0),
    ];
    for iterations in [1, 2, 8] {
        let mut b = bodies.clone();
        b[2].velocity = V(-10.0, 0.0, 0.0);
        let mut constraints = vec![row(0, 1), row(1, 2)];
        constraints[1].normal_mass = 0.5;
        let mut r = Report::default();
        let prepared: Vec<_> = b.iter().map(|b| PreparedResponse::new(b, &mut r)).collect();
        solve::<true, true>(
            &mut b,
            &prepared,
            &mut constraints,
            iterations,
            Convergence::default(),
            &mut r,
        );
        assert_eq!(r.impulse_iterations, u64::from(iterations));
        assert_eq!(r.convergence.capped_substeps, 1);
        assert_eq!(
            r.convergence.probe_passes,
            if iterations == 8 { 2 } else { 0 }
        );
        assert_eq!(r.convergence.converged_substeps, 0);
    }
}

#[test]
fn tolerance_and_stopping_decisions_repeat_through_warm_start_dt_changes_and_support_removal() {
    let mut a = World::new(Config {
        gravity: V(0.0, -10.0, 0.0),
        ..Config::default()
    })
    .unwrap();
    a.add_body(Body::new(
        BodyId(1),
        Shape::Box(V(10.0, 1.0, 10.0)),
        V(0.0, -1.0, 0.0),
        0.0,
    ))
    .unwrap();
    for id in 2..6 {
        let mut body = Body::new(
            BodyId(id),
            Shape::Box(V(1.0, 1.0, 1.0)),
            V(0.0, 1.0 + 2.0 * (id - 2) as f64, 0.0),
            1.0,
        );
        body.rotation_locked = true;
        a.add_body(body).unwrap();
    }
    let mut b = a.clone();
    for _ in 0..240 {
        let ra = a.step(1.0 / 60.0).unwrap();
        let rb = b.step(1.0 / 60.0).unwrap();
        assert_eq!(a.bodies, b.bodies);
        assert_eq!(ra.convergence, rb.convergence);
    }
    assert!(a.is_quiescent());
    a.remove_body(BodyId(1));
    b.remove_body(BodyId(1));
    for dt in [1.0 / 30.0, 1.0 / 120.0, 1.0 / 60.0, 0.01] {
        let ra = a.step(dt).unwrap();
        let rb = b.step(dt).unwrap();
        assert_eq!(a.bodies, b.bodies);
        assert_eq!(ra.convergence, rb.convergence);
        assert_eq!(
            ra.impulse_iterations + ra.convergence.skipped_iterations,
            u64::from(ra.substeps) * 8
        );
    }
    assert!(a.body(BodyId(2)).unwrap().velocity.1 < 0.0);
}

#[test]
fn mass_ratio_friction_and_offcenter_contacts_keep_fixed_reference_quality() {
    for mass in [1e-6, 1.0, 1e6] {
        for locked in [true, false] {
            let mut a = World::new(Config {
                gravity: V(0.0, -10.0, 0.0),
                ..Config::default()
            })
            .unwrap();
            a.add_body(Body::new(
                BodyId(1),
                Shape::Box(V(100.0, 1.0, 100.0)),
                V(0.0, -1.0, 0.0),
                0.0,
            ))
            .unwrap();
            let mut body = Body::new(
                BodyId(2),
                Shape::Box(V(1.0, 1.0, 1.0)),
                V(0.0, 1.0, 0.0),
                mass,
            );
            body.rotation_locked = locked;
            body.velocity = V(5.0, -1.0, 0.0);
            body.friction = 0.8;
            a.add_body(body).unwrap();
            let mut fixed = a.clone();
            fixed.config.convergence = None;
            for _ in 0..60 {
                a.step(1.0 / 60.0).unwrap();
                fixed.step(1.0 / 60.0).unwrap();
                let (a, b) = (a.body(BodyId(2)).unwrap(), fixed.body(BodyId(2)).unwrap());
                assert!(
                    (a.position - b.position).length() < 0.001,
                    "mass={mass}, locked={locked}"
                );
                assert!((a.velocity - b.velocity).length() < 0.001);
                assert!(a.position.1 > 0.95);
            }
        }
    }
}

#[test]
fn rounding_away_a_correction_does_not_prove_convergence() {
    let mut b = vec![sphere(1, 0.0, 0.0), sphere(2, 2.0, 1.0)];
    let mut c = row(0, 1);
    c.normal_impulse = 1e20;
    c.bias = 0.1;
    assert_eq!(c.normal_impulse + c.bias * c.normal_mass, c.normal_impulse);
    assert!(
        projected_residual(
            &b,
            std::slice::from_ref(&c),
            Convergence::default(),
            &mut ConvergenceStats::default()
        )
        .is_none()
    );
    c.bias = 0.0;
    c.friction = 1.0;
    c.tangent_impulse = [-1e19, 0.0];
    b[1].velocity = V(0.0, 0.1, 0.0);
    assert_eq!(
        c.tangent_impulse[0] - b[1].velocity.1 * c.tangent_mass[0],
        c.tangent_impulse[0]
    );
    assert!(
        projected_residual(
            &b,
            &[c],
            Convergence::default(),
            &mut ConvergenceStats::default()
        )
        .is_none()
    );
}
#[test]
fn large_sideways_velocity_cannot_relax_the_normal_error_limit() {
    let mut b = vec![sphere(1, 0.0, 0.0), sphere(2, 2.0, 1.0)];
    b[1].velocity = V(-0.01, 1e9, 0.0);
    let mut c = row(0, 1);
    c.friction = 0.0;
    assert!(
        projected_residual(
            &b,
            &[c],
            Convergence::default(),
            &mut ConvergenceStats::default()
        )
        .is_none()
    );
}

#[cfg(feature = "experimental-soft-contact")]
#[test]
fn soft_residual_checks_compliance_instead_of_rigid_zero_velocity() {
    let b = vec![sphere(1, 0.0, 0.0), sphere(2, 2.0, 1.0)];
    let mut c = row(0, 1);
    c.hard_normal = false;
    c.friction = 0.0;
    c.bias = 3.0;
    let coefficients = crate::approximate::SoftContact::default()
        .prepare(1.0 / 240.0)
        .unwrap();
    c.normal_coefficients = coefficients;
    c.normal_impulse =
        c.bias * c.normal_mass * coefficients.mass_scale / coefficients.impulse_scale;
    assert!(
        projected_residual(
            &b,
            std::slice::from_ref(&c),
            Convergence::default(),
            &mut ConvergenceStats::default()
        )
        .is_some()
    );
    c.hard_normal = true;
    assert!(
        projected_residual(
            &b,
            std::slice::from_ref(&c),
            Convergence::default(),
            &mut ConvergenceStats::default()
        )
        .is_none()
    );
    c.hard_normal = false;
    c.normal_impulse *= 1.01;
    assert!(
        projected_residual(
            &b,
            &[c],
            Convergence::default(),
            &mut ConvergenceStats::default()
        )
        .is_none()
    );
}
#[cfg(feature = "experimental-soft-contact")]
#[test]
fn soft_solver_counts_complete_passes_and_checks_its_final_residual() {
    let mut b = vec![sphere(1, 0.0, 0.0), sphere(2, 2.0, 1.0)];
    let mut c = row(0, 1);
    c.hard_normal = false;
    c.friction = 0.0;
    c.bias = 3.0;
    let coefficients = crate::approximate::SoftContact::default()
        .prepare(1.0 / 240.0)
        .unwrap();
    let mut report = Report::default();
    let responses = vec![PreparedResponse::default(); 2];
    c.normal_coefficients = coefficients;
    solve::<false, true>(
        &mut b,
        &responses,
        std::slice::from_mut(&mut c),
        8,
        Convergence::default(),
        &mut report,
    );
    assert_eq!(report.convergence.converged_substeps, 1);
    assert_eq!(
        report.impulse_iterations + report.convergence.skipped_iterations,
        8
    );
    assert!(c.normal_impulse > 0.0);
    assert!(
        projected_residual(
            &b,
            &[c],
            Convergence::default(),
            &mut ConvergenceStats::default()
        )
        .is_some()
    );
}
