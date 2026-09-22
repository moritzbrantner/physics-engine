//! Comparison adapter. Both solvers import the same Rust fixture, but each owns its own state.
use super::{PLAYER_ID, PROJECTILE_ID_START, ProjectileType, controller};
use physics_engine::{
    BodyId,
    approximate::{Body, Config, Quaternion, Shape, SoftContact, Vector, World},
};
use std::cell::RefCell;

struct Experiment {
    world: World,
    next_id: u64,
    kind: ProjectileType,
    snapshot: Vec<f64>,
    error: i32,
    retired: u32,
}
std::thread_local! {static EXPERIMENT:RefCell<Option<Experiment>>=const{RefCell::new(None)};}
fn read<T>(default: T, f: impl FnOnce(&Experiment) -> T) -> T {
    EXPERIMENT.with(|s| s.borrow().as_ref().map_or(default, f))
}
fn write(f: impl FnOnce(&mut Experiment) -> i32) -> i32 {
    EXPERIMENT.with(|s| s.borrow_mut().as_mut().map_or(-1, f))
}

/// Must be called immediately after the ordinary Pages reset. Construction copies only the fixture;
/// there is no legacy state roundtrip during any approximate tick.
#[unsafe(no_mangle)]
pub extern "C" fn approximate_reset_from_sandbox(substeps: i32, iterations: i32) -> i32 {
    reset_with_convergence(substeps, iterations, true, None)
}

/// Fixed-pass diagnostic reference; same fixture, math and collision path, no early termination.
#[unsafe(no_mangle)]
pub extern "C" fn approximate_reset_fixed_iterations_from_sandbox(
    substeps: i32,
    iterations: i32,
) -> i32 {
    reset_with_convergence(substeps, iterations, false, None)
}

/// Opt-in diagnostic correction: existing reset exports keep the merged Baumgarte policy.
#[unsafe(no_mangle)]
pub extern "C" fn approximate_reset_soft_from_sandbox(
    substeps: i32,
    iterations: i32,
    frequency_hz: f64,
    damping_ratio: f64,
    relaxation_iterations: i32,
) -> i32 {
    let Ok(relaxation_iterations) = u8::try_from(relaxation_iterations) else {
        return -1;
    };
    reset_with_convergence(
        substeps,
        iterations,
        true,
        Some(SoftContact {
            frequency_hz,
            damping_ratio,
            relaxation_iterations,
        }),
    )
}

fn reset_with_convergence(
    substeps: i32,
    iterations: i32,
    early: bool,
    soft_contact: Option<SoftContact>,
) -> i32 {
    let Ok(substeps) = u8::try_from(substeps) else {
        return -1;
    };
    let Ok(velocity_iterations) = u8::try_from(iterations) else {
        return -1;
    };
    let result = super::with_sandbox(|s| {
        let mut world = World::new(Config {
            gravity: s.world.config().gravity.into(),
            substeps,
            velocity_iterations,
            convergence: early.then(Default::default),
            soft_contact,
            ..Config::default()
        })?;
        for b in s.world.boxes() {
            world.add_body(Body::from_legacy(b))?;
        }
        Ok::<_, physics_engine::approximate::Error>(world)
    });
    let Ok(world) = result else {
        return -1;
    };
    EXPERIMENT.with(|s| {
        *s.borrow_mut() = Some(Experiment {
            world,
            next_id: PROJECTILE_ID_START,
            kind: ProjectileType::Sphere,
            snapshot: Vec::new(),
            error: 0,
            retired: 0,
        })
    });
    0
}
#[unsafe(no_mangle)]
pub extern "C" fn approximate_set_projectile_type(kind: i32) -> i32 {
    write(|s| {
        let Some(kind) = ProjectileType::from_i32(kind) else {
            return -1;
        };
        if s.world.bodies().any(|b| b.id.0 >= PROJECTILE_ID_START) {
            return -1;
        }
        s.kind = kind;
        0
    })
}
#[unsafe(no_mangle)]
pub extern "C" fn approximate_shoot(x: i32, y: i32, z: i32) -> i32 {
    write(|s| {
        let direction = Vector(
            x.clamp(-120, 120) as f64,
            y.clamp(-120, 120) as f64,
            z.clamp(-120, 120) as f64,
        );
        if direction == Vector::ZERO {
            return -1;
        }
        let Some(player) = s.world.body(PLAYER_ID) else {
            return -1;
        };
        let id = BodyId(s.next_id);
        let shape = match s.kind {
            ProjectileType::Sphere => Shape::Sphere(3.0),
            ProjectileType::Arrow => Shape::Box(Vector(2.0, 2.0, 12.0)),
            ProjectileType::Rigid => Shape::Box(Vector(3.0, 3.0, 3.0)),
        };
        // Match the current adapter's integer launch offset for paired benchmark comparability.
        let mut body = Body::new(
            id,
            shape,
            player.position
                + Vector(0.0, 12.0, 0.0)
                + Vector(
                    (x.clamp(-120, 120) / 3) as f64,
                    (y.clamp(-120, 120) / 3) as f64,
                    (z.clamp(-120, 120) / 3) as f64,
                ),
            1.0,
        );
        body.velocity = direction * 60.0;
        body.ccd = true;
        body.sleep_allowed = false;
        body.friction = 0.0;
        body.layers = controller::scenario_rules::projectile_layers();
        let policy = controller::scenario_rules::projectile_impact_policy();
        body.restitution = policy.restitution_milli() as f64 / 1000.0;
        body.retire_on_impact = policy.retire_on_contact();
        if s.kind == ProjectileType::Arrow {
            let q = super::projectile_direction_orientation(physics_engine::Vec3i::new(x, y, z));
            let k = physics_engine::ORIENTATION_SCALE as f64;
            body.orientation = Quaternion(
                q.x as f64 / k,
                q.y as f64 / k,
                q.z as f64 / k,
                q.w as f64 / k,
            );
            body.rotation_locked = true;
        }
        if s.world.add_body(body).is_err() {
            return -1;
        }
        s.next_id += 1;
        let projectiles = s
            .world
            .bodies()
            .filter(|b| b.id.0 >= PROJECTILE_ID_START)
            .map(|b| b.id)
            .collect::<Vec<_>>();
        if projectiles.len() > super::MAX_PROJECTILES {
            s.world.remove_body(projectiles[0]);
        }
        id.0 as i32
    })
}
#[unsafe(no_mangle)]
pub extern "C" fn approximate_step_velocity(x: i32, z: i32, jump: i32) -> i32 {
    write(|s| {
        let Some(player) = s.world.body(PLAYER_ID) else {
            return -1;
        };
        let mut velocity = player.velocity;
        velocity.0 += (x.clamp(-420, 420) as f64 - velocity.0).clamp(-120.0, 120.0);
        velocity.2 += (z.clamp(-420, 420) as f64 - velocity.2).clamp(-120.0, 120.0);
        if jump != 0 && s.world.has_support(PLAYER_ID) {
            velocity.1 = 960.0;
        }
        if s.world.set_velocity(PLAYER_ID, velocity).is_err() {
            s.error = 1;
            return 1;
        }
        match s.world.step(1.0 / 60.0) {
            Ok(report) => {
                s.error = 0;
                s.retired += report.retired.len() as u32;
            }
            Err(_) => {
                s.error = 1;
                return 1;
            }
        }
        let out = s
            .world
            .bodies()
            .filter(|b| {
                b.id.0 >= PROJECTILE_ID_START
                    && (b.position.1 < -150.0 || b.position.abs().max_component() > 1600.0)
            })
            .map(|b| b.id)
            .collect::<Vec<_>>();
        for id in out {
            s.world.remove_body(id);
        }
        0
    })
}
#[unsafe(no_mangle)]
pub extern "C" fn approximate_refresh_snapshot() -> usize {
    EXPERIMENT.with(|s| {
        let mut s = s.borrow_mut();
        let Some(s) = s.as_mut() else {
            return 0;
        };
        s.snapshot.clear();
        for b in s.world.bodies() {
            let h = b.shape.half_extents();
            let q = b.orientation;
            let role = if b.id.0 >= PROJECTILE_ID_START {
                match s.kind {
                    ProjectileType::Sphere => 3.0,
                    ProjectileType::Arrow => 4.0,
                    ProjectileType::Rigid => 5.0,
                }
            } else {
                super::role_for(b.id) as f64
            };
            s.snapshot.extend_from_slice(&[
                role,
                b.position.0,
                b.position.1,
                b.position.2,
                h.0,
                h.1,
                h.2,
                q.0,
                q.1,
                q.2,
                q.3,
            ]);
        }
        s.snapshot.as_ptr() as usize
    })
}
#[unsafe(no_mangle)]
pub extern "C" fn approximate_snapshot_len() -> usize {
    read(0, |s| s.snapshot.len())
}
#[unsafe(no_mangle)]
pub extern "C" fn approximate_body_sleeping(index: u32) -> i32 {
    read(-1, |s| {
        s.world
            .bodies()
            .nth(index as usize)
            .map_or(-1, |b| i32::from(b.is_sleeping()))
    })
}
#[unsafe(no_mangle)]
pub extern "C" fn approximate_body_velocity(index: u32, axis: u32) -> f64 {
    read(f64::NAN, |s| {
        s.world
            .bodies()
            .nth(index as usize)
            .filter(|_| axis < 3)
            .map_or(f64::NAN, |b| b.velocity.at(axis as usize))
    })
}
/// Read-only diagnostics for the stability experiment, outside the timed step.
#[unsafe(no_mangle)]
pub extern "C" fn approximate_body_kinetic_energy(index: u32) -> f64 {
    read(f64::NAN, |s| {
        s.world
            .bodies()
            .nth(index as usize)
            .map_or(f64::NAN, Body::kinetic_energy)
    })
}
#[unsafe(no_mangle)]
pub extern "C" fn approximate_body_angular_velocity(index: u32, axis: u32) -> f64 {
    read(f64::NAN, |s| {
        s.world
            .bodies()
            .nth(index as usize)
            .filter(|_| axis < 3)
            .map_or(f64::NAN, |b| b.angular_velocity.at(axis as usize))
    })
}
#[unsafe(no_mangle)]
pub extern "C" fn approximate_is_quiescent() -> i32 {
    read(-1, |s| i32::from(s.world.is_quiescent()))
}
/// Stats: 0=time; 1=substeps; 2=pair tests; 3=narrow tests; 4=points; 5=iterations;
/// 6=integrations; 7=woken bodies; 8=swept points; 9=retired total; 10=max penetration.
/// 11=response preparations; 12=inertia preparations; 13=inertia applications.
/// 14..=23: active rebuilds/scans, adjacency rebuilds/edges, island bodies/edges,
/// bound updates/sorted rows, tracked scratch capacity grows/retained bytes.
/// 24: bound-order comparisons (even when a full sort is skipped).
/// 25..=39: contact-geometry reuse counters; see docs/contact-geometry-reuse.md.
/// 40..=51: bounded convergence work/exit diagnostics; see docs/velocity-convergence.md.
/// 52..=57: soft points, relaxation passes/visits/residual visits/skipped passes/retained bytes.
/// 58: opt-in hard fixed-support points.
#[unsafe(no_mangle)]
pub extern "C" fn approximate_stat(index: u32) -> f64 {
    read(f64::NAN, |s| {
        let r = &s.world.last_report;
        match index {
            0 => s.world.elapsed_seconds(),
            1 => r.substeps as f64,
            2 => r.pair_tests as f64,
            3 => r.narrow_tests as f64,
            4 => r.contact_points as f64,
            5 => r.impulse_iterations as f64,
            6 => r.integrated_bodies as f64,
            7 => r.woken_bodies as f64,
            8 => r.swept_contacts as f64,
            9 => s.retired as f64,
            10 => r.max_penetration,
            11 => r.response_preparations as f64,
            12 => r.inertia_preparations as f64,
            13 => r.inertia_applications as f64,
            14 => r.bookkeeping.active_view_rebuilds as f64,
            15 => r.bookkeeping.active_body_scans as f64,
            16 => r.bookkeeping.adjacency_rebuilds as f64,
            17 => r.bookkeeping.adjacency_edges_indexed as f64,
            18 => r.bookkeeping.island_body_visits as f64,
            19 => r.bookkeeping.island_edge_visits as f64,
            20 => r.bookkeeping.bounds_updates as f64,
            21 => r.bookkeeping.bound_rows_sorted as f64,
            22 => r.bookkeeping.scratch_growths as f64,
            23 => r.bookkeeping.scratch_retained_bytes as f64,
            24 => r.bookkeeping.bounds_order_checks as f64,
            25 => r.geometry.current_queries as f64,
            26 => r.geometry.manifold_hits as f64,
            27 => r.geometry.negative_hits as f64,
            28 => r.geometry.manifold_refreshes as f64,
            29 => r.geometry.frame_preparations as f64,
            30 => r.geometry.frame_reuses as f64,
            31 => r.geometry.projection_preparations as f64,
            32 => r.geometry.projection_reuses as f64,
            33 => r.geometry.sat_queries as f64,
            34 => r.geometry.sat_axes_tested as f64,
            35 => r.geometry.clip_passes as f64,
            36 => r.geometry.sweep_queries as f64,
            37 => r.geometry.pair_invalidations as f64,
            38 => r.geometry.cached_pairs_peak as f64,
            39 => r.geometry.retained_bytes as f64,
            40 => r.convergence.constraint_visits as f64,
            41 => r.convergence.residual_checks as f64,
            42 => r.convergence.residual_constraint_visits as f64,
            43 => r.convergence.converged_substeps as f64,
            44 => r.convergence.capped_substeps as f64,
            45 => r.convergence.empty_substeps as f64,
            46 => r.convergence.skipped_iterations as f64,
            47 => r.convergence.max_exit_impulse_delta,
            48 => r.convergence.max_exit_velocity_residual,
            49 => r.convergence.fixed_substeps as f64,
            50 => r.convergence.probe_passes as f64,
            51 => r.convergence.delta_constraint_checks as f64,
            52 => r.correction.softened_points as f64,
            53 => r.correction.relaxation_iterations as f64,
            54 => r.correction.relaxation_constraint_visits as f64,
            55 => r.correction.relaxation_residual_visits as f64,
            56 => r.correction.relaxation_skipped_iterations as f64,
            57 => r.correction.relaxation_motion_bytes as f64,
            58 => r.correction.hard_support_points as f64,
            _ => f64::NAN,
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn correction_reset_rejects_invalid_options_and_observers_do_not_mutate() {
        assert_eq!(
            controller::baking::sandbox_reset_tower_with_baking_options(0, 0, 1),
            0
        );
        assert_eq!(approximate_reset_soft_from_sandbox(4, 8, 60.0, 1.0, 2), 0);
        assert_eq!(approximate_step_velocity(0, 0, 0), 0);
        let before = read(Vec::new(), |s| {
            s.world.bodies().cloned().collect::<Vec<_>>()
        });
        let elapsed = approximate_stat(0);
        for (f, z, r) in [
            (f64::NAN, 1.0, 2),
            (60.0, f64::INFINITY, 2),
            (0.0, 1.0, 2),
            (60.0, -1.0, 2),
            (60.0, 1.0, -1),
            (60.0, 1.0, 9),
        ] {
            assert_eq!(approximate_reset_soft_from_sandbox(4, 8, f, z, r), -1);
            assert_eq!(approximate_stat(0), elapsed);
        }
        for i in 0..before.len() as u32 {
            assert!(approximate_body_kinetic_energy(i).is_finite());
            for axis in 0..3 {
                assert!(approximate_body_angular_velocity(i, axis).is_finite());
            }
        }
        assert!(approximate_body_kinetic_energy(u32::MAX).is_nan());
        assert!(approximate_body_angular_velocity(0, 3).is_nan());
        assert_eq!(
            read(Vec::new(), |s| s
                .world
                .bodies()
                .cloned()
                .collect::<Vec<_>>()),
            before
        );
        assert_eq!(approximate_reset_from_sandbox(4, 8), 0);
        assert!(read(false, |s| s.world.config().soft_contact.is_none()));
    }
    #[test]
    fn real_tower_adapter_retains_float_state_through_all_projectile_impacts() {
        for upright in [0, 1] {
            for kind in 0..3 {
                assert_eq!(
                    controller::baking::sandbox_reset_tower_with_baking_options(0, upright, 1),
                    0
                );
                assert_eq!(approximate_reset_from_sandbox(4, 8), 0);
                for _ in 0..240 {
                    assert_eq!(approximate_step_velocity(0, 0, 0), 0);
                }
                assert_eq!(approximate_is_quiescent(), 1, "settled {upright}/{kind}");
                assert_eq!(approximate_set_projectile_type(kind), 0);
                assert!(approximate_shoot(0, 0, -96) > 0);
                for tick in 0..240 {
                    assert_eq!(
                        approximate_step_velocity(0, 0, 0),
                        0,
                        "impact {upright}/{kind}/{tick}"
                    );
                }
                assert!(approximate_stat(0) >= 7.99);
            }
        }
    }
}
