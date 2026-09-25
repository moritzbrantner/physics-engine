//! Diagnostic mixed contact fixture. Not a second physics authority or production Pages module.
//! Scope0 = prior whole-world stopping; scope1 = independent contact islands. Same scene and tick.
#[cfg(not(feature = "baseline"))]
use physics_engine::approximate::ConvergenceScope;
use physics_engine::{
    BodyId,
    approximate::{Body, Config, Quaternion, Shape, Vector as V, World},
};
use std::cell::RefCell;

struct Experiment {
    world: World,
    tick: u32,
    scene: u32,
    snapshots: Vec<f64>,
    error: i32,
}
std::thread_local! {static EXP: RefCell<Option<Experiment>> = const { RefCell::new(None) };}

/// Scenes: 0 mixed rest, 1 mixed impact, 2 independent only, 3 hard-only impact,
/// 4 sleeping scene/near miss, 5 real swept bridge between two supports.
fn fixture(scene: u32, easy: u32, scope: u32) -> Result<World, &'static str> {
    if scene > 5 || easy > 512 || scope > 1 {
        return Err("invalid fixture");
    }
    #[cfg(feature = "baseline")]
    if scope != 0 {
        return Err("baseline only supports whole-world scope");
    }
    let mut w = World::new(Config {
        fixed_position_iterations: 2,
        #[cfg(not(feature = "baseline"))]
        convergence_scope: if scope == 0 {
            ConvergenceScope::WholeWorld
        } else {
            ConvergenceScope::ContactIslands
        },
        ..Config::default()
    })
    .map_err(|_| "config")?;
    let mut floor = Body::new(
        BodyId(0),
        Shape::Box(V(10000.0, 10.0, 10000.0)),
        V(0.0, -10.0, 0.0),
        0.0,
    );
    floor.friction = 1.0;
    w.add_body(floor).map_err(|_| "floor")?;
    let sleeping = scene == 4 || scene == 5;
    if scene != 2 && scene != 5 {
        for y in 0..4 {
            for x in 0..4 {
                for z in 0..2 {
                    let mut b = Body::new(
                        BodyId(1 + y * 8 + x * 2 + z),
                        Shape::Box(V(18.0, 18.0, 18.0)),
                        V(x as f64 * 36.0, 18.0 + y as f64 * 36.0, z as f64 * 36.0),
                        2.0,
                    );
                    b.friction = 1.0;
                    b.sleep_allowed = sleeping;
                    w.add_body(b).map_err(|_| "tower")?;
                }
            }
        }
    }
    if scene == 5 {
        for (id, x) in [(1, -36.0), (2, 36.0)] {
            let mut b = Body::new(
                BodyId(id),
                Shape::Box(V(18.0, 18.0, 18.0)),
                V(x, 18.0, 0.0),
                2.0,
            );
            b.friction = 1.0;
            w.add_body(b).map_err(|_| "bridge")?;
        }
    }
    if scene != 3 {
        for n in 0..easy {
            let mut b = Body::new(
                BodyId(100 + n as u64),
                Shape::Box(V(18.0, 18.0, 18.0)),
                V(500.0 + (n % 16) as f64 * 80.0, 18.0, (n / 16) as f64 * 80.0),
                2.0,
            );
            b.friction = 1.0;
            b.sleep_allowed = sleeping;
            w.add_body(b).map_err(|_| "easy")?;
        }
    }
    Ok(w)
}

#[unsafe(no_mangle)]
pub extern "C" fn island_reset(scene: u32, easy: u32, scope: u32) -> i32 {
    let Ok(world) = fixture(scene, easy, scope) else {
        return -1;
    };
    EXP.with(|s| {
        *s.borrow_mut() = Some(Experiment {
            world,
            tick: 0,
            scene,
            snapshots: Vec::new(),
            error: 0,
        })
    });
    0
}

fn tick(e: &mut Experiment) -> Result<(), &'static str> {
    // All variants receive identical external inputs. Scheduling/insertion costs are timed.
    if e.tick == 240 && [1, 3, 4, 5].contains(&e.scene) {
        let (shape, position, velocity) = match e.scene {
            4 => (
                Shape::Sphere(4.0),
                V(-200.0, 40.0, -200.0),
                V(0.0, 0.0, -1200.0),
            ),
            5 => (
                Shape::Box(V(54.0, 4.0, 4.0)),
                V(0.0, 40.0, -120.0),
                V(0.0, 0.0, 2400.0),
            ),
            _ => (
                Shape::Sphere(4.0),
                V(54.0, 40.0, -120.0),
                V(0.0, 0.0, 2400.0),
            ),
        };
        let mut p = Body::new(BodyId(2000), shape, position, 0.25);
        p.velocity = velocity;
        p.ccd = true;
        p.retire_on_impact = true;
        e.world.add_body(p).map_err(|_| "projectile")?;
    }
    e.world.step(1.0 / 60.0).map_err(|_| "step")?;
    e.tick += 1;
    Ok(())
}
#[unsafe(no_mangle)]
pub extern "C" fn island_step() -> i32 {
    EXP.with(|slot| {
        let mut slot = slot.borrow_mut();
        let Some(e) = slot.as_mut() else {
            return -1;
        };
        if e.error != 0 {
            return e.error;
        }
        if tick(e).is_err() {
            e.error = -2;
        }
        e.error
    })
}
fn read(default: f64, f: impl FnOnce(&Experiment) -> f64) -> f64 {
    EXP.with(|s| s.borrow().as_ref().map_or(default, f))
}
#[unsafe(no_mangle)]
pub extern "C" fn island_stat(index: u32) -> f64 {
    read(f64::NAN, |e| {
        let r = &e.world.last_report;
        match index {
            0 => e.world.elapsed_seconds(),
            1 => r.substeps as f64,
            2 => r.impulse_iterations as f64,
            3 => r.convergence.constraint_visits as f64,
            4 => r.convergence.residual_constraint_visits as f64,
            5 => r.convergence.skipped_iterations as f64,
            6 => r.contact_points as f64,
            7 => r.narrow_tests as f64,
            8 => r.woken_bodies as f64,
            9 => r.retired.len() as f64,
            10 => r.position.max_distance,
            11 => r.convergence.max_exit_velocity_residual,
            12 => r.convergence.max_exit_impulse_delta,
            13 => r.swept_contacts as f64,
            14 => f64::from(e.world.is_quiescent()),
            15 => e.tick as f64,
            #[cfg(not(feature = "baseline"))]
            16..=33 => {
                let s = r.islands;
                [
                    s.partition_builds,
                    s.rows_indexed,
                    s.endpoints_checked,
                    s.dynamic_nodes,
                    s.union_attempts,
                    s.root_links_followed,
                    s.islands,
                    s.max_island_rows,
                    s.single_island_substeps,
                    s.island_iterations,
                    s.converged_islands,
                    s.capped_islands,
                    s.skipped_constraint_visits,
                    s.scratch_growths,
                    s.scratch_retained_bytes,
                    s.epoch_resets,
                    s.shared_prefix_iterations,
                    s.prefix_converged_substeps,
                ][(index - 16) as usize] as f64
            }
            _ => f64::NAN,
        }
    })
}

fn bottom(b: &Body) -> f64 {
    let h = b.shape.half_extents();
    let depth = match b.shape {
        Shape::Sphere(r) => r,
        Shape::Capsule {
            half_segment,
            radius,
        } => b.orientation.rotate(V::Y).1.abs() * half_segment + radius,
        Shape::Cylinder {
            half_height,
            radius,
        } => {
            let axis_y = b.orientation.rotate(V::Y).1;
            axis_y.abs() * half_height + (1.0 - axis_y * axis_y).max(0.0).sqrt() * radius
        }
        Shape::Box(_) | Shape::Wedge(_) => {
            b.orientation.rotate(V::X).1.abs() * h.0
                + b.orientation.rotate(V::Y).1.abs() * h.1
                + b.orientation.rotate(V::Z).1.abs() * h.2
        }
    };
    b.position.1 - depth
}

/// Full authoritative pose/velocities plus sleep/ground-quality observations; 16 scalars per body.
#[unsafe(no_mangle)]
pub extern "C" fn island_snapshot() -> *const f64 {
    EXP.with(|s| {
        let mut s = s.borrow_mut();
        let Some(e) = s.as_mut() else {
            return std::ptr::null();
        };
        e.snapshots.clear();
        for b in e.world.bodies() {
            let Quaternion(x, y, z, w) = b.orientation;
            e.snapshots.extend_from_slice(&[
                b.id.0 as f64,
                b.position.0,
                b.position.1,
                b.position.2,
                x,
                y,
                z,
                w,
                b.velocity.0,
                b.velocity.1,
                b.velocity.2,
                b.angular_velocity.0,
                b.angular_velocity.1,
                b.angular_velocity.2,
                f64::from(b.is_sleeping()),
                bottom(b),
            ]);
        }
        e.snapshots.as_ptr()
    })
}
#[unsafe(no_mangle)]
pub extern "C" fn island_snapshot_len() -> u32 {
    read(0.0, |e| e.snapshots.len() as f64) as u32
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn invalid_reset_does_not_replace_a_live_world() {
        assert_eq!(island_reset(0, 8, 0), 0);
        assert_eq!(island_step(), 0);
        assert_eq!(island_reset(6, 8, 0), -1);
        assert_eq!(island_stat(15), 1.0);
        assert_eq!(island_reset(0, 513, 0), -1);
        assert_eq!(island_reset(0, 8, 2), -1);
    }
}
