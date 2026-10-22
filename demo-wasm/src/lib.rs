//! Narrow WASM adapter for the Pages tower lab. Rust owns every body, contact and step; the page
//! only sends commands and reads a render snapshot.

use physics_engine::{
    BodyId,
    approximate::{Body, Config, PositionCorrection, Quaternion, Real, Shape, Vector, World},
};
use std::cell::RefCell;

const PLAYER_KEY: u64 = 1;
const CRATE_ID_START: u64 = 100;
const MAX_PROJECTILES: usize = 48;
/// Floats per snapshot row: role, position (3), half extents (3), orientation (4).
pub const SNAPSHOT_STRIDE: usize = 11;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Projectile {
    Sphere,
    Arrow,
    Rigid,
}

struct Lab {
    world: World,
    kind: Projectile,
    projectiles: Vec<(BodyId, Projectile)>,
    snapshot: Vec<f64>,
}

std::thread_local! { static LAB: RefCell<Option<Lab>> = const { RefCell::new(None) }; }

fn with_lab<T>(default: T, f: impl FnOnce(&mut Lab) -> T) -> T {
    LAB.with(|lab| lab.borrow_mut().as_mut().map_or(default, f))
}

fn fixed(key: u64, position: Vector, half: Vector) -> Body {
    Body::keyed(key, Shape::Box(half), position, 0.0)
}

/// Builds the 32-crate tower in a walled room with a rotation-locked player box.
pub fn tower(upright_crates: bool) -> Result<World, physics_engine::approximate::Error> {
    let mut world = World::new(Config {
        // The tower is in scene units of 36-unit crates, tuned with these settings.
        gravity: Vector(0.0, -3_600.0, 0.0),
        contact_slop: 0.02,
        sleep_speed: 1.0,
        substeps: 4,
        velocity_iterations: 8,
        fixed_position_iterations: 2,
        position_correction: PositionCorrection::AdmittedContacts,
        ..Config::default()
    })?;
    let room = [
        (10, Vector(0.0, -16.0, 0.0), Vector(520.0, 16.0, 520.0)),
        (11, Vector(0.0, 72.0, -520.0), Vector(520.0, 72.0, 8.0)),
        (12, Vector(0.0, 72.0, 520.0), Vector(520.0, 72.0, 8.0)),
        (13, Vector(-520.0, 72.0, 0.0), Vector(8.0, 72.0, 520.0)),
        (14, Vector(520.0, 72.0, 0.0), Vector(8.0, 72.0, 520.0)),
    ];
    for (id, position, half) in room {
        world.add_body(fixed(id, position, half))?;
    }
    let mut player = Body::keyed(
        PLAYER_KEY,
        Shape::Box(Vector(12.0, 20.0, 12.0)),
        Vector(0.0, 38.0, 320.0),
        4.0,
    );
    player.rotation_locked = true;
    world.add_body(player)?;
    let mut id = CRATE_ID_START;
    for level in 0..4 {
        for depth in 0..2 {
            for column in 0..4 {
                let position = Vector(
                    -54.0 + column as Real * 36.0,
                    18.0 + level as Real * 36.0,
                    82.0 + depth as Real * 36.0,
                );
                let mut body = Body::keyed(id, Shape::Box(Vector(18.0, 18.0, 18.0)), position, 2.0);
                body.friction = 1.0;
                body.rotation_locked = upright_crates;
                world.add_body(body)?;
                id += 1;
            }
        }
    }
    Ok(world)
}

/// Resets the lab. `upright` locks crate rotation. Returns 0 on success.
#[unsafe(no_mangle)]
pub extern "C" fn lab_reset(upright: i32) -> i32 {
    match tower(upright != 0) {
        Ok(world) => {
            LAB.with(|lab| {
                *lab.borrow_mut() = Some(Lab {
                    world,
                    kind: Projectile::Arrow,
                    projectiles: Vec::new(),
                    snapshot: Vec::new(),
                });
            });
            0
        }
        Err(_) => -1,
    }
}

/// 0 = sphere, 1 = arrow, 2 = rigid box.
#[unsafe(no_mangle)]
pub extern "C" fn lab_set_projectile(kind: i32) -> i32 {
    let kind = match kind {
        0 => Projectile::Sphere,
        1 => Projectile::Arrow,
        2 => Projectile::Rigid,
        _ => return -1,
    };
    with_lab(-1, |lab| {
        lab.kind = kind;
        0
    })
}

/// Fires a projectile from the player in direction `(x, y, z)`. Returns its id or -1.
#[unsafe(no_mangle)]
pub extern "C" fn lab_shoot(x: f32, y: f32, z: f32) -> i32 {
    with_lab(-1, |lab| {
        let direction = Vector(x, y, z);
        if !direction.finite() || direction == Vector::ZERO {
            return -1;
        }
        let Some(player) = lab
            .world
            .body_id(PLAYER_KEY)
            .and_then(|id| lab.world.body(id))
        else {
            return -1;
        };
        let unit = direction.unit();
        let shape = match lab.kind {
            Projectile::Sphere => Shape::Sphere(3.0),
            Projectile::Arrow => Shape::Box(Vector(2.0, 2.0, 12.0)),
            Projectile::Rigid => Shape::Box(Vector(3.0, 3.0, 3.0)),
        };
        let mut body = Body::new(
            shape,
            player.position + Vector(0.0, 12.0, 0.0) + unit * 40.0,
            1.0,
        );
        body.velocity = unit * 5_760.0;
        body.ccd = true;
        body.sleep_allowed = false;
        body.friction = 0.0;
        if lab.kind == Projectile::Arrow {
            // Rotate local +Z onto the flight direction.
            let axis = Vector::Z.cross(unit);
            let w = 1.0 + Vector::Z.dot(unit);
            body.orientation = if w < 1e-6 {
                Quaternion(0.0, 1.0, 0.0, 0.0)
            } else {
                Quaternion(axis.0, axis.1, axis.2, w).normalized()
            };
            body.rotation_locked = true;
        }
        let Ok(id) = lab.world.add_body(body) else {
            return -1;
        };
        lab.projectiles.push((id, lab.kind));
        if lab.projectiles.len() > MAX_PROJECTILES {
            let (oldest, _) = lab.projectiles.remove(0);
            lab.world.remove_body(oldest);
        }
        id.slot() as i32
    })
}

/// Advances one 60 Hz tick. Returns 0 on success.
#[unsafe(no_mangle)]
pub extern "C" fn lab_step() -> i32 {
    with_lab(-1, |lab| match lab.world.step(1.0 / 60.0) {
        Ok(report) => {
            lab.projectiles
                .retain(|(id, _)| !report.retired.contains(id));
            let world = &mut lab.world;
            lab.projectiles.retain(|&(id, _)| {
                let out = world.body(id).is_none_or(|b| {
                    b.position.1 < -150.0 || b.position.abs().max_component() > 1_600.0
                });
                if out {
                    world.remove_body(id);
                }
                !out
            });
            0
        }
        Err(_) => 1,
    })
}

/// Fills the render snapshot and returns its address in linear memory.
#[unsafe(no_mangle)]
pub extern "C" fn lab_refresh_snapshot() -> usize {
    with_lab(0, |lab| {
        lab.snapshot.clear();
        for b in lab.world.bodies() {
            let role = if b.key == Some(PLAYER_KEY) {
                1.0
            } else if b.key.is_none() {
                match lab
                    .projectiles
                    .iter()
                    .find(|(id, _)| *id == b.id)
                    .map(|p| p.1)
                {
                    Some(Projectile::Sphere) => 3.0,
                    Some(Projectile::Arrow) => 4.0,
                    _ => 5.0,
                }
            } else if b.key.is_some_and(|k| k >= CRATE_ID_START) {
                2.0
            } else {
                0.0
            };
            let h = b.shape.half_extents();
            let q = b.orientation;
            lab.snapshot.extend(
                [role].into_iter().chain(
                    [
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
                    ]
                    .map(f64::from),
                ),
            );
        }
        lab.snapshot.as_ptr() as usize
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn lab_snapshot_len() -> usize {
    with_lab(0, |lab| lab.snapshot.len())
}

/// Number of awake crates.
#[unsafe(no_mangle)]
pub extern "C" fn lab_awake_crates() -> u32 {
    with_lab(0, |lab| {
        lab.world
            .bodies()
            .filter(|b| b.key.is_some_and(|k| k >= CRATE_ID_START) && !b.is_sleeping())
            .count() as u32
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tower_settles_and_survives_a_volley() {
        assert_eq!(lab_reset(0), 0);
        for _ in 0..240 {
            assert_eq!(lab_step(), 0);
        }
        for kind in 0..3 {
            assert_eq!(lab_set_projectile(kind), 0);
            assert!(lab_shoot(0.0, 0.0, -1.0) >= 0);
        }
        for _ in 0..240 {
            assert_eq!(lab_step(), 0);
        }
        assert!(lab_refresh_snapshot() != 0);
        assert_eq!(lab_snapshot_len() % SNAPSHOT_STRIDE, 0);
    }
}
