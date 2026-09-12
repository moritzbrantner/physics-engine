use std::cell::RefCell;

use physics_engine::{
    AngularState3d, AngularVelocity3d, BodyId, Material, Orientation3d, OrientedBox3d, RigidBody,
    RigidBox3d, RotatingWorld3d, RotatingWorldConfig3d, RotatingWorldError3d, Vec3i,
};

const PLAYER_ID: BodyId = BodyId(1);
const PROJECTILE_ID_START: u64 = 1_000;
const MAX_PROJECTILES: usize = 48;
const MOVE_SPEED: i32 = 7;
const JUMP_SPEED: i32 = 16;
const PROJECTILE_SPEED_LIMIT: i32 = 120;
const ROTATING_TICKS_PER_SECOND: i32 = 60;
const CRATE_RESTITUTION_MILLI: u16 = 50;
const CRATE_FRICTION_MILLI: u16 = 850;

struct Sandbox {
    world: RotatingWorld3d,
    next_projectile_id: u64,
    projectile_ids: Vec<BodyId>,
    last_rotating_events: usize,
    last_tail_contacts: usize,
    total_collisions: u32,
    error_code: i32,
}

impl Sandbox {
    fn new() -> Result<Self, RotatingWorldError3d> {
        let mut world = RotatingWorld3d::new(RotatingWorldConfig3d {
            gravity: Vec3i::new(0, -3_600, 0),
            sample_count: 32,
            refinement_steps: 4,
            solver_passes: 8,
            max_events: 32,
        });

        let fixed_bodies = [
            (10, Vec3i::new(0, -16, 0), Vec3i::new(520, 16, 520)),
            (11, Vec3i::new(0, 72, -520), Vec3i::new(520, 72, 8)),
            (12, Vec3i::new(0, 72, 520), Vec3i::new(520, 72, 8)),
            (13, Vec3i::new(-520, 72, 0), Vec3i::new(8, 72, 520)),
            (14, Vec3i::new(520, 72, 0), Vec3i::new(8, 72, 520)),
            // Deliberately thin target for sampled rotating CCD acceptance.
            (15, Vec3i::new(0, 72, -180), Vec3i::new(120, 72, 3)),
            (20, Vec3i::new(-190, 24, 40), Vec3i::new(70, 24, 70)),
            (21, Vec3i::new(185, 8, 75), Vec3i::new(45, 8, 45)),
            (22, Vec3i::new(185, 16, 20), Vec3i::new(45, 16, 45)),
            (23, Vec3i::new(185, 24, -35), Vec3i::new(45, 24, 45)),
            (24, Vec3i::new(185, 32, -90), Vec3i::new(45, 32, 45)),
        ];
        for (id, position, half_extents) in fixed_bodies {
            world.add_box(rotating_box(RigidBody::fixed(
                BodyId(id),
                position,
                half_extents,
            )))?;
        }

        world.add_box(
            rotating_box(
                RigidBody::dynamic(
                    PLAYER_ID,
                    Vec3i::new(0, 38, 320),
                    Vec3i::ZERO,
                    Vec3i::new(12, 20, 12),
                )
                .with_mass(4),
            )
            .with_rotation_locked(),
        )?;

        let crate_positions = [
            Vec3i::new(-75, 18, 135),
            Vec3i::new(-75, 54, 135),
            Vec3i::new(80, 18, 120),
            Vec3i::new(116, 18, 120),
            Vec3i::new(98, 54, 120),
            Vec3i::new(0, 18, -70),
        ];
        for (offset, position) in crate_positions.into_iter().enumerate() {
            world.add_box(rotating_box(
                RigidBody::dynamic(
                    BodyId(100 + offset as u64),
                    position,
                    Vec3i::ZERO,
                    Vec3i::new(18, 18, 18),
                )
                .with_mass(2)
                .with_material(
                    Material::new(CRATE_RESTITUTION_MILLI).with_friction(CRATE_FRICTION_MILLI),
                ),
            ))?;
        }

        Ok(Self {
            world,
            next_projectile_id: PROJECTILE_ID_START,
            projectile_ids: Vec::new(),
            last_rotating_events: 0,
            last_tail_contacts: 0,
            total_collisions: 0,
            error_code: 0,
        })
    }

    fn grounded(&self) -> Result<bool, RotatingWorldError3d> {
        let player = self
            .world
            .box_by_id(PLAYER_ID)
            .ok_or(RotatingWorldError3d::MissingBody(PLAYER_ID))?;
        let position = player.body().position();
        let half = player.body().half_extents();
        let probe = OrientedBox3d::new(
            Vec3i::new(position.x, position.y - half.y - 1, position.z),
            Vec3i::new(
                half.x.saturating_sub(1).max(1),
                1,
                half.z.saturating_sub(1).max(1),
            ),
            Orientation3d::IDENTITY,
        );
        Ok(self
            .world
            .overlap_query(probe)?
            .into_iter()
            .any(|id| id != PLAYER_ID))
    }

    fn step(&mut self, move_x: i32, move_z: i32, jump: bool) -> i32 {
        self.error_code = 0;
        let Some(player) = self.world.box_by_id(PLAYER_ID) else {
            self.error_code = 1;
            return self.error_code;
        };
        let current_velocity = player.body().velocity();
        let grounded = match self.grounded() {
            Ok(value) => value,
            Err(_) => {
                self.error_code = 2;
                return self.error_code;
            }
        };
        let next_y = if jump && grounded {
            JUMP_SPEED.saturating_mul(ROTATING_TICKS_PER_SECOND)
        } else {
            current_velocity.y
        };
        let velocity = Vec3i::new(
            move_x
                .clamp(-MOVE_SPEED, MOVE_SPEED)
                .saturating_mul(ROTATING_TICKS_PER_SECOND),
            next_y,
            move_z
                .clamp(-MOVE_SPEED, MOVE_SPEED)
                .saturating_mul(ROTATING_TICKS_PER_SECOND),
        );
        if velocity != current_velocity
            && self.world.set_linear_velocity(PLAYER_ID, velocity).is_err()
        {
            self.error_code = 3;
            return self.error_code;
        }

        let report = match self.world.step(1, ROTATING_TICKS_PER_SECOND) {
            Ok(report) => report,
            Err(_) => {
                self.error_code = 6;
                return self.error_code;
            }
        };

        self.last_rotating_events = report.stats.sampled_events;
        self.last_tail_contacts = report.stats.tail_contacts;
        let collisions = self
            .last_rotating_events
            .saturating_add(self.last_tail_contacts);
        self.total_collisions = self
            .total_collisions
            .saturating_add(u32::try_from(collisions).unwrap_or(u32::MAX));
        self.cleanup_projectiles();
        0
    }

    fn shoot(&mut self, velocity_x: i32, velocity_y: i32, velocity_z: i32) -> i32 {
        let input_velocity = Vec3i::new(
            velocity_x.clamp(-PROJECTILE_SPEED_LIMIT, PROJECTILE_SPEED_LIMIT),
            velocity_y.clamp(-PROJECTILE_SPEED_LIMIT, PROJECTILE_SPEED_LIMIT),
            velocity_z.clamp(-PROJECTILE_SPEED_LIMIT, PROJECTILE_SPEED_LIMIT),
        );
        if input_velocity == Vec3i::ZERO {
            return -1;
        }
        let Some(player) = self.world.box_by_id(PLAYER_ID) else {
            return -1;
        };
        let player_position = player.body().position();
        let spawn = Vec3i::new(
            player_position.x + input_velocity.x / 3,
            player_position.y + 12 + input_velocity.y / 3,
            player_position.z + input_velocity.z / 3,
        );
        let velocity = Vec3i::new(
            input_velocity.x.saturating_mul(ROTATING_TICKS_PER_SECOND),
            input_velocity.y.saturating_mul(ROTATING_TICKS_PER_SECOND),
            input_velocity.z.saturating_mul(ROTATING_TICKS_PER_SECOND),
        );
        let id = BodyId(self.next_projectile_id);
        self.next_projectile_id = self.next_projectile_id.saturating_add(1);

        if self
            .world
            .add_box(rotating_box(
                RigidBody::dynamic(id, spawn, velocity, Vec3i::new(3, 3, 3))
                    .with_material(Material::new(350)),
            ))
            .is_err()
        {
            self.error_code = 5;
            return -1;
        }
        self.projectile_ids.push(id);
        if self.projectile_ids.len() > MAX_PROJECTILES {
            let oldest = self.projectile_ids.remove(0);
            self.world.remove_box(oldest);
        }
        i32::try_from(id.0).unwrap_or(i32::MAX)
    }

    fn cleanup_projectiles(&mut self) {
        let stale = self
            .projectile_ids
            .iter()
            .copied()
            .filter(|id| {
                self.world.box_by_id(*id).is_none_or(|rigid_box| {
                    let position = rigid_box.body().position();
                    position.x.abs() > 1_200
                        || position.y < -300
                        || position.y > 900
                        || position.z.abs() > 1_200
                })
            })
            .collect::<Vec<_>>();
        for id in stale {
            self.world.remove_box(id);
            self.projectile_ids.retain(|candidate| *candidate != id);
        }
    }

    fn body_count(&self) -> usize {
        self.world.boxes().count()
    }

    fn body_at(&self, index: u32) -> Option<&RigidBody> {
        self.world.boxes().nth(index as usize).map(RigidBox3d::body)
    }

    fn angular_at(&self, index: u32) -> AngularState3d {
        self.world.boxes().nth(index as usize).map_or(
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
            RigidBox3d::angular,
        )
    }
}

fn rotating_box(body: RigidBody) -> RigidBox3d {
    RigidBox3d::new(
        body,
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("sandbox rotating body must be valid")
}

fn role_for(id: BodyId) -> i32 {
    if id == PLAYER_ID {
        1
    } else if id.0 >= PROJECTILE_ID_START {
        3
    } else if id.0 >= 100 {
        2
    } else {
        0
    }
}

std::thread_local! {
    static SANDBOX: RefCell<Sandbox> = RefCell::new(
        Sandbox::new().expect("the built-in physics sandbox must be valid")
    );
}

fn with_sandbox<R>(callback: impl FnOnce(&Sandbox) -> R) -> R {
    SANDBOX.with(|sandbox| callback(&sandbox.borrow()))
}

fn with_sandbox_mut<R>(callback: impl FnOnce(&mut Sandbox) -> R) -> R {
    SANDBOX.with(|sandbox| callback(&mut sandbox.borrow_mut()))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_reset() {
    with_sandbox_mut(|sandbox| {
        *sandbox = Sandbox::new().expect("the built-in physics sandbox must be valid");
    });
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_step(move_x: i32, move_z: i32, jump: i32) -> i32 {
    with_sandbox_mut(|sandbox| sandbox.step(move_x, move_z, jump != 0))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_shoot(velocity_x: i32, velocity_y: i32, velocity_z: i32) -> i32 {
    with_sandbox_mut(|sandbox| sandbox.shoot(velocity_x, velocity_y, velocity_z))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_body_count() -> u32 {
    with_sandbox(|sandbox| u32::try_from(sandbox.body_count()).unwrap_or(u32::MAX))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_body_role(index: u32) -> i32 {
    with_sandbox(|sandbox| {
        sandbox
            .body_at(index)
            .map(|body| role_for(body.id()))
            .unwrap_or(-1)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_body_x(index: u32) -> i32 {
    with_sandbox(|sandbox| sandbox.body_at(index).map_or(0, |body| body.position().x))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_body_y(index: u32) -> i32 {
    with_sandbox(|sandbox| sandbox.body_at(index).map_or(0, |body| body.position().y))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_body_z(index: u32) -> i32 {
    with_sandbox(|sandbox| sandbox.body_at(index).map_or(0, |body| body.position().z))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_body_half_x(index: u32) -> i32 {
    with_sandbox(|sandbox| {
        sandbox
            .body_at(index)
            .map_or(0, |body| body.half_extents().x)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_body_half_y(index: u32) -> i32 {
    with_sandbox(|sandbox| {
        sandbox
            .body_at(index)
            .map_or(0, |body| body.half_extents().y)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_body_half_z(index: u32) -> i32 {
    with_sandbox(|sandbox| {
        sandbox
            .body_at(index)
            .map_or(0, |body| body.half_extents().z)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_body_orientation_x(index: u32) -> i32 {
    with_sandbox(|sandbox| sandbox.angular_at(index).orientation.x)
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_body_orientation_y(index: u32) -> i32 {
    with_sandbox(|sandbox| sandbox.angular_at(index).orientation.y)
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_body_orientation_z(index: u32) -> i32 {
    with_sandbox(|sandbox| sandbox.angular_at(index).orientation.z)
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_body_orientation_w(index: u32) -> i32 {
    with_sandbox(|sandbox| sandbox.angular_at(index).orientation.w)
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_collision_events() -> u32 {
    with_sandbox(|sandbox| {
        u32::try_from(
            sandbox
                .last_rotating_events
                .saturating_add(sandbox.last_tail_contacts),
        )
        .unwrap_or(u32::MAX)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_total_collisions() -> u32 {
    with_sandbox(|sandbox| sandbox.total_collisions)
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_grounded() -> i32 {
    with_sandbox(|sandbox| match sandbox.grounded() {
        Ok(true) => 1,
        Ok(false) | Err(_) => 0,
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_error_code() -> i32 {
    with_sandbox(|sandbox| sandbox.error_code)
}

#[cfg(test)]
mod tests {
    use physics_engine::{BodyId, RigidBody, Vec3i};

    use super::{PLAYER_ID, Sandbox, rotating_box};

    fn settle_player(sandbox: &mut Sandbox) {
        for _ in 0..16 {
            assert_eq!(sandbox.step(0, 0, false), 0);
        }
    }

    #[test]
    fn sandbox_contains_rotation_locked_player_in_unified_world() {
        let sandbox = Sandbox::new().expect("valid sandbox");
        let player = sandbox.world.box_by_id(PLAYER_ID).expect("player");
        assert!(player.rotation_locked());
        assert!(player.angular().angular_velocity.is_zero());
        assert!(sandbox.world.boxes().count() >= 10);
    }

    #[test]
    fn grounded_player_can_jump() {
        let mut sandbox = Sandbox::new().expect("valid sandbox");
        settle_player(&mut sandbox);
        assert!(sandbox.grounded().expect("valid foot probe"));
        let before = sandbox
            .world
            .box_by_id(PLAYER_ID)
            .expect("player")
            .body()
            .position()
            .y;

        assert_eq!(sandbox.step(0, 0, true), 0);

        let player = sandbox
            .world
            .box_by_id(PLAYER_ID)
            .expect("player after jump");
        assert!(
            player.body().position().y > before,
            "jump did not move upward"
        );
        assert!(
            player.body().velocity().y > 0,
            "jump did not preserve upward velocity"
        );
        assert!(!sandbox.grounded().expect("valid airborne foot probe"));
        assert!(player.rotation_locked());
        assert!(player.angular().angular_velocity.is_zero());
    }

    #[test]
    fn player_pushes_dynamic_box_without_tumbling() {
        let mut sandbox = Sandbox::new().expect("valid sandbox");
        settle_player(&mut sandbox);
        let crate_id = BodyId(900);
        sandbox
            .world
            .add_box(rotating_box(
                RigidBody::dynamic(
                    crate_id,
                    Vec3i::new(0, 18, 285),
                    Vec3i::ZERO,
                    Vec3i::new(18, 18, 18),
                )
                .with_mass(2),
            ))
            .expect("test crate");
        let before = sandbox
            .world
            .box_by_id(crate_id)
            .expect("test crate")
            .body()
            .position()
            .z;

        for _ in 0..8 {
            assert_eq!(sandbox.step(0, -7, false), 0);
        }

        let crate_z = sandbox
            .world
            .box_by_id(crate_id)
            .expect("test crate after push")
            .body()
            .position()
            .z;
        let player = sandbox
            .world
            .box_by_id(PLAYER_ID)
            .expect("player after push");
        assert!(crate_z < before, "player did not push the dynamic crate");
        assert!(player.rotation_locked());
        assert!(player.angular().angular_velocity.is_zero());
        assert_eq!(
            player.angular().orientation,
            physics_engine::Orientation3d::IDENTITY
        );
    }

    #[test]
    fn fast_projectile_does_not_tunnel_through_thin_target() {
        let mut sandbox = Sandbox::new().expect("valid sandbox");
        let projectile = sandbox.shoot(0, 0, -96);
        assert!(projectile >= 0);

        for _ in 0..7 {
            assert_eq!(sandbox.step(0, 0, false), 0);
        }

        let projectile = sandbox
            .world
            .box_by_id(physics_engine::BodyId(projectile as u64))
            .expect("projectile remains in the bounded test world");
        assert!(
            projectile.body().position().z > -190,
            "projectile tunneled through the thin sampled-CCD target: {:?}",
            projectile.body().position()
        );
    }
}
