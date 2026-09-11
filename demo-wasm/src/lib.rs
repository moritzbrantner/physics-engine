use std::cell::RefCell;

use physics_engine::{
    Aabb, BodyId, Material, QueryError, RigidBody, StepStats, Vec3i, World, WorldConfig,
};

const PLAYER_ID: BodyId = BodyId(1);
const PROJECTILE_ID_START: u64 = 1_000;
const MAX_PROJECTILES: usize = 48;
const MOVE_SPEED: i32 = 7;
const JUMP_SPEED: i32 = 16;
const PROJECTILE_SPEED_LIMIT: i32 = 120;

struct Sandbox {
    world: World,
    next_projectile_id: u64,
    projectile_ids: Vec<BodyId>,
    last_stats: StepStats,
    total_collisions: u32,
    error_code: i32,
}

impl Sandbox {
    fn new() -> Result<Self, physics_engine::PhysicsError> {
        let mut world = World::new(WorldConfig {
            gravity: Vec3i::new(0, -1, 0),
            ..WorldConfig::default()
        });

        add_fixed(
            &mut world,
            10,
            Vec3i::new(0, -16, 0),
            Vec3i::new(520, 16, 520),
        )?;
        add_fixed(
            &mut world,
            11,
            Vec3i::new(0, 72, -520),
            Vec3i::new(520, 72, 8),
        )?;
        add_fixed(
            &mut world,
            12,
            Vec3i::new(0, 72, 520),
            Vec3i::new(520, 72, 8),
        )?;
        add_fixed(
            &mut world,
            13,
            Vec3i::new(-520, 72, 0),
            Vec3i::new(8, 72, 520),
        )?;
        add_fixed(
            &mut world,
            14,
            Vec3i::new(520, 72, 0),
            Vec3i::new(8, 72, 520),
        )?;

        // A deliberately thin target makes fast projectile CCD observable in the live demo.
        add_fixed(
            &mut world,
            15,
            Vec3i::new(0, 72, -180),
            Vec3i::new(120, 72, 3),
        )?;
        add_fixed(
            &mut world,
            20,
            Vec3i::new(-190, 24, 40),
            Vec3i::new(70, 24, 70),
        )?;
        add_fixed(
            &mut world,
            21,
            Vec3i::new(185, 8, 75),
            Vec3i::new(45, 8, 45),
        )?;
        add_fixed(
            &mut world,
            22,
            Vec3i::new(185, 16, 20),
            Vec3i::new(45, 16, 45),
        )?;
        add_fixed(
            &mut world,
            23,
            Vec3i::new(185, 24, -35),
            Vec3i::new(45, 24, 45),
        )?;
        add_fixed(
            &mut world,
            24,
            Vec3i::new(185, 32, -90),
            Vec3i::new(45, 32, 45),
        )?;

        world.add_body(
            RigidBody::dynamic(
                PLAYER_ID,
                Vec3i::new(0, 38, 320),
                Vec3i::ZERO,
                Vec3i::new(12, 20, 12),
            )
            .with_mass(4),
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
            world.add_body(
                RigidBody::dynamic(
                    BodyId(100 + offset as u64),
                    position,
                    Vec3i::ZERO,
                    Vec3i::new(18, 18, 18),
                )
                .with_mass(2)
                .with_material(Material::new(100)),
            )?;
        }

        Ok(Self {
            world,
            next_projectile_id: PROJECTILE_ID_START,
            projectile_ids: Vec::new(),
            last_stats: StepStats::default(),
            total_collisions: 0,
            error_code: 0,
        })
    }

    fn grounded(&self) -> Result<bool, QueryError> {
        let Some(player) = self.world.body(PLAYER_ID) else {
            return Ok(false);
        };
        let position = player.position();
        let half = player.half_extents();
        let probe = Aabb::new(
            Vec3i::new(position.x, position.y - half.y - 1, position.z),
            Vec3i::new(
                half.x.saturating_sub(1).max(0),
                1,
                half.z.saturating_sub(1).max(0),
            ),
        );
        Ok(self
            .world
            .overlap_query(probe)?
            .into_iter()
            .any(|id| id != PLAYER_ID))
    }

    fn step(&mut self, move_x: i32, move_z: i32, jump: bool) -> i32 {
        self.error_code = 0;
        let Some(player) = self.world.body(PLAYER_ID) else {
            self.error_code = 1;
            return self.error_code;
        };
        let current_y = player.velocity().y;
        let grounded = match self.grounded() {
            Ok(value) => value,
            Err(_) => {
                self.error_code = 2;
                return self.error_code;
            }
        };
        let next_y = if jump && grounded {
            JUMP_SPEED
        } else {
            current_y
        };
        let velocity = Vec3i::new(
            move_x.clamp(-MOVE_SPEED, MOVE_SPEED),
            next_y,
            move_z.clamp(-MOVE_SPEED, MOVE_SPEED),
        );
        if self.world.set_velocity(PLAYER_ID, velocity).is_err() {
            self.error_code = 3;
            return self.error_code;
        }

        let report = match self.world.step(1) {
            Ok(report) => report,
            Err(_) => {
                self.error_code = 4;
                return self.error_code;
            }
        };
        self.total_collisions = self
            .total_collisions
            .saturating_add(u32::try_from(report.events.len()).unwrap_or(u32::MAX));
        self.last_stats = report.stats;
        self.cleanup_projectiles();
        0
    }

    fn shoot(&mut self, velocity_x: i32, velocity_y: i32, velocity_z: i32) -> i32 {
        let velocity = Vec3i::new(
            velocity_x.clamp(-PROJECTILE_SPEED_LIMIT, PROJECTILE_SPEED_LIMIT),
            velocity_y.clamp(-PROJECTILE_SPEED_LIMIT, PROJECTILE_SPEED_LIMIT),
            velocity_z.clamp(-PROJECTILE_SPEED_LIMIT, PROJECTILE_SPEED_LIMIT),
        );
        if velocity == Vec3i::ZERO {
            return -1;
        }
        let Some(player) = self.world.body(PLAYER_ID) else {
            return -1;
        };
        let player_position = player.position();
        let spawn = Vec3i::new(
            player_position.x + velocity.x / 3,
            player_position.y + 12 + velocity.y / 3,
            player_position.z + velocity.z / 3,
        );
        let id = BodyId(self.next_projectile_id);
        self.next_projectile_id = self.next_projectile_id.saturating_add(1);

        if self
            .world
            .add_body(
                RigidBody::dynamic(id, spawn, velocity, Vec3i::new(3, 3, 3))
                    .with_material(Material::new(350)),
            )
            .is_err()
        {
            self.error_code = 5;
            return -1;
        }
        self.projectile_ids.push(id);
        if self.projectile_ids.len() > MAX_PROJECTILES {
            let oldest = self.projectile_ids.remove(0);
            self.world.remove_body(oldest);
        }
        i32::try_from(id.0).unwrap_or(i32::MAX)
    }

    fn cleanup_projectiles(&mut self) {
        let stale = self
            .projectile_ids
            .iter()
            .copied()
            .filter(|id| {
                self.world.body(*id).is_none_or(|body| {
                    let position = body.position();
                    position.x.abs() > 1_200
                        || position.y < -300
                        || position.y > 900
                        || position.z.abs() > 1_200
                })
            })
            .collect::<Vec<_>>();
        for id in stale {
            self.world.remove_body(id);
            self.projectile_ids.retain(|candidate| *candidate != id);
        }
    }

    fn body_at(&self, index: u32) -> Option<&RigidBody> {
        self.world.bodies().nth(index as usize)
    }
}

fn add_fixed(
    world: &mut World,
    id: u64,
    position: Vec3i,
    half_extents: Vec3i,
) -> Result<(), physics_engine::PhysicsError> {
    world.add_body(RigidBody::fixed(BodyId(id), position, half_extents))
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
    with_sandbox(|sandbox| u32::try_from(sandbox.world.bodies().count()).unwrap_or(u32::MAX))
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
pub extern "C" fn sandbox_last_collision_events() -> u32 {
    with_sandbox(|sandbox| u32::try_from(sandbox.last_stats.collision_events).unwrap_or(u32::MAX))
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_last_pair_checks() -> u32 {
    with_sandbox(|sandbox| u32::try_from(sandbox.last_stats.pair_checks).unwrap_or(u32::MAX))
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
    use super::{PLAYER_ID, Sandbox};

    fn settle_player(sandbox: &mut Sandbox) {
        for _ in 0..16 {
            assert_eq!(sandbox.step(0, 0, false), 0);
        }
    }

    #[test]
    fn sandbox_contains_authoritative_player_and_test_world() {
        let sandbox = Sandbox::new().expect("valid sandbox");
        assert!(sandbox.world.body(PLAYER_ID).is_some());
        assert!(sandbox.world.bodies().count() >= 10);
    }

    #[test]
    fn grounded_player_can_jump() {
        let mut sandbox = Sandbox::new().expect("valid sandbox");
        settle_player(&mut sandbox);
        assert!(sandbox.grounded().expect("valid foot probe"));
        let before = sandbox.world.body(PLAYER_ID).expect("player").position().y;

        assert_eq!(sandbox.step(0, 0, true), 0);

        let player = sandbox.world.body(PLAYER_ID).expect("player after jump");
        assert!(player.position().y > before, "jump did not move upward");
        assert!(
            player.velocity().y > 0,
            "jump did not preserve upward velocity"
        );
        assert!(!sandbox.grounded().expect("valid airborne foot probe"));
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
            .body(physics_engine::BodyId(projectile as u64))
            .expect("projectile remains in the bounded test world");
        assert!(
            projectile.position().z > -190,
            "projectile tunneled through the thin CCD target: {:?}",
            projectile.position()
        );
    }
}
