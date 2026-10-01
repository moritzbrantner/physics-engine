// Numeric consumer geometry only; no game, interest or rendering ownership.
use physics_engine::{BodyId, RigidBody, Vec3i, World, WorldConfig};

pub const FIXED_COUNT: usize = 261;
pub const DYNAMIC_BASE: u64 = 1_000_000;

pub struct Input {
    pub fixed: Vec<RigidBody>,
    pub spawns: Vec<Vec3i>,
    pub half: Vec3i,
    pub gravity: Vec3i,
}

fn vector(value: &serde_json::Value) -> Vec3i {
    let [x, y, z] = serde_json::from_value::<[i32; 3]>(value.clone()).unwrap();
    Vec3i::new(x, y, z)
}

pub fn input() -> Input {
    let data: serde_json::Value =
        serde_json::from_str(include_str!("../fixtures/translational-fixed-heavy.json")).unwrap();
    assert_eq!(
        data["source_revision"],
        "20d34fbb2d9d7014e4bfb089ed1fb85fd1e9eddc"
    );
    let fixed = data["fixed"]
        .as_array()
        .unwrap()
        .iter()
        .map(|row| {
            RigidBody::fixed(
                BodyId(row[0].as_u64().unwrap()),
                vector(&row[1]),
                vector(&row[2]),
            )
        })
        .collect::<Vec<_>>();
    assert_eq!(fixed.len(), FIXED_COUNT);
    Input {
        fixed,
        spawns: data["spawns"]
            .as_array()
            .unwrap()
            .iter()
            .map(vector)
            .collect(),
        half: vector(&data["dynamic_half_extents"]),
        gravity: vector(&data["gravity"]),
    }
}

impl Input {
    pub fn world(&self, count: usize, gravity: Vec3i) -> World {
        assert!(count <= self.spawns.len());
        let mut world = World::new(WorldConfig {
            gravity,
            ..WorldConfig::default()
        });
        for fixed in &self.fixed {
            world.add_body(fixed.clone()).unwrap();
        }
        for (i, &position) in self.spawns[..count].iter().enumerate() {
            world
                .add_body(RigidBody::dynamic(
                    BodyId(DYNAMIC_BASE + i as u64),
                    position,
                    Vec3i::ZERO,
                    self.half,
                ))
                .unwrap();
        }
        world
    }
}

pub fn run() -> [f64; 12] {
    let input = input();
    let count = 8;
    let mut world = input.world(count, input.gravity);
    let mut totals = [0_u64; 7];
    let mut all_work = [0_u64; 4];
    for tick in 0..24 {
        let command = Vec3i::new(if tick % 2 == 0 { 1 } else { -1 }, 0, 0);
        world.set_velocity(BodyId(DYNAMIC_BASE), command).unwrap();
        // A rebuilding current-state control is supplementary to the existing
        // private exhaustive oracle; it is not a second production world authority.
        let mut rebuilt = World::new(WorldConfig {
            gravity: input.gravity,
            ..WorldConfig::default()
        });
        for body in world.bodies() {
            rebuilt.add_body(body.clone()).unwrap();
        }
        let report = world.step(1).unwrap();
        let reference = rebuilt.step(1).unwrap();
        assert_eq!(report.events, reference.events);
        assert_eq!(
            world.bodies().collect::<Vec<_>>(),
            rebuilt.bodies().collect::<Vec<_>>()
        );
        for work in [report.stats.work, reference.stats.work] {
            for (sum, value) in all_work.iter_mut().zip([
                work.fixed_sweep_bound_preparations,
                work.dynamic_sweep_bound_preparations,
                work.broad_phase_queries,
                work.staged_bodies,
            ]) {
                *sum += value as u64;
            }
        }
        let work = report.stats.work;
        assert_eq!(
            work.fixed_sweep_bound_preparations,
            FIXED_COUNT * work.broad_phase_queries
        );
        assert_eq!(
            work.dynamic_sweep_bound_preparations,
            count * work.broad_phase_queries
        );
        assert_eq!(
            work.fixed_sweep_bound_preparations + work.dynamic_sweep_bound_preparations,
            work.sweep_bound_preparations
        );
        assert!(work.broad_phase_queries > 0);
        for (sum, value) in totals.iter_mut().zip([
            work.fixed_sweep_bound_preparations,
            work.dynamic_sweep_bound_preparations,
            work.broad_phase_queries,
            work.staged_bodies,
            report.stats.contact_resolutions,
            report.stats.collision_events,
            1,
        ]) {
            *sum += value as u64;
        }
    }
    // Genuine no-query reuse stays distinct from preparing fixed geometry again.
    let mut quiet = input.world(count, Vec3i::ZERO);
    let bootstrap = quiet.step(1).unwrap().stats.work;
    let no_op = quiet.step(1).unwrap().stats.work;
    for work in [bootstrap, no_op] {
        for (sum, value) in all_work.iter_mut().zip([
            work.fixed_sweep_bound_preparations,
            work.dynamic_sweep_bound_preparations,
            work.broad_phase_queries,
            work.staged_bodies,
        ]) {
            *sum += value as u64;
        }
    }
    assert!(no_op.cached_stationary_step);
    assert_eq!(no_op.sweep_bound_preparations, 0);
    assert_eq!(no_op.fixed_sweep_bound_preparations, 0);
    assert_eq!(no_op.dynamic_sweep_bound_preparations, 0);
    let mut values = [0.0; 12];
    for (value, sum) in values[..7].iter_mut().zip(totals) {
        *value = sum as f64;
    }
    values[7] = world.retained_step_scratch_bytes() as f64;
    for (value, sum) in values[8..].iter_mut().zip(all_work) {
        *value = sum as f64;
    }
    values
}
