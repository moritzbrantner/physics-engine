use super::{ProjectileType, Sandbox};
use physics_engine::{BodyId, Vec3i};

fn settled_tower() -> Sandbox {
    let mut sandbox = Sandbox::with_tower_options(false, false).unwrap();
    let encoded = (1 << 29) | ((1 << 11) - 2) | (1 << 14) | (2 << 12);
    let rules = super::controller::scenario_rules::ScenarioRules::decode(encoded, false).unwrap();
    super::controller::scenario_rules::apply_to_sandbox(&mut sandbox, rules).unwrap();
    for tick in 0..240 {
        assert_eq!(sandbox.step_velocity(0, 0, false), 0, "settle {tick}");
    }
    assert!(sandbox.is_quiescent());
    sandbox
}

#[test]
fn tower_near_misses_keep_all_crates_parked_through_projectile_removal() {
    for projectile in [
        ProjectileType::Sphere,
        ProjectileType::Arrow,
        ProjectileType::Rigid,
    ] {
        for x in [-45, 45] {
            let mut sandbox = settled_tower();
            let initial = (100..132)
                .map(|id| sandbox.world.box_by_id(BodyId(id)).unwrap().clone())
                .collect::<Vec<_>>();
            assert_eq!(sandbox.set_projectile_type(projectile as i32), 0);
            assert!(sandbox.shoot(x, 0, -85) >= 0);
            for tick in 0..120 {
                assert_eq!(
                    sandbox.step_velocity(0, 0, false),
                    0,
                    "{projectile:?} direction {x}, tick {tick}: {}",
                    sandbox.error_detail
                );
                for before in &initial {
                    let id = before.body().id();
                    assert!(
                        sandbox.world.is_sleeping(id),
                        "{projectile:?} direction {x} woke {id:?} at {tick}"
                    );
                    assert_eq!(
                        sandbox.world.box_by_id(id),
                        Some(before),
                        "miss changed {id:?}"
                    );
                }
            }
            assert_eq!(sandbox.world.ballistic_sphere_count(), 0);
            assert_eq!(sandbox.active_projectile_count(), 0);
            assert!(
                sandbox.projectiles_retired_on_contact + sandbox.projectiles_retired_out_of_bounds
                    > 0
            );
        }
    }
}

#[test]
fn a_proven_miss_does_not_enter_tower_stabilization() {
    let mut sandbox = settled_tower();
    sandbox.set_projectile_type(ProjectileType::Sphere as i32);
    assert!(sandbox.shoot(45, 0, -85) >= 0);
    for tick in 0..5 {
        assert_eq!(sandbox.step_velocity(0, 0, false), 0, "miss tick {tick}");
        assert_eq!(sandbox.last_step_stats.response_authority_body_count, 0);
        assert_eq!(sandbox.last_step_stats.sampled_events, 0);
        assert_eq!(sandbox.last_step_stats.event_response_passes, 0);
        assert_eq!(sandbox.last_step_stats.stabilization_passes, 0);
    }
}

#[test]
fn predicted_real_impacts_still_wake_and_transfer_momentum() {
    for projectile in [
        ProjectileType::Sphere,
        ProjectileType::Arrow,
        ProjectileType::Rigid,
    ] {
        let mut sandbox = settled_tower();
        // Keep a single crate for response evidence; the separate tower convergence test is not
        // replaced by this focused wake-boundary regression.
        for id in 101..132 {
            sandbox.world.remove_box(BodyId(id)).unwrap();
        }
        for _ in 0..240 {
            assert_eq!(sandbox.step_velocity(0, 0, false), 0);
        }
        let id = BodyId(100);
        let before = sandbox.world.box_by_id(id).unwrap().clone();
        assert!(sandbox.world.is_sleeping(id));
        sandbox.set_projectile_type(projectile as i32);
        assert!(sandbox.shoot(-21, 0, -94) >= 0);
        let mut affected = false;
        for tick in 0..10 {
            assert_eq!(
                sandbox.step_velocity(0, 0, false),
                0,
                "{projectile:?} hit tick {tick}: {}",
                sandbox.error_detail
            );
            let current = sandbox.world.box_by_id(id).unwrap();
            affected |= current.body().position() != before.body().position()
                || current.body().velocity() != Vec3i::ZERO
                || current.angular() != before.angular();
        }
        assert!(affected, "{projectile:?} did not affect the target");
    }
}
