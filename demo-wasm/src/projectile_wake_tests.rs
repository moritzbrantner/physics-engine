use super::{BodyId, ProjectileType, Sandbox};

#[test]
fn tower_near_misses_preserve_all_thirty_two_crates_through_projectile_retirement() {
    for upright in [false, true] {
        for projectile in [
            ProjectileType::Sphere,
            ProjectileType::Arrow,
            ProjectileType::Rigid,
        ] {
            let mut sandbox = Sandbox::with_tower_options(false, upright).unwrap();
            let encoded = (1 << 29)
                | ((1 << 11) - 2)
                | (1 << 14)
                | (2 << 12)
                | if upright { 1 << 11 } else { 0 };
            let rules =
                super::controller::scenario_rules::ScenarioRules::decode(encoded, upright).unwrap();
            super::controller::scenario_rules::apply_to_sandbox(&mut sandbox, rules).unwrap();
            for tick in 0..240 {
                assert_eq!(
                    sandbox.step_velocity(0, 0, false),
                    0,
                    "settling {tick}, detail {}",
                    sandbox.error_detail
                );
            }
            assert!(sandbox.is_quiescent());
            let before = (100..132)
                .map(|id| sandbox.world.box_by_id(BodyId(id)).unwrap().clone())
                .collect::<Vec<_>>();
            assert_eq!(sandbox.set_projectile_type(projectile as i32), 0);
            // Just to the right of the tower; eventually hits the outer wall and retires.
            assert!(sandbox.shoot(40, 0, -87) >= 0);
            for tick in 0..120 {
                assert_eq!(
                    sandbox.step_velocity(0, 0, false),
                    0,
                    "{projectile:?} upright={upright} tick={tick}, detail {}",
                    sandbox.error_detail
                );
                for body in &before {
                    let id = body.body().id();
                    assert!(
                        sandbox.world.is_sleeping(id),
                        "near-miss {projectile:?} woke {id:?} at tick {tick}"
                    );
                    assert_eq!(
                        sandbox.world.box_by_id(id),
                        Some(body),
                        "near-miss changed crate at tick {tick}"
                    );
                }
                assert_eq!(sandbox.last_step_stats.parked_bodies_woken, 0);
                assert_eq!(sandbox.last_step_stats.parked_wake_retries, 0);
                assert!(sandbox.last_step_stats.response_authority_body_count <= 2);
            }
            assert_eq!(
                sandbox.active_projectile_count(),
                0,
                "miss should complete its lifecycle"
            );
            assert_eq!(sandbox.projectiles_retired_on_contact, 1);
        }
    }
}
