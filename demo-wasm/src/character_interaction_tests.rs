use super::{
    PLAYER_ID, Sandbox, rotating_box, sandbox_reset, sandbox_reset_with_character_mode,
    with_sandbox,
};
use physics_engine::{BodyId, ContactMode3d, RigidBody, Vec3i};

fn settle(sandbox: &mut Sandbox) {
    for _ in 0..240 {
        assert_eq!(sandbox.step_velocity(0, 0, false), 0);
    }
    assert!(sandbox.is_quiescent());
}

#[test]
fn explicit_mode_is_opt_in_and_invalid_reset_is_atomic() {
    sandbox_reset();
    assert_eq!(
        with_sandbox(|s| s.world.box_by_id(PLAYER_ID).unwrap().contact_mode()),
        ContactMode3d::Physical
    );
    assert_eq!(sandbox_reset_with_character_mode(1), 0);
    let before = with_sandbox(|s| s.world.boxes().cloned().collect::<Vec<_>>());
    assert_eq!(sandbox_reset_with_character_mode(42), -1);
    assert_eq!(
        with_sandbox(|s| s.world.boxes().cloned().collect::<Vec<_>>()),
        before
    );
}

#[test]
fn upright_comparison_pushes_a_rough_offset_crate_without_rotating_it() {
    let mut sandbox = Sandbox::with_options(true, true).unwrap();
    settle(&mut sandbox);
    let id = BodyId(901);
    sandbox
        .world
        .add_box(
            rotating_box(
                RigidBody::dynamic(
                    id,
                    Vec3i::new(8, 18, 285),
                    Vec3i::ZERO,
                    Vec3i::new(18, 18, 18),
                )
                .with_mass(2)
                .with_material(physics_engine::Material::new(0).with_friction(1000)),
            )
            .with_rotation_locked(),
        )
        .unwrap();
    let before = sandbox.world.box_by_id(id).unwrap().clone();
    for tick in 0..12 {
        assert_eq!(sandbox.step_velocity(0, -420, false), 0, "tick {tick}");
        let current = sandbox.world.box_by_id(id).unwrap();
        assert_eq!(current.angular(), before.angular(), "tick {tick}");
    }
    assert!(sandbox.world.box_by_id(id).unwrap().body().position().z < before.body().position().z);
}

#[test]
#[ignore = "real sandbox landing replay; explicitly required by Validate"]
fn landing_at_a_crate_stack_edge_does_not_disturb_the_crates() {
    let mut sandbox = Sandbox::with_character_mode(true).unwrap();
    settle(&mut sandbox);
    let before = [BodyId(100), BodyId(101)].map(|id| sandbox.world.box_by_id(id).unwrap().clone());
    let mut landed = false;
    for tick in 0..360 {
        let x = if tick < 13 { -420 } else { 0 };
        let z = if (16..46).contains(&tick) { -420 } else { 0 };
        assert_eq!(
            sandbox.step_velocity(x, z, tick == 27),
            0,
            "tick {tick}, detail {}",
            sandbox.error_detail
        );
        for original in &before {
            let current = sandbox.world.box_by_id(original.body().id()).unwrap();
            assert_eq!(
                current.body().position(),
                original.body().position(),
                "crate displacement at tick {tick}"
            );
            assert_eq!(
                current.angular(),
                original.angular(),
                "crate angular state at tick {tick}"
            );
        }
        if tick > 50 && sandbox.grounded().unwrap() {
            landed = true;
        }
    }
    assert!(landed);
    assert!(
        sandbox
            .world
            .box_by_id(PLAYER_ID)
            .unwrap()
            .body()
            .position()
            .y
            >= 90,
        "player must remain on top, not tunnel through the stack"
    );
    assert!(sandbox.is_quiescent(), "resting scene must return to sleep");
}

#[test]
fn projectiles_still_rotate_crates_in_linear_character_mode() {
    let mut sandbox = Sandbox::with_character_mode(true).unwrap();
    settle(&mut sandbox);
    let before =
        [BodyId(100), BodyId(101)].map(|id| sandbox.world.box_by_id(id).unwrap().angular());
    assert!(sandbox.shoot(-38, 0, -88) >= 0);
    let mut rotated = false;
    for _ in 0..24 {
        assert_eq!(sandbox.step_velocity(0, 0, false), 0);
        for (index, id) in [BodyId(100), BodyId(101)].into_iter().enumerate() {
            rotated |= sandbox.world.box_by_id(id).unwrap().angular() != before[index];
        }
    }
    assert!(
        rotated,
        "projectile dynamics must not be globally rotation-locked"
    );
}

#[test]
fn physically_rotated_crates_return_to_sleep_after_the_impact() {
    let mut sandbox = Sandbox::with_character_mode(true).unwrap();
    settle(&mut sandbox);
    let crate_ids = [BodyId(100), BodyId(101)];
    let before = crate_ids.map(|id| sandbox.world.box_by_id(id).unwrap().clone());
    assert!(sandbox.shoot(-38, 0, -88) >= 0);

    let mut rotated = false;
    for tick in 0..24 {
        assert_eq!(sandbox.step_velocity(0, 0, false), 0, "impact tick {tick}");
        for (index, id) in crate_ids.into_iter().enumerate() {
            rotated |= sandbox.world.box_by_id(id).unwrap().angular() != before[index].angular();
        }
    }
    assert!(
        rotated,
        "acceptance setup must impart angular motion to a crate"
    );

    let mut settled = false;
    for tick in 0..600 {
        let before_step = crate_ids.map(|id| {
            let rigid_box = sandbox.world.box_by_id(id).unwrap();
            (
                id,
                rigid_box.body().position(),
                rigid_box.body().velocity(),
                rigid_box.angular().angular_velocity,
            )
        });
        let status = sandbox.step_velocity(0, 0, false);
        assert_eq!(
            status, 0,
            "settling tick {tick}, detail {}, crates before step {before_step:?}",
            sandbox.error_detail
        );
        if crate_ids
            .into_iter()
            .all(|id| sandbox.world.is_sleeping(id))
        {
            settled = true;
            break;
        }
    }
    assert!(
        settled,
        "rough zero-restitution crates must not keep rocking/bouncing on the ground indefinitely"
    );

    let sleeping = crate_ids.map(|id| sandbox.world.box_by_id(id).unwrap().clone());
    for tick in 0..60 {
        assert_eq!(sandbox.step_velocity(0, 0, false), 0, "sleep tick {tick}");
    }
    for (index, id) in crate_ids.into_iter().enumerate() {
        assert_eq!(
            sandbox.world.box_by_id(id),
            Some(&sleeping[index]),
            "settled crate {id:?} must keep an exact resting pose"
        );
    }
}

#[test]
fn impact_retire_preserves_contact_response_before_projectile_removal() {
    let mut sandbox = Sandbox::with_character_mode(true).unwrap();
    // Explicit scenario marker + all pair bits + linear character response + explicit impact-retire policy.
    let all_pair_bits = (1_i32 << 11) - 2;
    let encoded_rules = (1_i32 << 29) | all_pair_bits | 1 | (1_i32 << 14) | (2_i32 << 12);
    let rules = super::controller::scenario_rules::ScenarioRules::decode(encoded_rules, false)
        .expect("impact-retire scenario rules");
    super::controller::scenario_rules::apply_to_sandbox(&mut sandbox, rules)
        .expect("apply impact-retire rules");
    settle(&mut sandbox);

    let crate_ids = [BodyId(100), BodyId(101)];
    let before = crate_ids.map(|id| sandbox.world.box_by_id(id).unwrap().clone());
    let projectile = sandbox.shoot(-38, 0, -88);
    assert!(projectile >= 0);
    let projectile_id = BodyId(projectile as u64);
    let mut crate_responded = false;

    for tick in 0..24 {
        assert_eq!(sandbox.step_velocity(0, 0, false), 0, "tick {tick}");
        for (index, id) in crate_ids.into_iter().enumerate() {
            let current = sandbox.world.box_by_id(id).unwrap();
            crate_responded |= current.body().position() != before[index].body().position()
                || current.angular() != before[index].angular();
        }
        if sandbox.world.box_by_id(projectile_id).is_none() {
            break;
        }
    }

    assert!(
        sandbox.world.box_by_id(projectile_id).is_none(),
        "impact-retire projectile must leave the authoritative world after contact"
    );
    assert_eq!(sandbox.projectiles_retired_on_contact, 1);
    assert!(
        crate_responded,
        "impact retirement must happen after the engine applies the collision response"
    );
}

#[test]
fn upright_and_character_response_options_are_independent() {
    for linear in [false, true] {
        for upright in [false, true] {
            let sandbox = Sandbox::with_options(linear, upright).unwrap();
            assert_eq!(
                sandbox
                    .world
                    .box_by_id(BodyId(100))
                    .unwrap()
                    .rotation_locked(),
                upright
            );
            assert_eq!(
                sandbox.world.box_by_id(PLAYER_ID).unwrap().contact_mode()
                    != ContactMode3d::Physical,
                linear
            );
            assert_eq!(sandbox.world.entity_count(), 18);
        }
    }
    assert_eq!(super::sandbox_reset_with_options(1, 1), 0);
    let before = with_sandbox(|s| s.world.boxes().cloned().collect::<Vec<_>>());
    assert_eq!(super::sandbox_reset_with_options(1, 2), -1);
    assert_eq!(super::sandbox_reset_with_options(-1, 0), -1);
    assert_eq!(
        with_sandbox(|s| s.world.boxes().cloned().collect::<Vec<_>>()),
        before
    );
}

#[test]
fn upright_crates_still_translate_when_hit_by_projectiles() {
    let mut sandbox = Sandbox::with_options(true, true).unwrap();
    settle(&mut sandbox);
    let before = [BodyId(100), BodyId(101)].map(|id| sandbox.world.box_by_id(id).unwrap().clone());
    assert!(sandbox.shoot(-38, 0, -88) >= 0);
    let mut moved = false;
    for _ in 0..24 {
        assert_eq!(sandbox.step_velocity(0, 0, false), 0);
        for original in &before {
            let current = sandbox.world.box_by_id(original.body().id()).unwrap();
            assert_eq!(current.angular(), original.angular());
            moved |= current.body().position() != original.body().position();
        }
    }
    assert!(
        moved,
        "upright does not mean fixed or immune to projectiles"
    );
}
