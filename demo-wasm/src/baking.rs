use physics_engine::{
    FIXED_GEOMETRY_PREPARATION_VERSION, FixedGeometryPreparationMode3d, InteractionCategory3d,
    InteractionPolicy3d, RotatingWorldError3d,
};

use crate::{
    Sandbox,
    controller::scenario_rules::{EXPLICIT_RULES_BIT, ScenarioRules, apply_to_sandbox},
    role_for, with_sandbox, with_sandbox_mut,
};

const INTERACTION_POLICY_PACK_BIT: i32 = 1 << 30;
const INTERACTION_POLICY_BITS_PER_PAIR: u32 = 5;
const INTERACTION_POLICY_CODE_MASK: i32 = (1 << INTERACTION_POLICY_BITS_PER_PAIR) - 1;
const INTERACTION_POLICY_PAYLOAD_MASK: i32 = (1 << (INTERACTION_POLICY_BITS_PER_PAIR * 3)) - 1;
const INTERACTION_POLICY_KNOWN_BITS: i32 =
    INTERACTION_POLICY_PACK_BIT | INTERACTION_POLICY_PAYLOAD_MASK;

const WORLD_CATEGORY: InteractionCategory3d = InteractionCategory3d::new(1);
const CHARACTER_CATEGORY: InteractionCategory3d = InteractionCategory3d::new(2);
const CRATE_CATEGORY: InteractionCategory3d = InteractionCategory3d::new(3);

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
struct PairPolicySettings {
    world_character: Option<u8>,
    world_crate: Option<u8>,
    world_projectile: Option<u8>,
}

fn preparation_mode(value: i32) -> Option<FixedGeometryPreparationMode3d> {
    match value {
        0 => Some(FixedGeometryPreparationMode3d::Runtime),
        1 => Some(FixedGeometryPreparationMode3d::PrepareAtLoad),
        _ => None,
    }
}

fn decode_pass_limit(code: i32) -> Option<Option<u8>> {
    match code {
        0 => Some(None),
        1..=17 => Some(Some((code - 1) as u8)),
        _ => None,
    }
}

fn decode_pair_policy_settings(value: i32) -> Option<PairPolicySettings> {
    if value < 0
        || value & INTERACTION_POLICY_PACK_BIT == 0
        || value & !INTERACTION_POLICY_KNOWN_BITS != 0
    {
        return None;
    }

    let payload = value & INTERACTION_POLICY_PAYLOAD_MASK;
    let code = |index: u32| {
        (payload >> (index * INTERACTION_POLICY_BITS_PER_PAIR)) & INTERACTION_POLICY_CODE_MASK
    };

    Some(PairPolicySettings {
        world_character: decode_pass_limit(code(0))?,
        world_crate: decode_pass_limit(code(1))?,
        world_projectile: decode_pass_limit(code(2))?,
    })
}

fn interaction_category_for_role(role: i32) -> InteractionCategory3d {
    match role {
        0 => WORLD_CATEGORY,
        1 => CHARACTER_CATEGORY,
        2 => CRATE_CATEGORY,
        _ => InteractionCategory3d::DEFAULT,
    }
}

fn set_pair_policy(
    sandbox: &mut Sandbox,
    left: InteractionCategory3d,
    right: InteractionCategory3d,
    pass_limit: Option<u8>,
) {
    if let Some(pass_limit) = pass_limit {
        sandbox.world.set_pair_interaction_policy(
            left,
            right,
            InteractionPolicy3d::default().with_fixed_boundary_stabilization_pass_limit(pass_limit),
        );
    }
}

fn apply_interaction_policy_settings(
    sandbox: &mut Sandbox,
    settings: PairPolicySettings,
) -> Result<(), RotatingWorldError3d> {
    // The demo's role taxonomy stays outside rigid-body material/layer state. Projectiles deliberately
    // remain in the default category so projectiles spawned after reset inherit the configured world pair.
    let assignments = sandbox
        .world
        .boxes()
        .map(|rigid_box| {
            let id = rigid_box.body().id();
            (id, interaction_category_for_role(role_for(id)))
        })
        .collect::<Vec<_>>();

    for (id, category) in assignments {
        if category != InteractionCategory3d::DEFAULT {
            sandbox.world.set_body_interaction_category(id, category)?;
        }
    }

    set_pair_policy(
        sandbox,
        WORLD_CATEGORY,
        CHARACTER_CATEGORY,
        settings.world_character,
    );
    set_pair_policy(
        sandbox,
        WORLD_CATEGORY,
        CRATE_CATEGORY,
        settings.world_crate,
    );
    set_pair_policy(
        sandbox,
        WORLD_CATEGORY,
        InteractionCategory3d::DEFAULT,
        settings.world_projectile,
    );
    Ok(())
}

/// Independent sandbox comparison axes.
///
/// The first argument accepts legacy 0 physical / 1 linear character values or the explicit scenario-rules
/// bitfield. For legacy calls the second argument remains the 0/1 upright-crate compatibility input. For
/// explicit rules, old 0/1 callers remain accepted and ignored because crate motion is already encoded in the
/// first argument; a marked second argument carries the settings-backed pair-policy payload. Fixed geometry
/// preparation remains a separate performance/storage choice.
fn reset_with_baking_options(
    simulation_rules: i32,
    upright_crates_or_pair_policies: i32,
    fixed_geometry_mode: i32,
    build_layout: fn(bool, bool) -> Result<Sandbox, RotatingWorldError3d>,
) -> i32 {
    let explicit_rules = simulation_rules & EXPLICIT_RULES_BIT != 0;
    let (legacy_upright_crates, pair_policy_settings) = if explicit_rules
        && upright_crates_or_pair_policies & INTERACTION_POLICY_PACK_BIT != 0
    {
        let Some(settings) = decode_pair_policy_settings(upright_crates_or_pair_policies) else {
            return -1;
        };
        (false, settings)
    } else {
        if !(0..=1).contains(&upright_crates_or_pair_policies) {
            return -1;
        }
        (
            upright_crates_or_pair_policies == 1,
            PairPolicySettings::default(),
        )
    };

    let Some(rules) = ScenarioRules::decode(simulation_rules, legacy_upright_crates) else {
        return -1;
    };
    let Some(fixed_geometry_mode) = preparation_mode(fixed_geometry_mode) else {
        return -1;
    };

    // SANDBOX is lazy and its default constructor resets the scenario-rules thread-local. Initialize it
    // before staging a valid replacement so a first explicit reset cannot erase the rules after they are
    // applied to the replacement. Invalid inputs still return above without touching the current sandbox.
    with_sandbox(|_| ());

    let Ok(mut replacement) =
        build_layout(rules.character_linear_push(), rules.upright_crates())
    else {
        return -2;
    };
    if apply_to_sandbox(&mut replacement, rules).is_err()
        || apply_interaction_policy_settings(&mut replacement, pair_policy_settings).is_err()
    {
        return -2;
    }
    replacement
        .world
        .set_fixed_geometry_preparation_mode(fixed_geometry_mode);
    with_sandbox_mut(|sandbox| *sandbox = replacement);
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_reset_with_baking_options(
    simulation_rules: i32,
    upright_crates_or_pair_policies: i32,
    fixed_geometry_mode: i32,
) -> i32 {
    reset_with_baking_options(
        simulation_rules,
        upright_crates_or_pair_policies,
        fixed_geometry_mode,
        Sandbox::with_options,
    )
}

/// Reset the shared Rust/WASM sandbox with the deterministic 32-crate tower layout.
/// Layout selection stays separate from simulation rules so the browser remains an advisory consumer.
#[unsafe(no_mangle)]
pub extern "C" fn sandbox_reset_tower_with_baking_options(
    simulation_rules: i32,
    upright_crates_or_pair_policies: i32,
    fixed_geometry_mode: i32,
) -> i32 {
    reset_with_baking_options(
        simulation_rules,
        upright_crates_or_pair_policies,
        fixed_geometry_mode,
        Sandbox::with_tower_options,
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_fixed_geometry_mode() -> i32 {
    with_sandbox(
        |sandbox| match sandbox.world.fixed_geometry_preparation_stats().mode {
            FixedGeometryPreparationMode3d::Runtime => 0,
            FixedGeometryPreparationMode3d::PrepareAtLoad => 1,
        },
    )
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_fixed_geometry_prepared_count() -> u32 {
    with_sandbox(|sandbox| {
        u32::try_from(
            sandbox
                .world
                .fixed_geometry_preparation_stats()
                .prepared_body_count,
        )
        .unwrap_or(u32::MAX)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_fixed_geometry_total_preparations() -> u32 {
    with_sandbox(|sandbox| {
        u32::try_from(
            sandbox
                .world
                .fixed_geometry_preparation_stats()
                .total_preparations,
        )
        .unwrap_or(u32::MAX)
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_fixed_geometry_retained_bytes() -> u32 {
    with_sandbox(|sandbox| {
        u32::try_from(
            sandbox
                .world
                .fixed_geometry_preparation_stats()
                .retained_bytes,
        )
        .unwrap_or(u32::MAX)
    })
}

#[unsafe(no_mangle)]
pub const extern "C" fn sandbox_fixed_geometry_representation_version() -> u32 {
    FIXED_GEOMETRY_PREPARATION_VERSION
}

#[cfg(test)]
mod tests {
    use physics_engine::{BodyId, FixedGeometryPreparationMode3d};

    use crate::{
        Sandbox,
        controller::scenario_rules::{EXPLICIT_RULES_BIT, sandbox_simulation_rules},
        with_sandbox,
    };

    use super::{
        CHARACTER_CATEGORY, CRATE_CATEGORY, INTERACTION_POLICY_PACK_BIT, PairPolicySettings,
        WORLD_CATEGORY, decode_pair_policy_settings, sandbox_reset_with_baking_options,
    };

    #[test]
    fn first_explicit_reset_preserves_projectile_policy_bits() {
        let all_pair_bits = (1_i32 << 11) - 2;
        let impact_retire = (1_i32 << 14) | (2_i32 << 12);
        let encoded = EXPLICIT_RULES_BIT | all_pair_bits | impact_retire;

        assert_eq!(sandbox_reset_with_baking_options(encoded, 0, 0), 0);
        assert_eq!(sandbox_simulation_rules(), encoded);
    }

    #[test]
    fn marked_pair_policy_payload_decodes_all_supported_fixed_world_pairs() {
        let encoded = INTERACTION_POLICY_PACK_BIT | 5 | (1 << 5) | (17 << 10);
        assert_eq!(
            decode_pair_policy_settings(encoded),
            Some(PairPolicySettings {
                world_character: Some(4),
                world_crate: Some(0),
                world_projectile: Some(16),
            })
        );
        assert_eq!(
            decode_pair_policy_settings(INTERACTION_POLICY_PACK_BIT),
            Some(PairPolicySettings::default())
        );
        assert_eq!(
            decode_pair_policy_settings(INTERACTION_POLICY_PACK_BIT | (18 << 10)),
            None
        );
    }

    #[test]
    fn old_explicit_rule_boolean_argument_keeps_legacy_compatibility() {
        let all_pair_bits = (1_i32 << 11) - 2;
        let encoded = EXPLICIT_RULES_BIT | all_pair_bits;

        assert_eq!(sandbox_reset_with_baking_options(encoded, 1, 0), 0);
    }

    #[test]
    fn settings_reset_assigns_demo_roles_to_engine_interaction_categories() {
        let all_pair_bits = (1_i32 << 11) - 2;
        let encoded = EXPLICIT_RULES_BIT | all_pair_bits;
        let pair_settings = INTERACTION_POLICY_PACK_BIT | (3 << 5);

        assert_eq!(
            sandbox_reset_with_baking_options(encoded, pair_settings, 0),
            0
        );
        with_sandbox(|sandbox| {
            assert_eq!(
                sandbox.world.body_interaction_category(BodyId(10)),
                WORLD_CATEGORY
            );
            assert_eq!(
                sandbox.world.body_interaction_category(BodyId(1)),
                CHARACTER_CATEGORY
            );
            assert_eq!(
                sandbox.world.body_interaction_category(BodyId(100)),
                CRATE_CATEGORY
            );
        });
    }

    #[test]
    fn prepare_at_load_keeps_sleeping_dynamics_out_of_baked_set() {
        let mut sandbox = Sandbox::with_options(false, false).expect("valid sandbox");
        sandbox
            .world
            .set_fixed_geometry_preparation_mode(FixedGeometryPreparationMode3d::PrepareAtLoad);
        let initial = sandbox.world.fixed_geometry_preparation_stats();
        assert_eq!(initial.prepared_body_count, 11);
        assert_eq!(initial.total_preparations, 11);

        for _ in 0..240 {
            assert_eq!(sandbox.step(0, 0, false), 0);
        }
        let settled = sandbox.world.fixed_geometry_preparation_stats();
        assert_eq!(settled.prepared_body_count, 11);
        assert_eq!(settled.total_preparations, 11);
    }

    #[test]
    fn runtime_reference_retains_no_prepared_geometry() {
        let sandbox = Sandbox::with_options(false, false).expect("valid sandbox");
        assert_eq!(
            sandbox
                .world
                .fixed_geometry_preparation_stats()
                .prepared_body_count,
            0
        );
    }
}
