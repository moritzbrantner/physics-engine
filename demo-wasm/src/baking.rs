use physics_engine::{FIXED_GEOMETRY_PREPARATION_VERSION, FixedGeometryPreparationMode3d};

use crate::{
    Sandbox,
    controller::scenario_rules::{ScenarioRules, apply_to_sandbox},
    with_sandbox, with_sandbox_mut,
};

fn preparation_mode(value: i32) -> Option<FixedGeometryPreparationMode3d> {
    match value {
        0 => Some(FixedGeometryPreparationMode3d::Runtime),
        1 => Some(FixedGeometryPreparationMode3d::PrepareAtLoad),
        _ => None,
    }
}

/// Independent sandbox comparison axes. The first argument accepts legacy 0 physical / 1 linear
/// character values or the explicit scenario-rules bitfield. The second argument remains a legacy crate
/// compatibility input; explicit rules carry their own crate-motion policy. Fixed geometry preparation
/// remains a separate performance/storage choice.
#[unsafe(no_mangle)]
pub extern "C" fn sandbox_reset_with_baking_options(
    simulation_rules: i32,
    upright_crates: i32,
    fixed_geometry_mode: i32,
) -> i32 {
    if !(0..=1).contains(&upright_crates) {
        return -1;
    }
    let Some(rules) = ScenarioRules::decode(simulation_rules, upright_crates == 1) else {
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
        Sandbox::with_options(rules.character_linear_push(), rules.upright_crates())
    else {
        return -2;
    };
    if apply_to_sandbox(&mut replacement, rules).is_err() {
        return -2;
    }
    replacement
        .world
        .set_fixed_geometry_preparation_mode(fixed_geometry_mode);
    with_sandbox_mut(|sandbox| *sandbox = replacement);
    0
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
    use physics_engine::FixedGeometryPreparationMode3d;

    use crate::{
        Sandbox,
        controller::scenario_rules::{EXPLICIT_RULES_BIT, sandbox_simulation_rules},
    };

    use super::sandbox_reset_with_baking_options;

    #[test]
    fn first_explicit_reset_preserves_projectile_policy_bits() {
        let all_pair_bits = (1_i32 << 11) - 2;
        let impact_retire = (1_i32 << 14) | (2_i32 << 12);
        let encoded = EXPLICIT_RULES_BIT | all_pair_bits | impact_retire;

        assert_eq!(sandbox_reset_with_baking_options(encoded, 0, 0), 0);
        assert_eq!(sandbox_simulation_rules(), encoded);
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
