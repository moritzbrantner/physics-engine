use std::cell::Cell;

use physics_engine::{CollisionLayers3d, RotatingWorldError3d};

use crate::{PLAYER_ID, PROJECTILE_ID_START, Sandbox};

pub(crate) const EXPLICIT_RULES_BIT: i32 = 1 << 29;
const CHARACTER_LINEAR_BIT: i32 = 1;
const WORLD_WORLD_BIT: i32 = 1 << 1;
const WORLD_CHARACTER_BIT: i32 = 1 << 2;
const WORLD_CRATE_BIT: i32 = 1 << 3;
const WORLD_PROJECTILE_BIT: i32 = 1 << 4;
const CHARACTER_CHARACTER_BIT: i32 = 1 << 5;
const CHARACTER_CRATE_BIT: i32 = 1 << 6;
const CHARACTER_PROJECTILE_BIT: i32 = 1 << 7;
const CRATE_CRATE_BIT: i32 = 1 << 8;
const CRATE_PROJECTILE_BIT: i32 = 1 << 9;
const PROJECTILE_PROJECTILE_BIT: i32 = 1 << 10;
const CRATE_UPRIGHT_BIT: i32 = 1 << 11;
const ALL_PAIR_BITS: i32 = WORLD_WORLD_BIT
    | WORLD_CHARACTER_BIT
    | WORLD_CRATE_BIT
    | WORLD_PROJECTILE_BIT
    | CHARACTER_CHARACTER_BIT
    | CHARACTER_CRATE_BIT
    | CHARACTER_PROJECTILE_BIT
    | CRATE_CRATE_BIT
    | CRATE_PROJECTILE_BIT
    | PROJECTILE_PROJECTILE_BIT;
const KNOWN_BITS: i32 =
    EXPLICIT_RULES_BIT | CHARACTER_LINEAR_BIT | CRATE_UPRIGHT_BIT | ALL_PAIR_BITS;

const WORLD_LAYER: u32 = 1 << 0;
const CHARACTER_LAYER: u32 = 1 << 1;
const CRATE_LAYER: u32 = 1 << 2;
const PROJECTILE_LAYER: u32 = 1 << 3;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ScenarioRole {
    World,
    Character,
    Crate,
    Projectile,
}

impl ScenarioRole {
    const ALL: [Self; 4] = [Self::World, Self::Character, Self::Crate, Self::Projectile];

    const fn layer(self) -> u32 {
        match self {
            Self::World => WORLD_LAYER,
            Self::Character => CHARACTER_LAYER,
            Self::Crate => CRATE_LAYER,
            Self::Projectile => PROJECTILE_LAYER,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct ScenarioRules {
    encoded: i32,
}

impl ScenarioRules {
    const fn all(character_linear: bool, upright_crates: bool) -> Self {
        let mut encoded = EXPLICIT_RULES_BIT | ALL_PAIR_BITS;
        if character_linear {
            encoded |= CHARACTER_LINEAR_BIT;
        }
        if upright_crates {
            encoded |= CRATE_UPRIGHT_BIT;
        }
        Self { encoded }
    }

    pub(crate) const fn decode(value: i32, legacy_upright_crates: bool) -> Option<Self> {
        if value == 0 || value == 1 {
            return Some(Self::all(value == 1, legacy_upright_crates));
        }
        if value < 0 || value & EXPLICIT_RULES_BIT == 0 || value & !KNOWN_BITS != 0 {
            return None;
        }
        Some(Self { encoded: value })
    }

    pub(crate) const fn character_linear_push(self) -> bool {
        self.encoded & CHARACTER_LINEAR_BIT != 0
    }

    pub(crate) const fn upright_crates(self) -> bool {
        self.encoded & CRATE_UPRIGHT_BIT != 0
    }

    pub(crate) const fn encoded(self) -> i32 {
        self.encoded
    }

    const fn pair_bit(left: ScenarioRole, right: ScenarioRole) -> i32 {
        match (left, right) {
            (ScenarioRole::World, ScenarioRole::World) => WORLD_WORLD_BIT,
            (ScenarioRole::World, ScenarioRole::Character)
            | (ScenarioRole::Character, ScenarioRole::World) => WORLD_CHARACTER_BIT,
            (ScenarioRole::World, ScenarioRole::Crate)
            | (ScenarioRole::Crate, ScenarioRole::World) => WORLD_CRATE_BIT,
            (ScenarioRole::World, ScenarioRole::Projectile)
            | (ScenarioRole::Projectile, ScenarioRole::World) => WORLD_PROJECTILE_BIT,
            (ScenarioRole::Character, ScenarioRole::Character) => CHARACTER_CHARACTER_BIT,
            (ScenarioRole::Character, ScenarioRole::Crate)
            | (ScenarioRole::Crate, ScenarioRole::Character) => CHARACTER_CRATE_BIT,
            (ScenarioRole::Character, ScenarioRole::Projectile)
            | (ScenarioRole::Projectile, ScenarioRole::Character) => CHARACTER_PROJECTILE_BIT,
            (ScenarioRole::Crate, ScenarioRole::Crate) => CRATE_CRATE_BIT,
            (ScenarioRole::Crate, ScenarioRole::Projectile)
            | (ScenarioRole::Projectile, ScenarioRole::Crate) => CRATE_PROJECTILE_BIT,
            (ScenarioRole::Projectile, ScenarioRole::Projectile) => PROJECTILE_PROJECTILE_BIT,
        }
    }

    const fn pair_enabled(self, left: ScenarioRole, right: ScenarioRole) -> bool {
        self.encoded & Self::pair_bit(left, right) != 0
    }

    fn collision_layers(self, role: ScenarioRole) -> CollisionLayers3d {
        let mut mask = 0_u32;
        for other in ScenarioRole::ALL {
            if self.pair_enabled(role, other) {
                mask |= other.layer();
            }
        }
        CollisionLayers3d::new(role.layer(), mask)
    }
}

std::thread_local! {
    static CURRENT_RULES: Cell<ScenarioRules> = Cell::new(ScenarioRules::all(false, false));
}

pub(crate) fn reset_default() {
    CURRENT_RULES.with(|rules| rules.set(ScenarioRules::all(false, false)));
}

pub(crate) fn projectile_layers() -> CollisionLayers3d {
    CURRENT_RULES.with(|rules| rules.get().collision_layers(ScenarioRole::Projectile))
}

pub(crate) fn apply_to_sandbox(
    sandbox: &mut Sandbox,
    rules: ScenarioRules,
) -> Result<(), RotatingWorldError3d> {
    let boxes = sandbox.world.boxes().cloned().collect::<Vec<_>>();
    for rigid_box in boxes {
        let id = rigid_box.body().id();
        let role = if id == PLAYER_ID {
            ScenarioRole::Character
        } else if id.0 >= PROJECTILE_ID_START {
            ScenarioRole::Projectile
        } else if id.0 >= 100 {
            ScenarioRole::Crate
        } else {
            ScenarioRole::World
        };
        let removed = sandbox.world.remove_box(id);
        debug_assert!(
            removed.is_some(),
            "cloned sandbox body must still exist during role reassignment"
        );
        sandbox
            .world
            .add_box(rigid_box.with_collision_layers(rules.collision_layers(role)))?;
    }
    CURRENT_RULES.with(|current| current.set(rules));
    Ok(())
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_simulation_rules() -> i32 {
    CURRENT_RULES.with(|rules| rules.get().encoded())
}

#[cfg(test)]
mod tests {
    use super::{
        CHARACTER_CRATE_BIT, CHARACTER_PROJECTILE_BIT, CRATE_UPRIGHT_BIT, EXPLICIT_RULES_BIT,
        ScenarioRole, ScenarioRules,
    };

    #[test]
    fn legacy_modes_keep_all_collision_pairs_enabled() {
        let rules = ScenarioRules::decode(1, false).expect("legacy linear mode");
        assert!(rules.character_linear_push());
        for left in ScenarioRole::ALL {
            for right in ScenarioRole::ALL {
                assert!(rules.pair_enabled(left, right));
            }
        }
    }

    #[test]
    fn explicit_rules_keep_response_and_collision_axes_independent() {
        let encoded = EXPLICIT_RULES_BIT | CHARACTER_CRATE_BIT | CRATE_UPRIGHT_BIT;
        let rules = ScenarioRules::decode(encoded, false).expect("explicit rules");
        assert!(!rules.character_linear_push());
        assert!(rules.upright_crates());
        assert!(rules.pair_enabled(ScenarioRole::Character, ScenarioRole::Crate));
        assert!(!rules.pair_enabled(ScenarioRole::Character, ScenarioRole::Projectile));
        assert_eq!(rules.encoded() & CHARACTER_PROJECTILE_BIT, 0);
    }
}
