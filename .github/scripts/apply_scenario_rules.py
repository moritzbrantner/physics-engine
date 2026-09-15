from pathlib import Path


def replace_once(path: str, old: str, new: str) -> None:
    file = Path(path)
    text = file.read_text()
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{path}: expected one occurrence, found {count}: {old[:100]!r}")
    file.write_text(text.replace(old, new, 1))


# Broad phase: masks affect pair eligibility but not BVH topology.
replace_once(
    "src/rotating_broad_phase.rs",
    "        let exact = &self.exact;\n        let mut pairs = Vec::new();\n        self.tree.for_each_candidate_pair(|left, right| {\n",
    "        let exact = &self.exact;\n        let collision_layers = boxes\n            .iter()\n            .map(|rigid_box| (rigid_box.body().id(), rigid_box.collision_layers()))\n            .collect::<BTreeMap<_, _>>();\n        let mut pairs = Vec::new();\n        self.tree.for_each_candidate_pair(|left, right| {\n",
)
replace_once(
    "src/rotating_broad_phase.rs",
    "            if bounds_overlap(left_body.bounds, right_body.bounds) {\n                pairs.push(RotationalSweepPair3d { left, right });\n            }\n",
    "            let left_layers = collision_layers\n                .get(&left)\n                .copied()\n                .expect(\"indexed broad phase keeps every left collision layer\");\n            let right_layers = collision_layers\n                .get(&right)\n                .copied()\n                .expect(\"indexed broad phase keeps every right collision layer\");\n            if bounds_overlap(left_body.bounds, right_body.bounds)\n                && left_layers.collides_with(right_layers)\n            {\n                pairs.push(RotationalSweepPair3d { left, right });\n            }\n",
)

# Current-contact cache: policy changes must invalidate a geometry-identical cached graph.
replace_once(
    "src/current_contact_query.rs",
    "    BodyId, BodyKind, OrientedBox3d, RigidBox3d, RigidBoxFreeFlightConfig3d,\n",
    "    BodyId, BodyKind, CollisionLayers3d, OrientedBox3d, RigidBox3d,\n    RigidBoxFreeFlightConfig3d,\n",
)
replace_once(
    "src/current_contact_query.rs",
    "fingerprint: Vec<(BodyId, BodyKind, OrientedBox3d)>,",
    "fingerprint: Vec<(BodyId, BodyKind, CollisionLayers3d, OrientedBox3d)>,",
)
replace_once(
    "src/current_contact_query.rs",
    ") -> Vec<(BodyId, BodyKind, OrientedBox3d)> {",
    ") -> Vec<(BodyId, BodyKind, CollisionLayers3d, OrientedBox3d)> {",
)
replace_once(
    "src/current_contact_query.rs",
    "                rigid_box.body().kind(),\n                rigid_box.oriented_box(),\n",
    "                rigid_box.body().kind(),\n                rigid_box.collision_layers(),\n                rigid_box.oriented_box(),\n",
)
replace_once(
    "src/current_contact_query.rs",
    "fingerprint.sort_by_key(|(id, _, _)| *id);",
    "fingerprint.sort_by_key(|(id, _, _, _)| *id);",
)
replace_once(
    "src/current_contact_query.rs",
    "fingerprint: &[(BodyId, BodyKind, OrientedBox3d)],",
    "fingerprint: &[(BodyId, BodyKind, CollisionLayers3d, OrientedBox3d)],",
)
replace_once(
    "src/current_contact_query.rs",
    "fn cache_graph(fingerprint: Vec<(BodyId, BodyKind, OrientedBox3d)>, graph: CurrentContactGraph3d) {",
    "fn cache_graph(\n    fingerprint: Vec<(BodyId, BodyKind, CollisionLayers3d, OrientedBox3d)>,\n    graph: CurrentContactGraph3d,\n) {",
)

# Persistent-tail contact handling has one direct all-pairs probe outside broad phase.
replace_once(
    "src/rotating_world.rs",
    "            if left.body().kind() == BodyKind::Fixed && right.body().kind() == BodyKind::Fixed {\n                continue;\n            }\n            let Some(contact) = obb_contact_seed(left.oriented_box(), right.oriented_box())? else {\n",
    "            if left.body().kind() == BodyKind::Fixed && right.body().kind() == BodyKind::Fixed {\n                continue;\n            }\n            if !left\n                .collision_layers()\n                .collides_with(right.collision_layers())\n            {\n                continue;\n            }\n            let Some(contact) = obb_contact_seed(left.oriented_box(), right.oriented_box())? else {\n",
)

# Demo adapter: scenario vocabulary stays outside the reusable physics core.
replace_once(
    "demo-wasm/src/controller.rs",
    '#[path = "baking.rs"]\nmod baking;\n',
    '#[path = "baking.rs"]\nmod baking;\n#[path = "scenario_rules.rs"]\npub(crate) mod scenario_rules;\n',
)
replace_once(
    "demo-wasm/src/lib.rs",
    "    fn with_options(linear_push: bool, upright_crates: bool) -> Result<Self, RotatingWorldError3d> {\n        let mut world = RotatingWorld3d::new(RotatingWorldConfig3d {\n",
    "    fn with_options(linear_push: bool, upright_crates: bool) -> Result<Self, RotatingWorldError3d> {\n        controller::scenario_rules::reset_default();\n        let mut world = RotatingWorld3d::new(RotatingWorldConfig3d {\n",
)
replace_once(
    "demo-wasm/src/lib.rs",
    "            .add_box(rotating_box(\n                RigidBody::dynamic(id, spawn, velocity, Vec3i::new(3, 3, 3))\n                    .with_material(Material::new(350)),\n            ))\n",
    "            .add_box(\n                rotating_box(\n                    RigidBody::dynamic(id, spawn, velocity, Vec3i::new(3, 3, 3))\n                        .with_material(Material::new(350)),\n                )\n                .with_collision_layers(controller::scenario_rules::projectile_layers()),\n            )\n",
)

Path("demo-wasm/src/scenario_rules.rs").write_text('''use std::cell::Cell;

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
const KNOWN_BITS: i32 = EXPLICIT_RULES_BIT | CHARACTER_LINEAR_BIT | CRATE_UPRIGHT_BIT | ALL_PAIR_BITS;

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
        sandbox.world.remove_box(id)?;
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
''')

Path("demo-wasm/src/baking.rs").write_text('''use physics_engine::{FIXED_GEOMETRY_PREPARATION_VERSION, FixedGeometryPreparationMode3d};

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

    use crate::Sandbox;

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
''')

Path("tests/collision_layers.rs").write_text('''use physics_engine::{
    AngularState3d, AngularVelocity3d, BodyId, CollisionLayers3d, Orientation3d, RigidBody,
    RigidBox3d, RigidBoxFreeFlightConfig3d, RotatingWorld3d, RotatingWorldConfig3d, Vec3i,
    body_has_support, rotational_sweep_candidate_pairs,
};

fn dynamic(id: u64, position: Vec3i, layers: CollisionLayers3d) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::dynamic(BodyId(id), position, Vec3i::ZERO, Vec3i::new(2, 2, 2)),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("valid dynamic box")
    .with_collision_layers(layers)
}

fn fixed(id: u64, position: Vec3i, layers: CollisionLayers3d) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::fixed(BodyId(id), position, Vec3i::new(2, 2, 2)),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    )
    .expect("valid fixed box")
    .with_collision_layers(layers)
}

#[test]
fn broad_phase_excludes_pairs_rejected_by_symmetric_collision_layers() {
    let character = CollisionLayers3d::new(0b0010, 0b0100);
    let crate_layers = CollisionLayers3d::new(0b0100, 0b0010);
    let projectile = CollisionLayers3d::new(0b1000, 0b1000);
    let boxes = [
        dynamic(1, Vec3i::ZERO, character),
        dynamic(2, Vec3i::new(1, 0, 0), crate_layers),
        dynamic(3, Vec3i::new(-1, 0, 0), projectile),
    ];

    let pairs = rotational_sweep_candidate_pairs(
        &boxes,
        RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 0, 1),
    )
    .expect("valid broad phase");

    assert_eq!(pairs.len(), 1);
    assert_eq!(pairs[0].left, BodyId(1));
    assert_eq!(pairs[0].right, BodyId(2));
}

#[test]
fn support_queries_do_not_reintroduce_disabled_pairs_from_the_contact_cache() {
    let mut world = RotatingWorld3d::new(RotatingWorldConfig3d {
        gravity: Vec3i::new(0, -60, 0),
        sample_count: 16,
        refinement_steps: 2,
        solver_passes: 4,
        max_events: 8,
    });
    let character = CollisionLayers3d::new(0b0010, 0b0100);
    let world_only = CollisionLayers3d::new(0b0001, 0b0001);
    world
        .add_box(dynamic(1, Vec3i::new(0, 4, 0), character))
        .expect("character");
    world
        .add_box(fixed(2, Vec3i::ZERO, world_only))
        .expect("floor");

    assert!(
        !body_has_support(&world, BodyId(1), Vec3i::new(0, -60, 0)).expect("support query")
    );
}

#[test]
fn default_layers_preserve_historical_collision_eligibility() {
    let boxes = [
        dynamic(1, Vec3i::ZERO, CollisionLayers3d::ALL),
        dynamic(
            2,
            Vec3i::new(1, 0, 0),
            CollisionLayers3d::default(),
        ),
    ];
    let pairs = rotational_sweep_candidate_pairs(
        &boxes,
        RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 0, 1),
    )
    .expect("valid broad phase");
    assert_eq!(pairs.len(), 1);
}
''')

Path("site/simulation-rules-config.mjs").write_text('''export const EXPLICIT_RULES_BIT = 1 << 29;
export const CHARACTER_LINEAR_BIT = 1;
export const CRATE_UPRIGHT_BIT = 1 << 11;

export const COLLISION_PAIRS = [
  ["world-world", 1 << 1],
  ["world-character", 1 << 2],
  ["world-crate", 1 << 3],
  ["world-projectile", 1 << 4],
  ["character-character", 1 << 5],
  ["character-crate", 1 << 6],
  ["character-projectile", 1 << 7],
  ["crate-crate", 1 << 8],
  ["crate-projectile", 1 << 9],
  ["projectile-projectile", 1 << 10],
];

const PAIR_KEYS = new Set(COLLISION_PAIRS.map(([key]) => key));

export function enabledPairsFromQuery(value) {
  if (value == null || value === "all") return new Set(PAIR_KEYS);
  if (value === "none") return new Set();
  return new Set(value.split(",").filter((key) => PAIR_KEYS.has(key)));
}

export function enabledPairsToQuery(enabledPairs) {
  const enabled = COLLISION_PAIRS.map(([key]) => key).filter((key) => enabledPairs.has(key));
  if (enabled.length === COLLISION_PAIRS.length) return "all";
  if (enabled.length === 0) return "none";
  return enabled.join(",");
}

export function encodeScenarioRules({ characterResponse, crateMotion, enabledPairs }) {
  let encoded = EXPLICIT_RULES_BIT;
  if (characterResponse === "linear") encoded |= CHARACTER_LINEAR_BIT;
  if (crateMotion === "upright") encoded |= CRATE_UPRIGHT_BIT;
  for (const [key, bit] of COLLISION_PAIRS) {
    if (enabledPairs.has(key)) encoded |= bit;
  }
  return encoded;
}
''')

Path("site/simulation-rules.mjs").write_text('''import {
  COLLISION_PAIRS,
  encodeScenarioRules,
  enabledPairsFromQuery,
  enabledPairsToQuery,
} from "./simulation-rules-config.mjs";

const legacyCharacter = document.querySelector("#character-mode");
const legacyUprightCrates = document.querySelector("#upright-crates");
const characterResponse = document.querySelector("#character-response");
const crateMotion = document.querySelector("#crate-motion");
const fixedGeometry = document.querySelector("#fixed-geometry-mode");
const reset = document.querySelector("#reset");
const pairControls = new Map(
  [...document.querySelectorAll("[data-collision-pair]")].map((control) => [
    control.dataset.collisionPair,
    control,
  ]),
);

const initial = new URL(window.location.href);
characterResponse.value =
  initial.searchParams.get("response") ??
  (initial.searchParams.get("character") === "physical" ? "physical" : "linear");
crateMotion.value =
  initial.searchParams.get("crate-motion") ??
  (initial.searchParams.has("crates")
    ? initial.searchParams.get("crates") === "upright"
      ? "upright"
      : "free"
    : "free");
const enabledPairs = enabledPairsFromQuery(initial.searchParams.get("collisions"));
for (const [key] of COLLISION_PAIRS) {
  const control = pairControls.get(key);
  if (control) control.checked = enabledPairs.has(key);
}
legacyUprightCrates.checked = crateMotion.value === "upright";

function selectedPairs() {
  return new Set(
    COLLISION_PAIRS.map(([key]) => key).filter((key) => pairControls.get(key)?.checked),
  );
}

function encodedRules() {
  return encodeScenarioRules({
    characterResponse: characterResponse.value,
    crateMotion: crateMotion.value,
    enabledPairs: selectedPairs(),
  });
}

Object.defineProperty(legacyCharacter, "value", {
  configurable: true,
  get() {
    return String(encodedRules());
  },
  set(value) {
    if (String(value) === "0") characterResponse.value = "physical";
    if (String(value) === "1") characterResponse.value = "linear";
  },
});

function syncUrl() {
  legacyUprightCrates.checked = crateMotion.value === "upright";
  const url = new URL(window.location.href);
  url.searchParams.set("response", characterResponse.value);
  url.searchParams.set("crate-motion", crateMotion.value);
  url.searchParams.set("collisions", enabledPairsToQuery(selectedPairs()));
  url.searchParams.set("character", characterResponse.value === "physical" ? "physical" : "linear");
  url.searchParams.set("crates", crateMotion.value === "upright" ? "upright" : "free");
  window.history.replaceState(null, "", url);
}

syncUrl();

const scenarioControls = [characterResponse, crateMotion, ...pairControls.values()];
for (const control of scenarioControls) {
  control.addEventListener("keydown", (event) => event.stopPropagation());
  control.addEventListener("change", () => {
    syncUrl();
    reset.click();
  });
}

function syncDisabledState() {
  for (const control of scenarioControls) control.disabled = legacyCharacter.disabled;
}
const disabledObserver = new MutationObserver(syncDisabledState);
disabledObserver.observe(legacyCharacter, { attributes: true, attributeFilter: ["disabled"] });
syncDisabledState();

fixedGeometry.addEventListener("change", () => queueMicrotask(syncUrl));
''')

Path("site/simulation-rules-config.test.mjs").write_text('''import assert from "node:assert/strict";
import test from "node:test";

import {
  CHARACTER_LINEAR_BIT,
  COLLISION_PAIRS,
  CRATE_UPRIGHT_BIT,
  EXPLICIT_RULES_BIT,
  encodeScenarioRules,
  enabledPairsFromQuery,
  enabledPairsToQuery,
} from "./simulation-rules-config.mjs";

test("default puzzle-friendly rules keep all collision pairs and free crate rotation", () => {
  const enabledPairs = enabledPairsFromQuery("all");
  const encoded = encodeScenarioRules({
    characterResponse: "linear",
    crateMotion: "free",
    enabledPairs,
  });
  assert.notEqual(encoded & EXPLICIT_RULES_BIT, 0);
  assert.notEqual(encoded & CHARACTER_LINEAR_BIT, 0);
  assert.equal(encoded & CRATE_UPRIGHT_BIT, 0);
  assert.equal(enabledPairsToQuery(enabledPairs), "all");
  assert.equal(enabledPairs.size, COLLISION_PAIRS.length);
});

test("collision query round-trips deterministic subsets", () => {
  const enabled = enabledPairsFromQuery("character-crate,crate-projectile");
  assert.deepEqual([...enabled], ["character-crate", "crate-projectile"]);
  assert.equal(enabledPairsToQuery(enabled), "character-crate,crate-projectile");
});

test("response axes remain independent", () => {
  const encoded = encodeScenarioRules({
    characterResponse: "physical",
    crateMotion: "upright",
    enabledPairs: enabledPairsFromQuery("none"),
  });
  assert.equal(encoded & CHARACTER_LINEAR_BIT, 0);
  assert.notEqual(encoded & CRATE_UPRIGHT_BIT, 0);
  assert.equal(enabledPairsToQuery(new Set()), "none");
});
''')

# Replace the old coarse comparison fieldset with scenario rules + separate engine preparation.
index = Path("site/index.html")
html = index.read_text()
old = '''        <fieldset>
          <legend>Comparison options</legend>
          <label for="character-mode">Character interaction</label>
          <select id="character-mode" disabled>
            <option value="1">Linear pushes · stable crate landings</option>
            <option value="0">Physical impacts · original response</option>
          </select>
          <label>
            <input id="upright-crates" type="checkbox" checked disabled>
            Keep crates upright (including projectile impacts)
          </label>
          <label for="fixed-geometry-mode">Fixed collision geometry</label>
          <select id="fixed-geometry-mode" disabled>
            <option value="0">Runtime preparation · reference path</option>
            <option value="1">Prepare fixed geometry at load</option>
          </select>
          <p>
            The three options are independent and stored in the URL. Prepare-at-load reuses immutable fixed
            collision geometry only; it does not bake motion or sleeping dynamic bodies. Changing any option
            resets the scene.
          </p>
        </fieldset>'''
new = '''        <fieldset>
          <legend>Simulation rules</legend>
          <p>
            These scenario rules are authoritative for interaction eligibility and response. Changing a rule
            resets the scene so comparisons start from identical initial state.
          </p>
          <label for="character-response">Character response</label>
          <select id="character-response" disabled>
            <option value="linear">Constrained linear push · no induced rotation</option>
            <option value="physical">Full physical impact</option>
          </select>
          <label for="crate-motion">Crate motion</label>
          <select id="crate-motion" disabled>
            <option value="free">Free rigid-body rotation</option>
            <option value="upright">Translation only · keep upright</option>
          </select>
          <p>Projectiles always use full rigid-body response, including off-center torque.</p>
          <table class="rules-table">
            <thead><tr><th>Collision pair</th><th>Enabled</th></tr></thead>
            <tbody>
              <tr><td>World ↔ world</td><td><input type="checkbox" data-collision-pair="world-world" checked disabled></td></tr>
              <tr><td>World ↔ character</td><td><input type="checkbox" data-collision-pair="world-character" checked disabled></td></tr>
              <tr><td>World ↔ crate</td><td><input type="checkbox" data-collision-pair="world-crate" checked disabled></td></tr>
              <tr><td>World ↔ projectile</td><td><input type="checkbox" data-collision-pair="world-projectile" checked disabled></td></tr>
              <tr><td>Character ↔ character</td><td><input type="checkbox" data-collision-pair="character-character" checked disabled></td></tr>
              <tr><td>Character ↔ crate</td><td><input type="checkbox" data-collision-pair="character-crate" checked disabled></td></tr>
              <tr><td>Character ↔ projectile</td><td><input type="checkbox" data-collision-pair="character-projectile" checked disabled></td></tr>
              <tr><td>Crate ↔ crate</td><td><input type="checkbox" data-collision-pair="crate-crate" checked disabled></td></tr>
              <tr><td>Crate ↔ projectile</td><td><input type="checkbox" data-collision-pair="crate-projectile" checked disabled></td></tr>
              <tr><td>Projectile ↔ projectile</td><td><input type="checkbox" data-collision-pair="projectile-projectile" checked disabled></td></tr>
            </tbody>
          </table>
          <select id="character-mode" hidden disabled>
            <option value="1">Linear</option>
            <option value="0">Physical</option>
          </select>
          <input id="upright-crates" type="checkbox" hidden disabled>
        </fieldset>
        <fieldset>
          <legend>Engine preparation</legend>
          <label for="fixed-geometry-mode">Fixed collision geometry</label>
          <select id="fixed-geometry-mode" disabled>
            <option value="0">Runtime preparation · reference path</option>
            <option value="1">Prepare fixed geometry at load</option>
          </select>
          <p>
            Prepare-at-load reuses immutable fixed collision geometry only; it does not bake motion or
            sleeping dynamic bodies. This is a performance/storage axis, not a gameplay rule.
          </p>
        </fieldset>'''
if html.count(old) != 1:
    raise SystemExit("site/index.html comparison fieldset drifted")
html = html.replace(old, new, 1)
html = html.replace(
    '    <script type="module" src="interaction-controls.mjs"></script>\n',
    '    <script type="module" src="simulation-rules.mjs"></script>\n    <script type="module" src="interaction-controls.mjs"></script>\n',
    1,
)
index.write_text(html)

styles = Path("site/styles.css")
css = styles.read_text()
if ".rules-table" not in css:
    css += '''\n\n.rules-table {\n  width: 100%;\n  border-collapse: collapse;\n  margin-block: 0.75rem;\n}\n\n.rules-table th,\n.rules-table td {\n  text-align: left;\n  padding: 0.4rem 0.6rem;\n  border-bottom: 1px solid currentColor;\n}\n\n.rules-table th:last-child,\n.rules-table td:last-child {\n  width: 6rem;\n  text-align: center;\n}\n'''
styles.write_text(css)

build = Path("scripts/build-pages.sh")
text = build.read_text()
text = text.replace(
    '  "sandbox_fixed_geometry_representation_version",\n',
    '  "sandbox_fixed_geometry_representation_version",\n  "sandbox_simulation_rules",\n',
    1,
)
text = text.replace(
    "node --check site/interaction-controls.mjs\nnode --test site/physics-error.test.mjs site/interaction-controls.test.mjs\n",
    "node --check site/interaction-controls.mjs\nnode --check site/simulation-rules.mjs\nnode --check site/simulation-rules-config.mjs\nnode --test site/physics-error.test.mjs site/interaction-controls.test.mjs site/simulation-rules-config.test.mjs\n",
    1,
)
text = text.replace(
    "test -s pages-dist/interaction-controls.mjs\n",
    "test -s pages-dist/interaction-controls.mjs\ntest -s pages-dist/simulation-rules.mjs\ntest -s pages-dist/simulation-rules-config.mjs\n",
    1,
)
build.write_text(text)

docs = Path("docs/interaction-and-baking-options.md")
doc = docs.read_text()
if "## Scenario-defined simulation rules" not in doc:
    doc += '''\n\n## Scenario-defined simulation rules\n\nThe sandbox scenario now owns an explicit interaction matrix for four roles: world, character, crate,\nand projectile. Each pair can be enabled or disabled independently. The engine represents those choices as\nsymmetric collision-layer memberships/masks and rejects disabled pairs in broad-phase discovery, current\ncontact queries, and persistent-tail contact handling. The browser only edits/serializes the scenario; it\ndoes not filter contacts after the fact.\n\nResponse policy is a separate axis from collision eligibility. The puzzle-friendly default uses the\nengine's constrained linear-push actuator policy for the character, free rigid-body rotation for crates,\nand ordinary physical response for projectiles. Walking into a crate therefore transfers predictable\nlinear motion without inducing torque, while an off-center projectile can still rotate the same crate.\nThe existing character-options benchmark remains the stable comparison workload for physical versus\nlinear character response. Fixed-geometry prepare-at-load remains independent of all gameplay rules.\n\nLegacy Wasm reset values `0` and `1` still mean physical/linear character response with every collision\npair enabled. New scenario-aware callers set the explicit-rules marker bit and encode response plus pair\nrules in the same integer; the legacy crate argument remains accepted only for compatibility. Rule changes\nreset the acceptance world and are stored in URL query state for reproducible comparisons.\n'''
docs.write_text(doc)
