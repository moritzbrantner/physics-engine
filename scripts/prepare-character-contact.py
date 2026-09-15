from pathlib import Path

def replace(path, old, new):
    p = Path(path)
    text = p.read_text()
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f'{path}: expected exactly one patch target, found {count}: {old[:90]!r}')
    p.write_text(text.replace(old, new))

def write(path, text):
    p = Path(path)
    if p.exists():
        raise RuntimeError(f'{path} already exists')
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(text)

replace('src/rigid_box.rs',
    '    AngularError3d, AngularState3d, AngularVelocity3d, BodyId, BodyKind, OrientedBox3d, RigidBody,',
    '    AngularError3d, AngularState3d, AngularVelocity3d, BodyId, BodyKind, OrientedBox3d, RigidBody, Vec3i,')
replace('src/rigid_box.rs', '/// Engine-native rotating cuboid state.', '''/// Optional response policy for externally controlled bodies. Collision discovery is unchanged.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum ContactMode3d {
    #[default]
    Physical,
    /// Normal-only, inelastic linear pushing. Contacts opposing the supplied support direction
    /// resolve this body against an unchanged support, without transferring landing load or torque.
    /// Zero support direction disables one-way support. This is an actuator policy, not a claim of
    /// momentum-conserving rigid-body dynamics. Two actuators use symmetric linear response.
    LinearPush { support_direction: Vec3i },
}

/// Engine-native rotating cuboid state.''')
replace('src/rigid_box.rs', '    pub(crate) rotation_locked: bool,', '    pub(crate) rotation_locked: bool,\n    pub(crate) contact_mode: ContactMode3d,')
replace('src/rigid_box.rs', '            rotation_locked: false,', '            rotation_locked: false,\n            contact_mode: ContactMode3d::Physical,')
replace('src/rigid_box.rs', '    #[must_use]\n    pub const fn body(&self) -> &RigidBody {', '''    /// Opts an externally controlled body into linear-only pushing and one-way support contacts.
    /// Only contacts involving this body change; ordinary objects and projectiles retain rotation.
    #[must_use]
    pub fn with_linear_push(mut self, support_direction: Vec3i) -> Self {
        self.contact_mode = ContactMode3d::LinearPush { support_direction };
        self.with_rotation_locked()
    }

    #[must_use]
    pub const fn contact_mode(&self) -> ContactMode3d {
        self.contact_mode
    }

    #[must_use]
    pub const fn body(&self) -> &RigidBody {''')
replace('src/rigid_box_free_flight.rs', '        rotation_locked: rigid_box.rotation_locked,', '        rotation_locked: rigid_box.rotation_locked,\n        contact_mode: rigid_box.contact_mode,')
replace('src/lib.rs', 'mod math;', 'mod math;\nmod linear_contact;')
replace('src/lib.rs', 'pub use rigid_box::{RigidBox3d, RigidBoxError3d};', 'pub use rigid_box::{ContactMode3d, RigidBox3d, RigidBoxError3d};')
replace('src/obb_response.rs', '''pub fn resolve_obb_contact(
    left: RigidBox3d,
    right: RigidBox3d,
    allow_restitution: bool,
) -> Result<ObbContactResponse3d, ObbContactResponseError3d> {''', '''pub fn resolve_obb_contact(
    left: RigidBox3d,
    right: RigidBox3d,
    allow_restitution: bool,
) -> Result<ObbContactResponse3d, ObbContactResponseError3d> {
    if crate::linear_contact::uses_linear_response(&left, &right) {
        return crate::linear_contact::resolve(left, right);
    }
    resolve_physical_obb_contact(left, right, allow_restitution)
}

pub(crate) fn resolve_physical_obb_contact(
    left: RigidBox3d,
    right: RigidBox3d,
    allow_restitution: bool,
) -> Result<ObbContactResponse3d, ObbContactResponseError3d> {''')
replace('src/obb_response.rs', 'fn validate_pair(left: &RigidBox3d, right: &RigidBox3d)', 'pub(crate) fn validate_pair(left: &RigidBox3d, right: &RigidBox3d)')
replace('src/obb_friction.rs', '''    let left_start = left.clone();
    let right_start = right.clone();
    let mut response = resolve_normal_obb_contact(left, right, allow_restitution)?;''', '''    let linear_only = crate::linear_contact::uses_linear_response(&left, &right);
    let left_start = left.clone();
    let right_start = right.clone();
    let mut response = resolve_normal_obb_contact(left, right, allow_restitution)?;
    // Actuator contacts intentionally do not drag a support or apply off-centre friction torque.
    // Material validation above remains unconditional; ordinary rigid contacts are unchanged.
    if linear_only {
        return Ok(response);
    }''')
write('src/linear_contact.rs', '''use crate::{
    BodyKind, ContactMode3d, ObbContactResponse3d, ObbContactResponseError3d, RigidBox3d, Vec3i,
    obb_contact_seed,
    obb_response::{resolve_physical_obb_contact, validate_pair},
};

pub(crate) fn uses_linear_response(left: &RigidBox3d, right: &RigidBox3d) -> bool {
    left.contact_mode != ContactMode3d::Physical || right.contact_mode != ContactMode3d::Physical
}

fn support_direction(body: &RigidBox3d) -> Option<Vec3i> {
    match body.contact_mode {
        ContactMode3d::Physical => None,
        ContactMode3d::LinearPush { support_direction } => Some(support_direction),
    }
}

fn dot(direction: Vec3i, axis: [i128; 3]) -> Result<i128, ObbContactResponseError3d> {
    [direction.x, direction.y, direction.z]
        .into_iter()
        .zip(axis)
        .try_fold(0_i128, |sum, (component, axis)| {
            i128::from(component)
                .checked_mul(axis)
                .and_then(|value| sum.checked_add(value))
                .ok_or(ObbContactResponseError3d::ArithmeticOverflow)
        })
}

/// Applies the opt-in actuator policy through the existing exact normal solver. Only response-local
/// inverse mass / inertia participation changes. Original body kinds, materials, policy, and existing
/// angular state are restored before returning; no proxy enters world membership or free flight.
pub(crate) fn resolve(
    left: RigidBox3d,
    right: RigidBox3d,
) -> Result<ObbContactResponse3d, ObbContactResponseError3d> {
    validate_pair(&left, &right)?;
    let Some(seed) = obb_contact_seed(left.oriented_box(), right.oriented_box())? else {
        return Ok(ObbContactResponse3d { left, right, contact: None });
    };
    let left_direction = support_direction(&left);
    let right_direction = support_direction(&right);
    let right_supports_left = match (left_direction, right_direction) {
        (Some(direction), None) => dot(direction, seed.axis)? > 0,
        _ => false,
    };
    let left_supports_right = match (left_direction, right_direction) {
        (None, Some(direction)) => dot(direction, seed.axis)? < 0,
        _ => false,
    };
    let mut left_proxy = left.clone();
    let mut right_proxy = right.clone();
    left_proxy.rotation_locked = true;
    right_proxy.rotation_locked = true;
    if right_supports_left {
        right_proxy.body.kind = BodyKind::Fixed;
    }
    if left_supports_right {
        left_proxy.body.kind = BodyKind::Fixed;
    }
    let mut response = resolve_physical_obb_contact(left_proxy, right_proxy, false)?;
    response.left.body.kind = left.body.kind;
    response.right.body.kind = right.body.kind;
    response.left.rotation_locked = left.rotation_locked;
    response.right.rotation_locked = right.rotation_locked;
    response.left.angular = left.angular;
    response.right.angular = right.angular;
    Ok(response)
}
''')
replace('demo-wasm/src/lib.rs', '''    fn new() -> Result<Self, RotatingWorldError3d> {
        let mut world''', '''    fn new() -> Result<Self, RotatingWorldError3d> {
        Self::with_character_mode(false)
    }

    fn with_character_mode(linear_push: bool) -> Result<Self, RotatingWorldError3d> {
        let mut world''')
replace('demo-wasm/src/lib.rs', '''        world.add_box(
            rotating_box(
                RigidBody::dynamic(
                    PLAYER_ID,''', '''        let player = rotating_box(
                RigidBody::dynamic(
                    PLAYER_ID,''')
replace('demo-wasm/src/lib.rs', '''            .with_rotation_locked(),
        )?;

        let crate_positions''', '''            .with_rotation_locked();
        world.add_box(if linear_push {
            player.with_linear_push(Vec3i::new(0, -1, 0))
        } else {
            player
        })?;

        let crate_positions''')
replace('demo-wasm/src/lib.rs', '#[unsafe(no_mangle)]\npub extern "C" fn sandbox_step(move_x:', '''/// Explicit comparison mode: 0 = legacy physical interactions, 1 = linear pushing / passive support.
/// Invalid modes leave the current sandbox untouched. Legacy reset retains the original benchmark.
#[unsafe(no_mangle)]
pub extern "C" fn sandbox_reset_with_character_mode(mode: i32) -> i32 {
    if mode != 0 && mode != 1 {
        return -1;
    }
    let Ok(replacement) = Sandbox::with_character_mode(mode == 1) else {
        return -2;
    };
    with_sandbox_mut(|sandbox| *sandbox = replacement);
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_step(move_x:''')
with Path('demo-wasm/src/lib.rs').open('a') as f:
    f.write('\n#[cfg(test)]\n#[path = "character_interaction_tests.rs"]\nmod character_interaction_tests;\n')
write('tests/linear_contact_policy.rs', '''use physics_engine::{
    AngularState3d, AngularVelocity3d, BodyId, ContactMode3d, Material, Orientation3d,
    RigidBody, RigidBox3d, RigidBoxFreeFlightConfig3d, Vec3i, resolve_obb_contact,
    sample_rigid_box_free_flight,
};

fn body(id: u64, position: Vec3i, velocity: Vec3i, half: Vec3i) -> RigidBox3d {
    RigidBox3d::new(
        RigidBody::dynamic(BodyId(id), position, velocity, half)
            .with_mass(if id == 1 { 4 } else { 2 })
            .with_material(Material::new(0).with_friction(1000)),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
    ).unwrap()
}

#[test]
fn off_centre_push_moves_the_object_without_adding_spin() {
    let player = body(1, Vec3i::new(-22, 30, 0), Vec3i::new(420, 0, 0), Vec3i::new(12, 20, 12))
        .with_rotation_locked();
    let object = body(2, Vec3i::new(0, 10, 0), Vec3i::ZERO, Vec3i::new(10, 10, 10));
    let physical = resolve_obb_contact(player.clone(), object.clone(), true).unwrap();
    assert!(!physical.right.angular().angular_velocity.is_zero(), "fixture must exercise off-centre torque");
    let linear = resolve_obb_contact(player.with_linear_push(Vec3i::new(0, -1, 0)), object.clone(), true).unwrap();
    assert!(linear.right.body().velocity().x > 0);
    assert_eq!(linear.right.angular(), object.angular());
    assert!(!linear.right.rotation_locked());
    assert_eq!(linear.right.body().kind(), object.body().kind());
}

#[test]
fn landing_projects_only_the_actuator_and_preserves_the_support() {
    for reverse in [false, true] {
        let player = body(if reverse { 3 } else { 1 }, Vec3i::new(3, 39, 2), Vec3i::new(0, -600, 0), Vec3i::new(12, 20, 12))
            .with_linear_push(Vec3i::new(0, -1, 0));
        let object = body(2, Vec3i::new(0, 10, 0), Vec3i::ZERO, Vec3i::new(18, 10, 18));
        let response = if reverse {
            resolve_obb_contact(object.clone(), player, true).unwrap()
        } else {
            resolve_obb_contact(player, object.clone(), true).unwrap()
        };
        let (player, support) = if reverse { (response.right, response.left) } else { (response.left, response.right) };
        assert_eq!(support, object);
        assert_eq!(player.body().position().y, 40);
        assert_eq!(player.body().velocity().y, 0);
        assert!(player.angular().angular_velocity.is_zero());
    }
}

#[test]
fn moving_support_keeps_its_velocity_and_existing_rotation() {
    let player = body(1, Vec3i::new(0, 40, 0), Vec3i::new(0, -600, 0), Vec3i::new(12, 20, 12))
        .with_linear_push(Vec3i::new(0, -1, 0));
    let support = RigidBox3d::new(
        RigidBody::dynamic(BodyId(2), Vec3i::new(0, 10, 0), Vec3i::new(50, 20, 0), Vec3i::new(18, 10, 18)),
        AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::new(0, 50000, 0)),
    ).unwrap();
    let response = resolve_obb_contact(player, support.clone(), false).unwrap();
    assert_eq!(response.right, support);
    assert_eq!(response.left.body().velocity().y, 20);
}

#[test]
fn sampled_free_flight_retains_the_opt_in_policy() {
    let player = body(1, Vec3i::ZERO, Vec3i::new(0, -60, 0), Vec3i::new(12, 20, 12))
        .with_linear_push(Vec3i::new(0, -1, 0));
    let sampled = sample_rigid_box_free_flight(&player, RigidBoxFreeFlightConfig3d::new(Vec3i::ZERO, 1, 60), 1, 1).unwrap();
    assert_eq!(sampled.contact_mode(), ContactMode3d::LinearPush { support_direction: Vec3i::new(0, -1, 0) });
    assert!(sampled.rotation_locked());
}
''')
write('demo-wasm/src/character_interaction_tests.rs', '''use super::{PLAYER_ID, Sandbox, rotating_box, sandbox_reset, sandbox_reset_with_character_mode, with_sandbox};
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
    assert_eq!(with_sandbox(|s| s.world.box_by_id(PLAYER_ID).unwrap().contact_mode()), ContactMode3d::Physical);
    assert_eq!(sandbox_reset_with_character_mode(1), 0);
    let before = with_sandbox(|s| s.world.boxes().cloned().collect::<Vec<_>>());
    assert_eq!(sandbox_reset_with_character_mode(42), -1);
    assert_eq!(with_sandbox(|s| s.world.boxes().cloned().collect::<Vec<_>>()), before);
}

#[test]
fn linear_character_pushes_an_offset_crate_without_rotating_it() {
    let mut sandbox = Sandbox::with_character_mode(true).unwrap();
    settle(&mut sandbox);
    let id = BodyId(901);
    sandbox.world.add_box(rotating_box(RigidBody::dynamic(
        id, Vec3i::new(8, 18, 285), Vec3i::ZERO, Vec3i::new(18, 18, 18),
    ).with_mass(2))).unwrap();
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
        assert_eq!(sandbox.step_velocity(x, z, tick == 27), 0, "tick {tick}, detail {}", sandbox.error_detail);
        for original in &before {
            let current = sandbox.world.box_by_id(original.body().id()).unwrap();
            assert_eq!(current.body().position(), original.body().position(), "crate displacement at tick {tick}");
            assert_eq!(current.angular(), original.angular(), "crate angular state at tick {tick}");
        }
        if tick > 50 && sandbox.grounded().unwrap() {
            landed = true;
        }
    }
    assert!(landed);
    assert!(sandbox.world.box_by_id(PLAYER_ID).unwrap().body().position().y >= 90, "player must remain on top, not tunnel through the stack");
    assert!(sandbox.is_quiescent(), "resting scene must return to sleep");
}

#[test]
fn projectiles_still_rotate_crates_in_linear_character_mode() {
    let mut sandbox = Sandbox::with_character_mode(true).unwrap();
    settle(&mut sandbox);
    let before = [BodyId(100), BodyId(101)].map(|id| sandbox.world.box_by_id(id).unwrap().angular());
    assert!(sandbox.shoot(-38, 0, -88) >= 0);
    let mut rotated = false;
    for _ in 0..24 {
        assert_eq!(sandbox.step_velocity(0, 0, false), 0);
        for (index, id) in [BodyId(100), BodyId(101)].into_iter().enumerate() {
            rotated |= sandbox.world.box_by_id(id).unwrap().angular() != before[index];
        }
    }
    assert!(rotated, "projectile dynamics must not be globally rotation-locked");
}
''')
replace('site/app.js', 'const keys = new Set();', '''const keys = new Set();
const characterModeControl = document.querySelector("#character-mode");
const characterParameters = new URLSearchParams(window.location.search);
characterModeControl.value = characterParameters.get("character") === "physical" ? "0" : "1";
characterModeControl.addEventListener("change", () => {
  const url = new URL(window.location.href);
  url.searchParams.set("character", characterModeControl.value === "0" ? "physical" : "linear");
  window.history.replaceState(null, "", url);
  if (engine) reset();
});''')
replace('site/app.js', '  engine.sandbox_reset();', '''  if (engine.sandbox_reset_with_character_mode(Number(characterModeControl.value)) !== 0) {
    throw new Error("Unable to initialize the selected character contact mode");
  }''')
replace('site/app.js', '  ensureCrosshair();\n  reset();', '  ensureCrosshair();\n  characterModeControl.disabled = false;\n  reset();')
html = Path('site/index.html').read_text()
marker = '<div class="viewport-shell">'
if html.count(marker) != 1:
    raise RuntimeError('viewport marker not found')
Path('site/index.html').write_text(html.replace(marker, '''<label for="character-mode">Character interaction (changing this resets the scene)</label>
      <select id="character-mode" disabled>
        <option value="1">Linear pushes · stable crate landings</option>
        <option value="0">Physical impacts · rotational comparison</option>
      </select>
      <div class="viewport-shell">'''))
replace('.github/workflows/validate.yml', '      - name: Build Pages artifact', '''      - name: Test character landing replay
        run: cargo test --manifest-path demo-wasm/Cargo.toml --locked --lib character_interaction_tests::landing_at_a_crate_stack_edge_does_not_disturb_the_crates -- --ignored

      - name: Build Pages artifact''')
write('docs/interaction-and-baking-options.md', '''# Independent physics comparison options

## Character contacts

The interactive page defaults to linear character pushing. `?character=physical` selects the prior rigid-body interaction for comparison; `?character=linear` selects the new mode. Switching modes resets the scene, preventing a previously disturbed pile from contaminating the comparison. The legacy `sandbox_reset` export and `sandbox-projectiles-v1` benchmark keep physical interactions unchanged. `sandbox_reset_with_character_mode(0|1)` explicitly selects either model and rejects invalid inputs without resetting the world.

`RigidBox3d::with_linear_push(support_direction)` is an opt-in actuator response policy. Non-supporting contacts resolve inelastically without angular response or tangential friction. A supporting contact resolves only the actuator against the unchanged other body, including its existing linear velocity. The pair solver restores real body kinds, angular state, and rotation-lock settings before returning. This is not a mass change or a global crate rotation lock. Ordinary crate/projectile contacts still use the existing physical solver. This mode deliberately does not model character weight or transfer landing momentum into a support. It is not a complete kinematic character controller (step climbing, slope limits, and rotating-platform transport remain separate capabilities).

## Optional baking: separate follow-on lane

Baking means preparing immutable static collision data, not recording and replaying predetermined physics motion. Retained static preparation must remain optional and independent of character interaction mode. Keep an uncached/runtime path as the reference; compare it with prepare-at-load and later serialized-bake paths using identical scenes and inputs. No baking toggle is claimed to exist in this slice.

Next acceptance: precompute fixed-body quantized vertices/edges/faces/bounds; retain them across ticks with bounded memory; invalidate on geometry, placement, membership, or policy/version changes; never treat sleeping dynamic crates as permanently static. Verify identical contacts, errors, and replay hashes with baking on/off, including removal and waking. Record startup cost, memory, preparation counts, and steady-state timings on the versioned projectile workload. Do not change dynamic CCD samples, event budgets, or solver semantics to make the baked mode look faster. Add disk bake artifacts only after the load-time option demonstrates value.
''')
Path('scripts/prepare-character-contact.py').unlink()
Path('.github/workflows/character-contact-workbench.yml').unlink()
