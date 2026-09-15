from pathlib import Path
import subprocess

# Match the source layout retained in the failed-run evidence.
subprocess.run(['cargo', 'fmt'], check=True)
subprocess.run(['cargo', 'fmt', '--manifest-path', 'demo-wasm/Cargo.toml'], check=True)

def replace(path, old, new):
    p = Path(path)
    text = p.read_text()
    if text.count(old) != 1:
        raise RuntimeError(f'{path}: non-unique patch target: {old[:100]!r}')
    p.write_text(text.replace(old, new))

for old, new in [
    ('point.component(axis) >= centre', 'point.component(axis) > centre'),
    ('point.component(axis) <= centre', 'point.component(axis) < centre'),
    ('envelope.minimum[axis] >= centre', 'envelope.minimum[axis] > centre'),
    ('envelope.maximum[axis] <= centre', 'envelope.maximum[axis] < centre'),
]:
    replace('src/linear_contact.rs', old, new)

replace('demo-wasm/src/lib.rs',
    '    fn with_character_mode(linear_push: bool) -> Result<Self, RotatingWorldError3d> {\n        let mut world',
    '''    fn with_character_mode(linear_push: bool) -> Result<Self, RotatingWorldError3d> {
        Self::with_options(linear_push, false)
    }

    fn with_options(linear_push: bool, upright_crates: bool) -> Result<Self, RotatingWorldError3d> {
        let mut world''')
replace('demo-wasm/src/lib.rs',
    '            world.add_box(rotating_box(\n                RigidBody::dynamic(\n                    BodyId(100 + offset as u64),',
    '            let crate_body = rotating_box(\n                RigidBody::dynamic(\n                    BodyId(100 + offset as u64),')
replace('demo-wasm/src/lib.rs',
    '''                    Material::new(CRATE_RESTITUTION_MILLI).with_friction(CRATE_FRICTION_MILLI),
                ),
            ))?;''',
    '''                    Material::new(CRATE_RESTITUTION_MILLI).with_friction(CRATE_FRICTION_MILLI),
                ),
            );
            world.add_box(if upright_crates { crate_body.with_rotation_locked() } else { crate_body })?;''')
replace('demo-wasm/src/lib.rs',
    '#[unsafe(no_mangle)]\npub extern "C" fn sandbox_step(move_x:',
    '''/// Independent comparison axes. Invalid options do not mutate the current scene.
#[unsafe(no_mangle)]
pub extern "C" fn sandbox_reset_with_options(character_mode: i32, upright_crates: i32) -> i32 {
    if !(0..=1).contains(&character_mode) || !(0..=1).contains(&upright_crates) {
        return -1;
    }
    let Ok(replacement) = Sandbox::with_options(character_mode == 1, upright_crates == 1) else {
        return -2;
    };
    with_sandbox_mut(|sandbox| *sandbox = replacement);
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn sandbox_step(move_x:''')

replace('demo-wasm/src/character_interaction_tests.rs',
    '''fn linear_character_pushes_an_offset_crate_without_rotating_it() {
    let mut sandbox = Sandbox::with_character_mode(true).unwrap();''',
    '''fn upright_comparison_pushes_a_rough_offset_crate_without_rotating_it() {
    let mut sandbox = Sandbox::with_options(true, true).unwrap();''')
replace('demo-wasm/src/character_interaction_tests.rs',
    '''            .with_material(physics_engine::Material::new(0).with_friction(1000)),
        ))''',
    '''            .with_material(physics_engine::Material::new(0).with_friction(1000)),
        ).with_rotation_locked())''')
with Path('demo-wasm/src/character_interaction_tests.rs').open('a') as f:
    f.write('''
#[test]
fn upright_and_character_response_options_are_independent() {
    for linear in [false, true] {
        for upright in [false, true] {
            let sandbox = Sandbox::with_options(linear, upright).unwrap();
            assert_eq!(sandbox.world.box_by_id(BodyId(100)).unwrap().rotation_locked(), upright);
            assert_eq!(sandbox.world.box_by_id(PLAYER_ID).unwrap().contact_mode() != ContactMode3d::Physical, linear);
            assert_eq!(sandbox.world.entity_count(), 18);
        }
    }
    assert_eq!(super::sandbox_reset_with_options(1, 1), 0);
    let before = with_sandbox(|s| s.world.boxes().cloned().collect::<Vec<_>>());
    assert_eq!(super::sandbox_reset_with_options(1, 2), -1);
    assert_eq!(super::sandbox_reset_with_options(-1, 0), -1);
    assert_eq!(with_sandbox(|s| s.world.boxes().cloned().collect::<Vec<_>>()), before);
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
    assert!(moved, "upright does not mean fixed or immune to projectiles");
}
''')
with Path('src/linear_contact.rs').open('a') as f:
    f.write('''
#[cfg(test)]
mod tests {
    use super::sweep_is_passive_support;
    use crate::{AngularState3d, AngularVelocity3d, BodyId, Orientation3d, RigidBody, RigidBox3d, RotationalSweepBounds3d, Vec3i};

    fn body(id: u64) -> RigidBox3d {
        RigidBox3d::new(RigidBody::dynamic(BodyId(id), Vec3i::ZERO, Vec3i::ZERO, Vec3i::new(1, 1, 1)),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default())).unwrap()
    }

    #[test]
    fn only_proven_passive_half_spaces_skip_waking() {
        let support = body(2);
        for axis in 0..3 {
            for sign in [-1, 1] {
                let mut components = [0; 3];
                components[axis] = sign;
                let actuator = body(1).with_linear_push(Vec3i::new(components[0], components[1], components[2]));
                let mut envelope = RotationalSweepBounds3d { minimum: [-2; 3], maximum: [2; 3] };
                assert!(!sweep_is_passive_support(&actuator, envelope, &support));
                if sign < 0 { envelope.minimum[axis] = 1; } else { envelope.maximum[axis] = -1; }
                assert!(sweep_is_passive_support(&actuator, envelope, &support));
                if sign < 0 { envelope.minimum[axis] = 0; } else { envelope.maximum[axis] = 0; }
                assert!(!sweep_is_passive_support(&actuator, envelope, &support), "centre-level sides must remain pushable");
            }
        }
        let envelope = RotationalSweepBounds3d { minimum: [1; 3], maximum: [2; 3] };
        for direction in [Vec3i::ZERO, Vec3i::new(-1, -1, 0)] {
            assert!(!sweep_is_passive_support(&body(1).with_linear_push(direction), envelope, &support));
        }
        assert!(!sweep_is_passive_support(&body(1), envelope, &support), "physical impacts always use normal waking");
    }
}
''')

replace('site/app.js',
    'const characterParameters = new URLSearchParams(window.location.search);',
    'const uprightCratesControl = document.querySelector("#upright-crates");\nconst characterParameters = new URLSearchParams(window.location.search);')
replace('site/app.js',
    '''characterModeControl.addEventListener("change", () => {
  const url = new URL(window.location.href);
  url.searchParams.set("character", characterModeControl.value === "0" ? "physical" : "linear");
  window.history.replaceState(null, "", url);
  if (engine) reset();
});''',
    '''uprightCratesControl.checked = characterParameters.get("crates") !== "free";
function resetInteractionOptions() {
  const url = new URL(window.location.href);
  url.searchParams.set("character", characterModeControl.value === "0" ? "physical" : "linear");
  url.searchParams.set("crates", uprightCratesControl.checked ? "upright" : "free");
  window.history.replaceState(null, "", url);
  if (engine) reset();
}
characterModeControl.addEventListener("change", resetInteractionOptions);
uprightCratesControl.addEventListener("change", resetInteractionOptions);''')
replace('site/app.js',
    '  if (engine.sandbox_reset_with_character_mode(Number(characterModeControl.value)) !== 0) {',
    '  if (engine.sandbox_reset_with_options(Number(characterModeControl.value), Number(uprightCratesControl.checked)) !== 0) {')
replace('site/app.js',
    '  characterModeControl.disabled = false;',
    '  characterModeControl.disabled = false;\n  uprightCratesControl.disabled = false;')
replace('site/index.html',
    '      <div class="viewport-shell">',
    '''      <label><input id="upright-crates" type="checkbox" checked disabled> Keep crates upright (including projectile impacts)</label>
      <p>Disable upright crates to compare full rotation. Changing either option resets the scene.</p>
      <div class="viewport-shell">''')

with Path('docs/interaction-and-baking-options.md').open('a') as f:
    f.write('''
## Upright crates are an independent constraint

The page defaults to `character=linear&crates=upright`. The upright option uses the existing engine rotation lock, not pose overwrites, increased damping, or a fixed-body substitution. Crates still translate under pushes and projectile impacts. Floor friction otherwise can create torque after an initially torque-free character push, so the checkbox explicitly suppresses **all** crate rotation, including projectile-induced rotation. Disable it with `crates=free` to test natural crate/projectile rotation while retaining the new passive character landings. `character=physical&crates=free` restores the original comparison. Neither axis is a baking option.

The landing replay is deliberately tested with free crate rotation, so it cannot pass by hiding the reported instability behind the upright constraint. Centre-level side contacts still push; only an actuator wholly strictly above a support centre falls inside the passive half-space. Existing physical replay fingerprints must remain identical, but comparison modes intentionally have different motion and must not be called equivalent performance optimizations.
''')

Path('scripts/finish-character-contact.py').unlink()
