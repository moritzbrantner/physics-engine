from pathlib import Path

def replace(path, old, new):
    p = Path(path)
    text = p.read_text()
    if text.count(old) != 1:
        raise RuntimeError(f'{path}: patch target is not unique: {old[:80]!r}')
    p.write_text(text.replace(old, new))

replace('src/linear_contact.rs',
    '(Some(direction), None) => dot(direction, seed.axis)? > 0,',
    '(Some(direction), None) => dot(direction, seed.axis)? > 0 || current_shape_is_passive_support(&left, &right)?,')
replace('src/linear_contact.rs',
    '(None, Some(direction)) => dot(direction, seed.axis)? < 0,',
    '(None, Some(direction)) => dot(direction, seed.axis)? < 0 || current_shape_is_passive_support(&right, &left)?,')
with Path('src/linear_contact.rs').open('a') as f:
    f.write('''
// Cardinal support directions additionally define a passive support half-space: when every point
// of the actuator lies above the other body's centre, a shallow edge/axis tie must not turn landing
// into a horizontal kick. The ground/support query still uses actual contact normals.
fn cardinal_support_axis(body: &RigidBox3d) -> Option<(usize, i32)> {
    let direction = support_direction(body)?;
    let components = [direction.x, direction.y, direction.z];
    let mut nonzero = components.into_iter().enumerate().filter(|(_, value)| *value != 0);
    let (axis, value) = nonzero.next()?;
    if nonzero.next().is_some() {
        return None;
    }
    Some((axis, value.signum()))
}

fn current_shape_is_passive_support(
    actuator: &RigidBox3d,
    other: &RigidBox3d,
) -> Result<bool, ObbContactResponseError3d> {
    let Some((axis, sign)) = cardinal_support_axis(actuator) else {
        return Ok(false);
    };
    let centre = other.body.position.component(axis);
    let vertices = crate::oriented_box_vertices(actuator.oriented_box())?;
    Ok(vertices.iter().all(|point| {
        if sign < 0 { point.component(axis) >= centre } else { point.component(axis) <= centre }
    }))
}

/// A conservative envelope entirely inside the passive support half-space cannot transmit a push
/// to this sleeper under the actuator policy. Keep the sleeper as an ordinary collision-tested fixed
/// proxy, not as absent geometry. Side pushes, projectiles, and unproven/general directions retain the
/// existing conservative wake path. This uses the same half-space as response, not a sleep threshold.
pub(crate) fn sweep_is_passive_support(
    actuator: &RigidBox3d,
    envelope: crate::RotationalSweepBounds3d,
    other: &RigidBox3d,
) -> bool {
    if other.contact_mode != ContactMode3d::Physical {
        return false;
    }
    let Some((axis, sign)) = cardinal_support_axis(actuator) else {
        return false;
    };
    let centre = i64::from(other.body.position.component(axis));
    if sign < 0 { envelope.minimum[axis] >= centre } else { envelope.maximum[axis] <= centre }
}
''')
replace('src/stabilized_rotating_world.rs',
    '.any(|(_, bounds)| sweep_bounds_overlap(*bounds, *sleeping_bounds))',
    '''.any(|(awake_id, bounds)| {
                                sweep_bounds_overlap(*bounds, *sleeping_bounds)
                                    && !self.inner.box_by_id(*awake_id).is_some_and(|awake| {
                                        self.inner.box_by_id(*id).is_some_and(|sleeping| {
                                            crate::linear_contact::sweep_is_passive_support(awake, *bounds, sleeping)
                                        })
                                    })
                            })''')
# Strengthen the push acceptance to use the real rough-crate material, not a frictionless test box.
replace('demo-wasm/src/character_interaction_tests.rs',
    ').with_mass(2))).unwrap();',
    ').with_mass(2).with_material(physics_engine::Material::new(0).with_friction(1000)))).unwrap();')
with Path('docs/interaction-and-baking-options.md').open('a') as f:
    f.write('\n### Passive-support sleep participation\n\nFor cardinal support directions, a contact from entirely above the support centre remains passive even at a shallow edge or SAT-axis tie. A conservative actuator sweep wholly inside this same half-space does not wake a sleeping support. The support remains collision-tested and still wakes for side pushes, other dynamic impacts, or topology changes. General directions use the conservative wake fallback. Grounding is still decided by actual engine contact normals.\n')
Path('scripts/refine-character-contact.py').unlink()
