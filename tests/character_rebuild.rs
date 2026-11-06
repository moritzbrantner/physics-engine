//! A consumer can rebuild its physics world from authoritative character poses.
use physics_engine::{
    BodyId,
    approximate::{
        Body, CharacterMove, CharacterSettings, Config, HeightfieldDesc, Material, Shape,
        Vector as V, World,
    },
};

fn world(at: V) -> (World, BodyId) {
    let mut world = World::new(Config {
        gravity: V::ZERO,
        length_unit: 100.,
        ..Config::default()
    })
    .unwrap();
    world
        .add_heightfield(
            HeightfieldDesc {
                origin: V(800., 0., -7600.),
                cell_size: (400., 400.),
                columns: 2,
                rows: 2,
                heights: vec![1100., 1100., 1287., 1150.],
                removed_cells: None,
                material: Material::default(),
            },
            Some(1),
        )
        .unwrap();
    let mut body = Body::keyed(2, Shape::capsule(60., 30.), at, 1.);
    body.external = true;
    let id = world.add_body(body).unwrap();
    world
        .add_character(
            id,
            CharacterSettings {
                offset: 0.25,
                ..world.character_settings()
            },
        )
        .unwrap();
    (world, id)
}
fn step(world: &mut World, id: BodyId, desired: V) {
    let mut results = Vec::new();
    world
        .move_characters(
            &[CharacterMove {
                body: id,
                desired,
                exclude: None,
                autostep: None,
            }],
            &mut results,
        )
        .unwrap();
}
#[test]
fn a_rebuilt_character_continues_exactly_on_a_sloped_terrain_cell() {
    for direction in 0..48 {
        let angle = direction as f32 * std::f32::consts::TAU / 48.;
        let desired = V(angle.sin() * 3., -0.2725, angle.cos() * 3.);
        let (mut original, id) = world(V(1116.2186, 1247.0002, -7388.426));
        for tick in 0..60 {
            let at = original.body(id).unwrap().position;
            let (mut rebuilt, other) = world(at);
            step(&mut original, id, desired);
            step(&mut rebuilt, other, desired);
            assert_eq!(
                original.body(id).unwrap().position,
                rebuilt.body(other).unwrap().position,
                "direction {direction}, tick {tick}, rebuilt from {at:?}"
            );
        }
    }
}
