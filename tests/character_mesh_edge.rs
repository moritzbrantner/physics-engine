use physics_engine::approximate::{
    Body, CharacterMove, Config, Shape, TriangleMeshDesc, Vector as V, World,
};
#[path = "support/brook_mesh.rs"]
mod brook;

fn world(unit: f32, start: [f32; 3]) -> (World, physics_engine::BodyId) {
    let scale = unit / 100.0;
    let mut w = World::new(Config {
        gravity: V::ZERO,
        length_unit: unit,
        ..Config::default()
    })
    .unwrap();
    w.add_triangle_mesh(
        TriangleMeshDesc {
            position: V::ZERO,
            vertices: brook::VERTICES
                .map(|p| V(p[0] * scale, p[1] * scale, p[2] * scale))
                .to_vec(),
            triangles: brook::TRIANGLES.to_vec(),
            material: Default::default(),
        },
        Some(65125),
    )
    .unwrap();
    let mut b = Body::keyed(
        1000001,
        Shape::capsule(0.6 * unit, 0.3 * unit),
        V(start[0] * scale, start[1] * scale, start[2] * scale),
        1.0,
    );
    b.external = true;
    let id = w.add_body(b).unwrap();
    let mut settings = w.character_settings();
    settings.offset = 0.0025 * unit;
    w.add_character(id, settings).unwrap();
    (w, id)
}
#[test]
fn grazing_authored_mesh_edge_preserves_height_then_loses_support() {
    for unit in [1.0, 100.0] {
        let (mut w, id) = world(unit, brook::START);
        let mut results = Vec::new();
        let step = CharacterMove {
            body: id,
            desired: V(-0.105 * unit, 0.0, 0.0),
            exclude: None,
            autostep: None,
        };
        w.move_characters(&[step], &mut results).unwrap();
        assert!(
            results[0].translation.1 >= -0.001 * unit,
            "unit {unit}: {:?}",
            results[0]
        );
        w.move_characters(&[step], &mut results).unwrap();
        assert!(
            !results[0].ground_contact,
            "a departed edge must lose support: {:?}",
            results[0]
        );
        let falling = CharacterMove {
            desired: V(-0.105 * unit, -0.01 * unit, 0.0),
            ..step
        };
        w.move_characters(&[falling], &mut results).unwrap();
        assert!(results[0].translation.1 <= -0.009 * unit);
        assert!(!results[0].ground_contact);
    }
}
#[test]
fn unsupported_start_and_underside_do_not_gain_support_from_grazing() {
    for unit in [1.0, 100.0] {
        for start in [
            [-3610.0, 96.25, 2142.5002],
            [-3558.4663, -104.25, 2142.5002],
        ] {
            let (mut w, id) = world(unit, start);
            let mut results = Vec::new();
            w.move_characters(
                &[CharacterMove {
                    body: id,
                    desired: V(-0.105 * unit, -0.01 * unit, 0.0),
                    exclude: None,
                    autostep: None,
                }],
                &mut results,
            )
            .unwrap();
            assert!(results[0].translation.1 < 0.0);
            assert!(!results[0].ground_contact, "{start:?} {:?}", results[0]);
        }
    }
}
