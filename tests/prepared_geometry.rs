//! Prepared hulls and meshes (#528): admission through a shared preparation is exactly
//! `add_convex_hull` / `add_triangle_mesh`.
use physics_engine::approximate::{
    Body, Config, ConvexHullDesc, Material, PreparedConvexHull, PreparedTriangleMesh, QueryFilter,
    RayCast, Shape, TriangleMeshDesc, Vector as V, World,
};

fn hull_vertices() -> Vec<V> {
    vec![
        V(1.0, 0.0, 0.0),
        V(-1.0, 0.0, 0.0),
        V(0.0, 1.0, 0.0),
        V(0.0, -1.0, 0.0),
        V(0.0, 0.0, 1.0),
        V(0.0, 0.0, -1.0),
    ]
}

const QUAD: [V; 4] = [
    V(-4.0, 0.0, -4.0),
    V(4.0, 0.0, -4.0),
    V(4.0, 0.0, 4.0),
    V(-4.0, 0.0, 4.0),
];
const TRIANGLES: [[u32; 3]; 2] = [[0, 1, 2], [0, 2, 3]];

fn with_dynamic(mut w: World) -> World {
    let mut ball = Body::keyed(3, Shape::Sphere(0.25), V(0.0, 1.6, 0.05), 1.0);
    ball.sleep_allowed = false;
    w.add_body(ball).unwrap();
    w
}

fn from_descriptions() -> World {
    let mut w = World::new(Config::default()).unwrap();
    w.add_convex_hull(
        ConvexHullDesc {
            vertices: hull_vertices(),
            position: V(0.0, 0.0, 0.0),
            material: Material::default(),
        },
        Some(1),
    )
    .unwrap();
    w.add_triangle_mesh(
        TriangleMeshDesc {
            position: V(0.0, -2.0, 0.0),
            vertices: QUAD.to_vec(),
            triangles: TRIANGLES.to_vec(),
            material: Material::default(),
        },
        Some(2),
    )
    .unwrap();
    with_dynamic(w)
}

fn from_prepared(hull: &PreparedConvexHull, mesh: &PreparedTriangleMesh) -> World {
    let mut w = World::new(Config::default()).unwrap();
    w.add_prepared_convex_hull(hull, V(0.0, 0.0, 0.0), Material::default(), Some(1))
        .unwrap();
    w.add_prepared_triangle_mesh(mesh, V(0.0, -2.0, 0.0), Material::default(), Some(2))
        .unwrap();
    with_dynamic(w)
}

#[test]
fn prepared_admission_gives_bit_identical_steps_and_queries_in_every_world() {
    let hull = PreparedConvexHull::new(&hull_vertices()).unwrap();
    let mesh = PreparedTriangleMesh::new(&QUAD, TRIANGLES.to_vec()).unwrap();
    let mut worlds = [
        from_descriptions(),
        from_prepared(&hull, &mesh),
        from_prepared(&hull, &mesh),
    ];
    for _ in 0..120 {
        let states = worlds.each_mut().map(|w| {
            w.step(1.0 / 60.0).unwrap();
            let body = w.bodies().find(|b| b.key == Some(3)).unwrap().clone();
            let ray = RayCast::new(V(0.2, 3.0, 0.1), V(0.0, -6.0, 0.0));
            let hit = w.cast_ray_first(ray, QueryFilter::default()).unwrap();
            (
                body.position,
                body.velocity,
                body.orientation,
                hit.map(|h| (h.fraction.to_bits(), h.normal)),
            )
        });
        assert_eq!(states[0], states[1]);
        assert_eq!(states[1], states[2]);
    }
}

#[test]
fn prepared_admission_validates_and_rolls_back_like_the_descriptions() {
    assert!(PreparedConvexHull::new(&[V::ZERO]).is_err());
    assert!(PreparedTriangleMesh::new(&[], TRIANGLES.to_vec()).is_err());
    let hull = PreparedConvexHull::new(&hull_vertices()).unwrap();
    let mesh = PreparedTriangleMesh::new(&QUAD, TRIANGLES.to_vec()).unwrap();
    let mut w = World::new(Config::default()).unwrap();
    let material = Material::default();
    assert!(
        w.add_prepared_convex_hull(&hull, V(f32::NAN, 0.0, 0.0), material, None)
            .is_err()
    );
    assert!(
        w.add_prepared_triangle_mesh(&mesh, V(1e13, 0.0, 0.0), material, None)
            .is_err()
    );
    w.add_prepared_convex_hull(&hull, V::ZERO, material, Some(7))
        .unwrap();
    assert!(
        w.add_prepared_triangle_mesh(&mesh, V::ZERO, material, Some(7))
            .is_err()
    );
    assert_eq!(w.body_count(), 1);
    // The handle stays usable after the rejected admission.
    w.add_prepared_triangle_mesh(&mesh, V::ZERO, material, Some(8))
        .unwrap();
    assert_eq!(w.body_count(), 2);
}
