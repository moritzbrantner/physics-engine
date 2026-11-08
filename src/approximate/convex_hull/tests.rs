use super::super::{Config, PreparedTriangleMesh};
use super::*;

fn octahedron() -> Vec<V> {
    vec![
        V(1.0, 0.0, 0.0),
        V(-1.0, 0.0, 0.0),
        V(0.0, 1.0, 0.0),
        V(0.0, -1.0, 0.0),
        V(0.0, 0.0, 1.0),
        V(0.0, 0.0, -1.0),
    ]
}

/// Issue #528's proof: the prepared bytes are shared across worlds, but no mutable query cache is.
#[test]
fn prepared_geometry_shares_bytes_across_worlds_but_not_query_caches() {
    let hull = PreparedConvexHull::new(&octahedron()).unwrap();
    let quad = [
        V::ZERO,
        V(4.0, 0.0, 0.0),
        V(4.0, 0.0, 4.0),
        V(0.0, 0.0, 4.0),
    ];
    let mesh = PreparedTriangleMesh::new(&quad, vec![[0, 1, 2], [0, 2, 3]]).unwrap();
    let material = Material::default();
    let mut worlds = [
        World::new(Config::default()).unwrap(),
        World::new(Config::default()).unwrap(),
    ];
    for world in &mut worlds {
        world
            .add_prepared_convex_hull(&hull, V(0.0, 1.0, 0.0), material, Some(1))
            .unwrap();
        world
            .add_prepared_triangle_mesh(&mesh, V(8.0, 0.0, 0.0), material, Some(2))
            .unwrap();
    }
    let hulls = [worlds[0].hulls[0].as_ref(), worlds[1].hulls[0].as_ref()].map(Option::unwrap);
    assert!(Arc::ptr_eq(&hulls[0].hull, &hulls[1].hull));
    assert!(Arc::ptr_eq(&hulls[0].planes, &hulls[1].planes));
    assert!(!Arc::ptr_eq(hulls[0], hulls[1]));
    // The handle plus each world.
    assert_eq!(Arc::strong_count(&hull.hull), 3);
    let meshes =
        [&worlds[0], &worlds[1]].map(|w| w.triangle_meshes[0].as_ref().unwrap().prepared.clone());
    assert!(Arc::ptr_eq(&meshes[0], &meshes[1]));
}
