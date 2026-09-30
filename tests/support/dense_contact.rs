use physics_engine::{
    BodyId,
    approximate::{Body, Config, PositionCorrection, Quaternion, Shape, Vector as V, World},
};

pub fn fixture(substeps: u8) -> World {
    let mut world = World::new(Config {
        substeps,
        velocity_iterations: 8,
        fixed_position_iterations: 2,
        position_correction: PositionCorrection::AdmittedContacts,
        ..Default::default()
    })
    .unwrap();
    for (id, position, velocity, orientation, angular) in [
        (
            148,
            V(-126.1724600442618, 54.005912969535785, 17.948062019573694),
            V(
                0.33645826561041753,
                -0.40791252039782216,
                -0.14353529032144557,
            ),
            Quaternion(
                0.0008518104306974698,
                -0.0008802192116679719,
                0.00020802533959284725,
                0.9999992281789961,
            ),
            V(
                -0.006049529622565041,
                0.01758089728459939,
                -0.01920306929658203,
            ),
        ),
        (
            156,
            V(-126.21560657580625, 54.04835894532563, 54.04957942376356),
            V(0.36832484060884035, -1.323021069237208, 0.25824421296230343),
            Quaternion(
                0.0014380893170550986,
                -0.0037902009808230956,
                0.0014393037189268923,
                0.9999907472974167,
            ),
            V(
                0.007943020384183397,
                -0.02659753230351231,
                -0.02354060693292012,
            ),
        ),
    ] {
        let mut body = Body::new(BodyId(id), Shape::Box(V(18.0, 18.0, 18.0)), position, 2.0);
        body.velocity = velocity;
        body.orientation = orientation;
        body.angular_velocity = angular;
        body.friction = 1.0;
        body.sleep_allowed = false;
        world.add_body(body).unwrap();
    }
    let mut projectile = Body::new(
        BodyId(100002),
        Shape::Box(V(1.0, 1.0, 9.0)),
        V(-122.0, 49.375, 138.0),
        1.0,
    );
    projectile.velocity = V(0.0, -60.0, -5760.0);
    projectile.friction = 0.0;
    projectile.rotation_locked = true;
    projectile.sleep_allowed = false;
    projectile.ccd = true;
    projectile.retire_on_impact = true;
    world.add_body(projectile).unwrap();
    world
}
// Independent geometric screening, not the engine's clipped response manifold.
pub fn penetration(a: &Body, b: &Body) -> f64 {
    let Shape::Box(ha) = a.shape else {
        panic!("box fixture")
    };
    let Shape::Box(hb) = b.shape else {
        panic!("box fixture")
    };
    let aa = a.orientation.axes();
    let bb = b.orientation.axes();
    let delta = b.position - a.position;
    let mut depth = f64::INFINITY;
    for axis in aa
        .into_iter()
        .chain(bb)
        .chain(aa.into_iter().flat_map(|u| bb.map(|v| u.cross(v))))
    {
        if axis.dot(axis) <= 1e-12 {
            continue;
        }
        let n = axis / axis.length();
        let extent = |basis: [V; 3], half: V| {
            basis[0].dot(n).abs() * half.0
                + basis[1].dot(n).abs() * half.1
                + basis[2].dot(n).abs() * half.2
        };
        let overlap = extent(aa, ha) + extent(bb, hb) - delta.dot(n).abs();
        if overlap < 0.0 {
            return 0.0;
        }
        depth = depth.min(overlap);
    }
    depth
}
pub fn run() {
    let mut world = fixture(4);
    let before = penetration(
        world.body(BodyId(148)).unwrap(),
        world.body(BodyId(156)).unwrap(),
    );
    assert!(
        before < 0.03,
        "reproducer must start with only a shallow contact: {before}"
    );
    let report = world.step(1.0 / 60.0).unwrap();
    let depth = penetration(
        world.body(BodyId(148)).unwrap(),
        world.body(BodyId(156)).unwrap(),
    );
    assert!(
        depth <= 1.8,
        "opposite face penetrated {depth}; unchanged dense screening bound is 1.8"
    );
    assert_eq!(report.retired, [BodyId(100002)]);
    assert!(report.swept_contacts > 0);
    assert!(report.impulse_iterations <= 32);
    assert!((world.elapsed_seconds() - 1.0 / 60.0).abs() < 1e-12);
    let expected = world.bodies().cloned().collect::<Vec<_>>();
    let mut observed = fixture(1);
    for substep in 1..=4 {
        observed.step(1.0 / 240.0).unwrap();
        let depth = penetration(
            observed.body(BodyId(148)).unwrap(),
            observed.body(BodyId(156)).unwrap(),
        );
        assert!(depth <= 1.8, "substep {substep} penetrated {depth}");
    }
    assert_eq!(observed.bodies().cloned().collect::<Vec<_>>(), expected);
}
