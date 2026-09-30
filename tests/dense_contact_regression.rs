#[path = "support/dense_contact.rs"]
mod dense;
#[test]
fn glancing_projectile_keeps_the_opposite_contact_face_inside_the_dense_overlap_limit() {
    dense::run();
}

#[test]
fn failed_dynamic_correction_rolls_back_both_endpoints_and_contact_history() {
    use physics_engine::{
        BodyId,
        approximate::{
            Body, CheckpointContext, Config, Error, PositionCorrection, Shape, Vector as V, World,
        },
    };
    let mut world = World::new(Config {
        gravity: V::ZERO,
        substeps: 1,
        fixed_position_iterations: 2,
        position_correction: PositionCorrection::AdmittedContacts,
        ..Default::default()
    })
    .unwrap();
    for (id, half, x, mass) in [
        (1, V(10.0, 20.0, 20.0), 1e12 - 8.0, 3.0),
        (2, V(2.0, 10.0, 10.0), 1e12 - 1.0, 1.0),
    ] {
        let mut body = Body::new(BodyId(id), Shape::Box(half), V(x, 0.0, 0.0), mass);
        body.rotation_locked = true;
        world.add_body(body).unwrap();
    }
    let context = CheckpointContext {
        build: [193; 32],
        content: [4; 32],
    };
    let before = world.checkpoint(context).unwrap().to_bytes();
    assert_eq!(
        world.step(1.0 / 240.0).unwrap_err(),
        Error::NonFiniteState(BodyId(2))
    );
    assert_eq!(world.checkpoint(context).unwrap().to_bytes(), before);
    let work = world.last_failed_step_work().unwrap();
    assert!(work.position_geometry.current_queries > 0);
    assert!(work.transaction.rolled_back);
    assert_eq!(world.elapsed_seconds(), 0.0);
    world.remove_body(BodyId(1));
    world.step(1.0 / 240.0).unwrap();
}
