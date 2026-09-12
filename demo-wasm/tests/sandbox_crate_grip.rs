use physics_engine_demo::{
    sandbox_body_count, sandbox_body_role, sandbox_body_x, sandbox_body_y, sandbox_body_z,
    sandbox_reset, sandbox_step,
};

fn crate_positions() -> Vec<(i32, i32, i32)> {
    let mut positions = (0..sandbox_body_count())
        .filter(|index| sandbox_body_role(*index) == 2)
        .map(|index| {
            (
                sandbox_body_x(index),
                sandbox_body_y(index),
                sandbox_body_z(index),
            )
        })
        .collect::<Vec<_>>();
    positions.sort_unstable();
    positions
}

#[test]
fn pushed_crates_come_to_a_visible_rest() {
    sandbox_reset();

    for _ in 0..16 {
        assert_eq!(sandbox_step(0, 0, 0), 0);
    }
    let before_push = crate_positions();

    for _ in 0..40 {
        assert_eq!(sandbox_step(0, -7, 0), 0);
    }
    let after_push = crate_positions();
    assert_ne!(after_push, before_push, "player never moved a crate");

    for _ in 0..180 {
        assert_eq!(sandbox_step(0, 0, 0), 0);
    }
    let settled = crate_positions();

    for _ in 0..60 {
        assert_eq!(sandbox_step(0, 0, 0), 0);
    }
    assert_eq!(
        crate_positions(),
        settled,
        "a crate kept visibly drifting after the settling window"
    );
}
