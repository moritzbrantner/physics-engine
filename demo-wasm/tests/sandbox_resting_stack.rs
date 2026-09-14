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
fn untouched_initial_crates_do_not_drift_or_topple() {
    sandbox_reset();
    let initial = crate_positions();

    for tick in 0..240 {
        assert_eq!(
            sandbox_step(0, 0, 0),
            0,
            "sandbox failed while reproducing untouched resting-stack drift at tick {tick}"
        );
    }

    assert_eq!(
        crate_positions(),
        initial,
        "the untouched sandbox changed crate poses under gravity alone"
    );
}
