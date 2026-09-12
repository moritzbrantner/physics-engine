use physics_engine_demo::{sandbox_reset, sandbox_shoot, sandbox_step};

const SMOKE_TICKS: usize = 180;

fn step(tick: usize, move_x: i32, move_z: i32, jump: bool) {
    let error = sandbox_step(move_x, move_z, i32::from(jump));
    assert_eq!(
        error, 0,
        "Pages sandbox failed at tick {tick} with error {error}"
    );
}

#[test]
fn pages_adapter_survives_three_seconds_idle() {
    sandbox_reset();
    for tick in 0..SMOKE_TICKS {
        step(tick, 0, 0, false);
    }
}

#[test]
fn pages_adapter_replays_reported_mixed_input_path() {
    sandbox_reset();
    let changes = [
        (0, (-5, 4)),
        (15, (-5, 5)),
        (30, (-2, -4)),
        (45, (5, -3)),
        (60, (0, 0)),
        (75, (6, -7)),
        (90, (-1, -6)),
        (105, (-1, 7)),
        (120, (-6, -5)),
    ];
    let mut movement = (0, 0);

    for tick in 0..=128 {
        if let Some((_, next)) = changes.iter().find(|(at, _)| *at == tick) {
            movement = *next;
        }
        if tick == 125 {
            let projectile = sandbox_shoot(84, -3, -36);
            assert!(projectile >= 0, "Pages sandbox rejected the projectile");
        }
        step(tick, movement.0, movement.1, tick == 71);
    }
}
