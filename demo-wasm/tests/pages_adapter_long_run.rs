use physics_engine_demo::{sandbox_reset, sandbox_shoot, sandbox_step};

const TICKS_PER_SECOND: usize = 60;
const TEN_SECONDS: usize = 10 * TICKS_PER_SECOND;

fn step(tick: usize, move_x: i32, move_z: i32, jump: bool) {
    let error = sandbox_step(move_x, move_z, i32::from(jump));
    assert_eq!(error, 0, "Pages sandbox failed at tick {tick} with error {error}");
}

#[test]
fn pages_adapter_survives_ten_seconds_idle() {
    sandbox_reset();
    for tick in 0..TEN_SECONDS {
        step(tick, 0, 0, false);
    }
}

#[test]
fn pages_adapter_survives_ten_seconds_held_forward() {
    sandbox_reset();
    for tick in 0..TEN_SECONDS {
        step(tick, 0, -7, false);
    }
}

#[test]
fn pages_adapter_survives_mixed_controls_and_projectiles() {
    sandbox_reset();
    let movement = [(0, -7), (5, -5), (7, 0), (5, 5), (0, 7), (-5, 5), (-7, 0), (-5, -5)];

    for tick in 0..TEN_SECONDS {
        if tick % 120 == 0 {
            let projectile = sandbox_shoot(0, 0, -96);
            assert!(projectile >= 0, "Pages sandbox rejected projectile at tick {tick}");
        }
        let (move_x, move_z) = movement[(tick / 30) % movement.len()];
        step(tick, move_x, move_z, tick % 90 == 0);
    }
}
