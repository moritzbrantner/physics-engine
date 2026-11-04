//! WASM acceptance for checkpoint continuation of the MMORPG-style zone (#452): the same zone
//! and commands as `tests/checkpoint_zone.rs`, run uninterrupted and through capture, drop and
//! restore at every capture tick, on both body storages. Panics (traps in WASM) on any
//! difference. `scripts/checkpoint-wasm.sh` builds it for wasm32-unknown-unknown and runs it
//! in Node; natively, `cargo run --example checkpoint_zone --features checkpoints`.
#[path = "../tests/support/checkpoint_zone.rs"]
mod zone;

use physics_engine::approximate::{
    BodyStorage, Checkpoint, CheckpointContext, CheckpointLimits, World,
};

const CONTEXT: CheckpointContext = CheckpointContext {
    build: [41; 32],
    content: [43; 32],
};

fn run(storage: BodyStorage, captures: &[u32]) -> Vec<(String, Vec<u8>)> {
    let (mut w, mut game) = zone::zone(storage);
    let mut out = Vec::new();
    let mut seen = Vec::new();
    for _ in 0..zone::TICKS {
        if captures.contains(&game.tick) {
            let saved = w.checkpoint(CONTEXT).unwrap().to_bytes();
            drop(w);
            w = Checkpoint::from_bytes(&saved, CONTEXT, CheckpointLimits::default())
                .unwrap()
                .restore(&[zone::terrain()])
                .unwrap();
        }
        let observed = zone::tick(&mut w, &mut game, &mut out);
        seen.push((observed, checkpoint(&w)));
    }
    seen
}

fn checkpoint(w: &World) -> Vec<u8> {
    w.checkpoint(CONTEXT).unwrap().to_bytes()
}

fn main() {
    for storage in [BodyStorage::Ordered, BodyStorage::Indexed] {
        let expected = run(storage, &[]);
        let actual = run(storage, &zone::CAPTURES);
        assert!(actual == expected, "{storage:?}: continuation differs");
        assert_eq!(
            expected
                .iter()
                .filter(|(o, _)| o.contains("failed"))
                .count(),
            2
        );
    }
}
