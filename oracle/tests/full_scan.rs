//! Every oracle scenario's Physics Engine side, on both body storages, from world setup to its
//! last tick, takes no O(n) stale-tree fallback scan (#414): the scenarios are the consumers'
//! call patterns, and the work contract's `full-scan-bodies` budget of 0 holds there too.
use physics_engine::approximate::BodyStorage;
use physics_engine::work::{self, Counter};
use physics_oracle::Trace;

#[test]
fn oracle_scenarios_never_take_the_stale_tree_fallback() {
    let mut nodes = 0;
    for scenario in physics_oracle::all() {
        for storage in [BodyStorage::Ordered, BodyStorage::Indexed] {
            work::reset();
            let mut sim = (scenario.engine)(storage);
            scenario.run(&mut *sim, &mut Trace::default());
            assert_eq!(
                work::get(Counter::FullScanBodies),
                0,
                "{} on {storage:?}",
                scenario.name
            );
            nodes += work::get(Counter::TreeNodes);
        }
    }
    // The counters are live: the scenarios' queries descended the tree.
    assert!(nodes > 0);
}
