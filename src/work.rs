//! Deterministic work counters for the complexity contracts (`.performance/contract.json`).
//! Compiled only with the test-only `work-counters` feature; every call site is a no-op
//! otherwise.

/// Counted operations.
#[derive(Clone, Copy, Debug)]
#[repr(usize)]
pub enum Counter {
    /// Query-tree nodes visited (ray and box traversals).
    TreeNodes,
    /// Bodies tested exactly by a query after tree pruning.
    ExactTests,
    /// Body records moved in storage by inserts and removals.
    StorageShifts,
    /// Leaves inserted into the query tree, including rebuilds.
    TreeInserts,
    /// Heightfield cells visited.
    TerrainCells,
    /// Heightfield triangles tested.
    TerrainTriangles,
}

const COUNTERS: usize = 6;

#[cfg(feature = "work-counters")]
std::thread_local! {
    static WORK: std::cell::Cell<[u64; COUNTERS]> = const { std::cell::Cell::new([0; COUNTERS]) };
}

#[inline(always)]
pub(crate) fn add(counter: Counter, amount: u64) {
    #[cfg(feature = "work-counters")]
    WORK.with(|w| {
        let mut v = w.get();
        v[counter as usize] += amount;
        w.set(v);
    });
    #[cfg(not(feature = "work-counters"))]
    let _ = (counter, amount);
}

/// Counter values since the last [`reset`] on this thread (zero without `work-counters`).
pub fn read() -> [u64; COUNTERS] {
    #[cfg(feature = "work-counters")]
    return WORK.with(std::cell::Cell::get);
    #[cfg(not(feature = "work-counters"))]
    [0; COUNTERS]
}

/// Value of one counter since the last [`reset`].
pub fn get(counter: Counter) -> u64 {
    read()[counter as usize]
}

pub fn reset() {
    #[cfg(feature = "work-counters")]
    WORK.with(|w| w.set([0; COUNTERS]));
}
