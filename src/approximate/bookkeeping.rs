//! Derived membership, contact adjacency and reusable traversal storage.
//! Body state and cached contact keys remain authoritative. No contact admission or solver math here.
use super::{Body, BodyId, CachedPoint, Scalar, Vector};
use std::collections::BTreeMap;

/// Source-level bookkeeping work. Capacity gauges exclude allocator overhead and contact geometry.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct BookkeepingStats {
    pub active_view_rebuilds: u64,
    pub active_body_scans: u64,
    pub adjacency_rebuilds: u64,
    pub adjacency_edges_indexed: u64,
    pub island_body_visits: u64,
    pub island_edge_visits: u64,
    /// Current-step directed support graphs built for contact-support closure.
    pub support_graph_builds: u64,
    /// Directed support edges indexed while building those graphs.
    pub support_edges_indexed: u64,
    /// Supported source bodies whose outgoing support edges were visited.
    pub support_body_visits: u64,
    /// Directed support edges visited by the propagation frontier.
    pub support_edge_visits: u64,
    pub bounds_updates: u64,
    pub bound_rows_sorted: u64,
    pub bounds_order_checks: u64,
    pub scratch_growths: u64,
    pub scratch_retained_bytes: u64,
}

pub(super) fn reserve<T>(v: &mut Vec<T>, needed: usize, work: &mut BookkeepingStats) {
    if v.capacity() < needed {
        v.reserve(needed - v.len());
        work.scratch_growths += 1;
    }
}
pub(super) fn push<T>(v: &mut Vec<T>, value: T, work: &mut BookkeepingStats) {
    let before = v.capacity();
    v.push(value);
    work.scratch_growths += u64::from(v.capacity() != before);
}
pub(super) fn bytes<T>(v: &Vec<T>) -> usize {
    v.capacity() * size_of::<T>()
}

#[derive(Clone, Debug, Default)]
pub(super) struct Activity {
    pub count: usize,
    pub indices: Vec<usize>,
    pub dirty: bool,
}
impl Activity {
    pub fn refresh(&mut self, bodies: &[Body], work: &mut BookkeepingStats) {
        if !self.dirty {
            return;
        }
        work.active_view_rebuilds += 1;
        work.active_body_scans += bodies.len() as u64;
        self.indices.clear();
        reserve(&mut self.indices, self.count, work);
        self.indices.extend(
            bodies
                .iter()
                .enumerate()
                .filter(|(_, b)| b.mass > 0.0 && !b.sleeping)
                .map(|(i, _)| i),
        );
        debug_assert_eq!(self.indices.len(), self.count);
        self.dirty = false;
    }
}

#[derive(Clone, Debug, Default)]
pub(super) struct Adjacency {
    pub dirty: bool,
    offsets: Vec<usize>,
    neighbors: Vec<usize>,
    cursor: Vec<usize>,
    edges: Vec<(usize, usize)>,
}
impl Adjacency {
    pub fn ensure(
        &mut self,
        bodies: &[Body],
        cache: &BTreeMap<(BodyId, BodyId), Vec<CachedPoint>>,
        work: &mut BookkeepingStats,
    ) {
        if !self.dirty && self.offsets.len() == bodies.len() + 1 {
            return;
        }
        work.adjacency_rebuilds += 1;
        work.adjacency_edges_indexed += cache.len() as u64;
        reserve(&mut self.offsets, bodies.len() + 1, work);
        self.offsets.clear();
        self.offsets.resize(bodies.len() + 1, 0);
        reserve(&mut self.edges, cache.len(), work);
        self.edges.clear();
        for &(a, b) in cache.keys() {
            let a = bodies
                .binary_search_by_key(&a, |b| b.id)
                .expect("live contact endpoint");
            let b = bodies
                .binary_search_by_key(&b, |b| b.id)
                .expect("live contact endpoint");
            self.offsets[a + 1] += 1;
            self.offsets[b + 1] += 1;
            self.edges.push((a, b));
        }
        for i in 1..self.offsets.len() {
            self.offsets[i] += self.offsets[i - 1];
        }
        reserve(&mut self.cursor, bodies.len(), work);
        self.cursor.clear();
        self.cursor.extend_from_slice(&self.offsets[..bodies.len()]);
        reserve(&mut self.neighbors, self.edges.len() * 2, work);
        self.neighbors.clear();
        self.neighbors.resize(self.edges.len() * 2, 0);
        // Canonical pair order produces the same per-body neighbor order as the original full scan.
        for &(a, b) in &self.edges {
            self.neighbors[self.cursor[a]] = b;
            self.cursor[a] += 1;
            self.neighbors[self.cursor[b]] = a;
            self.cursor[b] += 1;
        }
        self.dirty = false;
    }
    pub fn neighbors(&self, i: usize) -> &[usize] {
        &self.neighbors[self.offsets[i]..self.offsets[i + 1]]
    }
    fn retained_bytes(&self) -> usize {
        bytes(&self.offsets) + bytes(&self.neighbors) + bytes(&self.cursor) + bytes(&self.edges)
    }
}

#[derive(Clone, Debug, Default)]
pub(super) struct Traversal {
    seen: Vec<u64>,
    epoch: u64,
    stack: Vec<usize>,
    pub island: Vec<usize>,
}
impl Traversal {
    pub fn begin(&mut self, body_count: usize, work: &mut BookkeepingStats) {
        reserve(&mut self.seen, body_count, work);
        self.seen.resize(body_count, 0);
        self.epoch = self.epoch.wrapping_add(1);
        if self.epoch == 0 {
            self.seen.fill(0);
            self.epoch = 1;
        }
    }
    pub fn collect(
        &mut self,
        root: usize,
        bodies: &[Body],
        graph: &Adjacency,
        work: &mut BookkeepingStats,
    ) {
        self.stack.clear();
        self.island.clear();
        push(&mut self.stack, root, work);
        while let Some(i) = self.stack.pop() {
            if self.seen[i] == self.epoch {
                continue;
            }
            self.seen[i] = self.epoch;
            if !bodies[i].movable() {
                continue;
            }
            work.island_body_visits += 1;
            push(&mut self.island, i, work);
            let neighbors = graph.neighbors(i);
            work.island_edge_visits += neighbors.len() as u64;
            let needed = self.stack.len() + neighbors.len();
            reserve(&mut self.stack, needed, work);
            self.stack.extend_from_slice(neighbors);
        }
    }
    fn retained_bytes(&self) -> usize {
        bytes(&self.seen) + bytes(&self.stack) + bytes(&self.island)
    }
}

/// Reusable directed current-step support graph.
///
/// Building the CSR view is O(bodies + support edges). Propagation then visits every outgoing edge
/// of a reachable supported source at most once, replacing the previous repeated full-edge scan.
#[derive(Clone, Debug, Default)]
pub(super) struct SupportPropagation {
    offsets: Vec<usize>,
    neighbors: Vec<usize>,
    cursor: Vec<usize>,
    queue: Vec<usize>,
}
impl SupportPropagation {
    pub fn propagate(
        &mut self,
        supported: &mut [bool],
        edges: &[(usize, usize)],
        work: &mut BookkeepingStats,
    ) {
        if edges.is_empty() {
            return;
        }

        work.support_graph_builds += 1;
        work.support_edges_indexed += edges.len() as u64;

        reserve(&mut self.offsets, supported.len() + 1, work);
        self.offsets.clear();
        self.offsets.resize(supported.len() + 1, 0);
        for &(lower, upper) in edges {
            debug_assert!(lower < supported.len());
            debug_assert!(upper < supported.len());
            self.offsets[lower + 1] += 1;
        }
        for index in 1..self.offsets.len() {
            self.offsets[index] += self.offsets[index - 1];
        }

        reserve(&mut self.cursor, supported.len(), work);
        self.cursor.clear();
        self.cursor
            .extend_from_slice(&self.offsets[..supported.len()]);

        reserve(&mut self.neighbors, edges.len(), work);
        self.neighbors.clear();
        self.neighbors.resize(edges.len(), 0);
        for &(lower, upper) in edges {
            self.neighbors[self.cursor[lower]] = upper;
            self.cursor[lower] += 1;
        }

        self.queue.clear();
        reserve(
            &mut self.queue,
            supported.len().min(edges.len()),
            work,
        );
        for (index, is_supported) in supported.iter().copied().enumerate() {
            if is_supported && self.offsets[index] != self.offsets[index + 1] {
                push(&mut self.queue, index, work);
            }
        }

        let mut head = 0;
        while head < self.queue.len() {
            let lower = self.queue[head];
            head += 1;
            work.support_body_visits += 1;

            let start = self.offsets[lower];
            let end = self.offsets[lower + 1];
            work.support_edge_visits += (end - start) as u64;
            for edge_index in start..end {
                let upper = self.neighbors[edge_index];
                if !supported[upper] {
                    supported[upper] = true;
                    if self.offsets[upper] != self.offsets[upper + 1] {
                        push(&mut self.queue, upper, work);
                    }
                }
            }
        }
    }

    fn retained_bytes(&self) -> usize {
        bytes(&self.offsets)
            + bytes(&self.neighbors)
            + bytes(&self.cursor)
            + bytes(&self.queue)
    }
}

#[derive(Clone, Copy, Debug)]
pub(super) struct SweepRow {
    pub index: usize,
    pub lo: Vector,
    pub hi: Vector,
    pub active: bool,
}
#[derive(Clone, Debug, Default)]
pub(super) struct Scratch {
    pub position: super::position::Scratch,
    pub activity: Activity,
    pub graph: Adjacency,
    pub traversal: Traversal,
    pub work: BookkeepingStats,
    pub bounds: Vec<SweepRow>,
    pub roots: Vec<BodyId>,
    pub used: Vec<usize>,
    pub retired: Vec<BodyId>,
    pub support: SupportPropagation,
    pub supported: Vec<bool>,
    pub support_edges: Vec<(usize, usize)>,
    pub earliest: Vec<Scalar>,
}
impl Scratch {
    pub fn layout_changed(&mut self) {
        self.position.invalidate();
        self.activity.dirty = true;
        self.graph.dirty = true;
        self.bounds.clear();
    }
    pub fn ensure_graph(
        &mut self,
        bodies: &[Body],
        cache: &BTreeMap<(BodyId, BodyId), Vec<CachedPoint>>,
    ) {
        self.graph.ensure(bodies, cache, &mut self.work);
    }
    pub fn retained_bytes(&self) -> usize {
        bytes(&self.activity.indices)
            + self.graph.retained_bytes()
            + self.traversal.retained_bytes()
            + bytes(&self.bounds)
            + bytes(&self.roots)
            + bytes(&self.used)
            + bytes(&self.retired)
            + self.support.retained_bytes()
            + bytes(&self.supported)
            + bytes(&self.support_edges)
            + bytes(&self.earliest)
    }
}

#[cfg(test)]
mod tests;
