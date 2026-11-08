//! Derived membership, contact adjacency and reusable traversal storage.
//! Body state and cached contact keys remain authoritative. No contact admission or solver math here.
use super::{Body, CachedPoint, Order, Real, Vector};
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
    /// Awake bodies' swept bounds refitted into the query tree for contact pairs.
    pub bounds_updates: u64,
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

/// Awake bodies (`mass > 0`, not sleeping) in storage order. Wakes and sleeps update the list
/// in place; only a layout change (`dirty`) rescans the bodies.
#[derive(Clone, Debug, Default)]
pub(super) struct Activity {
    pub count: usize,
    /// Bodies with mass, awake or not. Where `count` reaches it no body sleeps, and a wake finds
    /// nothing.
    pub dynamic: usize,
    pub indices: Vec<usize>,
    pub dirty: bool,
}
impl Activity {
    /// Body `i` just woke.
    pub fn woke(&mut self, i: usize, work: &mut BookkeepingStats) {
        if self.dirty {
            return;
        }
        let at = self.indices.partition_point(|&k| k < i);
        debug_assert!(self.indices.get(at) != Some(&i));
        let needed = self.indices.len() + 1;
        reserve(&mut self.indices, needed, work);
        self.indices.insert(at, i);
    }
    /// Awake body `i` just left storage; no other body moved.
    pub fn left(&mut self, i: usize) {
        if self.dirty {
            return;
        }
        let at = self.indices.partition_point(|&k| k < i);
        debug_assert_eq!(self.indices.get(at), Some(&i));
        self.indices.remove(at);
    }
    /// Whether some body with mass sleeps. Conservative for kinematic bodies with mass.
    pub fn any_sleeper(&self) -> bool {
        self.dynamic != self.count
    }
    /// Bodies of the list just fell asleep.
    pub fn slept(&mut self, bodies: &[Body]) {
        if !self.dirty {
            self.indices.retain(|&i| !bodies[i].sleeping);
        }
    }
    pub fn refresh(&mut self, bodies: &[Body], work: &mut BookkeepingStats) {
        if !self.dirty {
            debug_assert_eq!(self.indices.len(), self.count);
            return;
        }
        work.active_view_rebuilds += 1;
        work.active_body_scans += bodies.len() as u64;
        crate::work::add(crate::work::Counter::BodyVisits, bodies.len() as u64);
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

/// One body's run in `Adjacency::neighbors`, valid while `epoch` is the graph's.
#[derive(Clone, Copy, Debug, Default)]
struct Run {
    epoch: u64,
    start: u32,
    end: u32,
}

/// Contact and joint neighbours per storage position. A rebuild visits the edges only: runs are
/// stamped with an epoch, so a body without an edge costs nothing (#523).
#[derive(Clone, Debug)]
pub(super) struct Adjacency {
    pub dirty: bool,
    epoch: u64,
    runs: Vec<Run>,
    /// Positions with an edge in this epoch, in the order first seen.
    nodes: Vec<usize>,
    neighbors: Vec<usize>,
    edges: Vec<(usize, usize)>,
}
impl Default for Adjacency {
    /// Dirty until first built: the cache may already hold pairs (a restored checkpoint).
    fn default() -> Self {
        Self {
            dirty: true,
            epoch: 0,
            runs: Vec::new(),
            nodes: Vec::new(),
            neighbors: Vec::new(),
            edges: Vec::new(),
        }
    }
}
impl Adjacency {
    /// O(edges · log bodies), independent of bodies without an edge.
    pub fn ensure(
        &mut self,
        bodies: &[Body],
        cached: impl Iterator<Item = (Order, Order)>,
        cached_len: usize,
        joints: impl ExactSizeIterator<Item = (Order, Order)>,
        work: &mut BookkeepingStats,
    ) {
        if !self.dirty {
            return;
        }
        work.adjacency_rebuilds += 1;
        let edges = cached_len + joints.len();
        work.adjacency_edges_indexed += edges as u64;
        self.epoch = self.epoch.wrapping_add(1);
        if self.epoch == 0 {
            self.runs.fill(Run::default());
            self.epoch = 1;
        }
        // Grown only: positions past the end hold stale epochs, never read as current.
        if self.runs.len() < bodies.len() {
            reserve(&mut self.runs, bodies.len(), work);
            self.runs.resize(bodies.len(), Run::default());
        }
        reserve(&mut self.edges, edges, work);
        self.edges.clear();
        self.nodes.clear();
        let epoch = self.epoch;
        let (runs, nodes) = (&mut self.runs, &mut self.nodes);
        let mut count = |i: usize, work: &mut BookkeepingStats| {
            let run = &mut runs[i];
            if run.epoch != epoch {
                *run = Run {
                    epoch,
                    start: 0,
                    end: 0,
                };
                push(nodes, i, work);
            }
            run.end += 1;
        };
        // Contact pairs in pair order, then joined pairs in joint order (#453).
        for (a, b) in cached.chain(joints) {
            let a = bodies
                .binary_search_by_key(&a, |b| b.order)
                .expect("live contact endpoint");
            let b = bodies
                .binary_search_by_key(&b, |b| b.order)
                .expect("live contact endpoint");
            count(a, work);
            count(b, work);
            self.edges.push((a, b));
        }
        // `end` held the count; it becomes the fill cursor, and the run's end once filled.
        let mut next = 0;
        for &i in &self.nodes {
            let run = &mut self.runs[i];
            let len = run.end;
            run.start = next;
            run.end = next;
            next += len;
        }
        reserve(&mut self.neighbors, self.edges.len() * 2, work);
        self.neighbors.clear();
        self.neighbors.resize(self.edges.len() * 2, 0);
        // Canonical pair order produces the same per-body neighbor order as the original full scan.
        for &(a, b) in &self.edges {
            self.neighbors[self.runs[a].end as usize] = b;
            self.runs[a].end += 1;
            self.neighbors[self.runs[b].end as usize] = a;
            self.runs[b].end += 1;
        }
        self.dirty = false;
    }
    pub fn neighbors(&self, i: usize) -> &[usize] {
        match self.runs.get(i) {
            Some(run) if run.epoch == self.epoch => {
                &self.neighbors[run.start as usize..run.end as usize]
            }
            _ => &[],
        }
    }
    fn retained_bytes(&self) -> usize {
        bytes(&self.runs) + bytes(&self.nodes) + bytes(&self.neighbors) + bytes(&self.edges)
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
        self.walk(root, bodies, work, |i, stack, work| {
            let neighbors = graph.neighbors(i);
            work.island_edge_visits += neighbors.len() as u64;
            let needed = stack.len() + neighbors.len();
            reserve(stack, needed, work);
            stack.extend_from_slice(neighbors);
        });
    }
    /// Collects the movable bodies `root` reaches through `expand`, which pushes a visited
    /// body's neighbors. Fixed and external bodies end a path.
    pub fn walk(
        &mut self,
        root: usize,
        bodies: &[Body],
        work: &mut BookkeepingStats,
        mut expand: impl FnMut(usize, &mut Vec<usize>, &mut BookkeepingStats),
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
            expand(i, &mut self.stack, work);
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
    /// The support edges sorted by lower body: each body's outgoing edges are one run.
    edges: Vec<(usize, usize)>,
    queue: Vec<usize>,
}
impl SupportPropagation {
    /// O(edges · log edges), independent of the body count.
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

        self.edges.clear();
        reserve(&mut self.edges, edges.len(), work);
        self.edges.extend_from_slice(edges);
        self.edges.sort_unstable();
        let sorted = &self.edges;
        let outgoing = |lower: usize| {
            let start = sorted.partition_point(|&(l, _)| l < lower);
            let end = start + sorted[start..].partition_point(|&(l, _)| l == lower);
            start..end
        };

        self.queue.clear();
        reserve(&mut self.queue, edges.len(), work);
        for run in sorted.chunk_by(|x, y| x.0 == y.0) {
            debug_assert!(run[0].0 < supported.len());
            if supported[run[0].0] {
                push(&mut self.queue, run[0].0, work);
            }
        }

        let mut head = 0;
        while head < self.queue.len() {
            let lower = self.queue[head];
            head += 1;
            work.support_body_visits += 1;

            let range = outgoing(lower);
            work.support_edge_visits += range.len() as u64;
            for &(_, upper) in &sorted[range] {
                debug_assert!(upper < supported.len());
                if !supported[upper] {
                    supported[upper] = true;
                    if !outgoing(upper).is_empty() {
                        push(&mut self.queue, upper, work);
                    }
                }
            }
        }
    }

    fn retained_bytes(&self) -> usize {
        bytes(&self.edges) + bytes(&self.queue)
    }
}

/// An awake body's broad-phase box (#412), Rapier's fat AABB: it covers the body's swept bounds
/// with room to move, its query-tree leaf holds it, and `broad_slots[start..start + len]` are
/// the tree leaves it overlapped when it was made. It lasts across steps until the swept bounds
/// leave it, the body sleeps or a resting body it overlaps changes.
#[derive(Clone, Copy, Debug, Default)]
pub(super) struct BroadBox {
    /// Valid while equal to `Scratch::broad_epoch`; zero is no box.
    pub epoch: u64,
    /// When the box was made and queried the tree (`Scratch::queries`).
    pub query: u64,
    pub lo: Vector,
    pub hi: Vector,
    pub start: u32,
    pub len: u32,
    /// The body's row in `Scratch::bounds` this contact generation.
    pub row: u32,
}

#[derive(Clone, Debug, Default)]
pub(super) struct Scratch {
    /// Emptied contact-point buffers of removed pairs, reused by new ones: transient bodies
    /// make and drop their pairs every tick. Never longer than the peak of cached pairs.
    pub point_pool: Vec<Vec<CachedPoint>>,
    /// The new pairs of one contact-cache walk, inserted after it.
    pub fresh_pairs: Vec<((Order, Order), Vec<CachedPoint>)>,
    pub position: super::position::Scratch,
    pub activity: Activity,
    pub graph: Adjacency,
    pub traversal: Traversal,
    pub work: BookkeepingStats,
    /// Awake bodies' swept bounds this contact generation: (index, lo, hi).
    pub bounds: Vec<(usize, Vector, Vector)>,
    /// Broad-phase boxes, indexed by id slot (as query-tree leaves).
    pub broad: Vec<BroadBox>,
    /// Query-tree leaf slots the broad-phase boxes overlapped.
    pub broad_slots: Vec<u32>,
    /// Compaction buffer for `broad_slots`.
    pub broad_spare: Vec<u32>,
    /// Bumping it drops every broad-phase box (`BroadBox::epoch`).
    pub broad_epoch: u64,
    /// False while no box of the current epoch can exist (none made since the last
    /// `forget_boxes`): a resting body's change then skips the tree walk (#419).
    pub boxes_live: bool,
    /// Counts broad-phase queries, for `BroadBox::query`.
    pub queries: u64,
    /// Broadphase pairs (lower, higher body order) before the narrow phase.
    pub candidates: Vec<(usize, usize)>,
    /// Pairs the next substep retests alone while reusing every other row (#543).
    pub retest: Vec<(usize, usize)>,
    /// The pairs being retested this substep, in pair order.
    pub retesting: Vec<(usize, usize)>,
    /// Bodies whose fixed-boundary sweep also retests their halfspace pairs.
    pub retest_bodies: Vec<usize>,
    /// Bodies awake at some point of the current step (step start plus wakes), for the
    /// end-of-step force reset and query-tree refit.
    pub stepped: Vec<usize>,
    /// The [`Self::layout`] for which the per-body vectors (`responses`, `supported`,
    /// `earliest`) hold their resting values for every body that is not awake: default
    /// responses, `true` and infinity. `None` after a rollback.
    pub per_body: Option<u64>,
    pub roots: Vec<Order>,
    /// Pairs that held a contact in this step, sorted (`Config::warm_start` off only): that
    /// mode's restitution gate, which reads no history from earlier steps.
    pub step_pairs: Vec<(Order, Order)>,
    pub used: Vec<usize>,
    pub retired: Vec<Order>,
    pub support: SupportPropagation,
    pub supported: Vec<bool>,
    pub support_edges: Vec<(usize, usize)>,
    pub earliest: Vec<Real>,
    /// Entries of `earliest` set by the current contact pass.
    pub earliest_set: Vec<usize>,
    /// CCD bodies with an admitted swept impact this substep (remaining-time advancement).
    pub ccd: Vec<usize>,
    /// Per-body position correction from remaining-time advancement, added at integration.
    pub ccd_offsets: Vec<Vector>,
    /// Substep fraction at which a body was last struck by an earlier-advanced CCD body.
    pub ccd_start: Vec<Real>,
    pub ccd_excluded: Vec<usize>,
    /// Retirements admitted by remaining-time impacts, merged after the contact solve.
    pub ccd_retired: Vec<Order>,
    /// Incremented by every layout change; the query index records the one it was built for.
    pub generation: u64,
    /// Incremented when live bodies' storage positions move (not when one is appended, or
    /// removed without moving the others): what position-indexed scratch is valid for.
    pub layout: u64,
}
impl Scratch {
    /// The epoch of valid broad-phase boxes (`BroadBox::epoch`), never zero.
    pub fn box_epoch(&self) -> u64 {
        self.broad_epoch + 1
    }
    /// Drops every broad-phase box.
    pub fn forget_boxes(&mut self) {
        self.broad_epoch += 1;
        self.boxes_live = false;
    }
    /// Drops the broad-phase box of the body in `slot`.
    pub fn drop_box(&mut self, slot: u32) {
        if let Some(e) = self.broad.get_mut(slot as usize) {
            e.epoch = 0;
        }
    }
    /// Storage positions moved: all position-indexed scratch is stale.
    pub fn layout_changed(&mut self) {
        self.generation += 1;
        self.layout += 1;
        self.activity.dirty = true;
        self.graph.dirty = true;
        self.poses_changed();
    }
    /// Keeps a removed pair's point buffer for the next new pair.
    pub fn recycle_points(&mut self, mut points: Vec<CachedPoint>) {
        points.clear();
        push(&mut self.point_pool, points, &mut self.work);
    }
    /// A body moved outside a step: derived pose caches are stale, the layout, the awake set
    /// and the contact graph are not.
    pub fn poses_changed(&mut self) {
        self.position.invalidate();
    }
    pub fn ensure_graph(
        &mut self,
        bodies: &[Body],
        cache: &BTreeMap<(Order, Order), Vec<CachedPoint>>,
        parked: &super::parked::ParkedContacts,
        joints: impl ExactSizeIterator<Item = (Order, Order)>,
    ) {
        self.graph.ensure(
            bodies,
            cache.keys().chain(parked.keys()).copied(),
            cache.len() + parked.len(),
            joints,
            &mut self.work,
        );
    }
    /// The graph of joints alone, for a world whose contact reach comes from geometry.
    pub fn ensure_joint_graph(
        &mut self,
        bodies: &[Body],
        joints: impl ExactSizeIterator<Item = (Order, Order)>,
    ) {
        self.graph
            .ensure(bodies, std::iter::empty(), 0, joints, &mut self.work);
    }
    /// Excludes the broad-phase boxes, which persist with the query tree (its storage is not
    /// counted either).
    pub fn retained_bytes(&self) -> usize {
        bytes(&self.point_pool)
            + bytes(&self.fresh_pairs)
            + bytes(&self.activity.indices)
            + self.graph.retained_bytes()
            + self.traversal.retained_bytes()
            + bytes(&self.bounds)
            + bytes(&self.candidates)
            + bytes(&self.retest)
            + bytes(&self.retesting)
            + bytes(&self.retest_bodies)
            + bytes(&self.stepped)
            + bytes(&self.roots)
            + bytes(&self.step_pairs)
            + bytes(&self.used)
            + bytes(&self.retired)
            + self.support.retained_bytes()
            + bytes(&self.supported)
            + bytes(&self.support_edges)
            + bytes(&self.earliest)
            + bytes(&self.earliest_set)
            + bytes(&self.ccd)
            + bytes(&self.ccd_offsets)
            + bytes(&self.ccd_start)
            + bytes(&self.ccd_excluded)
            + bytes(&self.ccd_retired)
    }
}

#[cfg(test)]
mod tests;
