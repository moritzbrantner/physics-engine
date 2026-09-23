//! Velocity-solver partitions derived from THIS substep's admitted response constraints.
//! Not the persistent warm-start/sleep graph: new swept contacts participate immediately and
//! removed contacts cannot leave a stale bridge. Only physically immutable velocity anchors are
//! excluded. A one-way row still READS both mutable velocities and therefore connects them.
use super::convergence::{Exit, Outcome};
use super::{Body, Constraint, Convergence, PreparedResponse, Report, convergence};

/// Which set of constraints must converge together? No setting changes the timestep or tolerances.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ConvergenceScope {
    /// Original reference: any difficult contact prevents all other contacts from finishing early.
    WholeWorld,
    /// Independent groups stop after their own complete-pass residual certificate.
    #[default]
    ContactIslands,
}

/// Actual grouping and solving work. Island sums are distinct from old world-substep counters.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct IslandStats {
    pub partition_builds: u64,
    pub rows_indexed: u64,
    pub endpoints_checked: u64,
    pub dynamic_nodes: u64,
    pub union_attempts: u64,
    pub root_links_followed: u64,
    pub islands: u64,
    pub max_island_rows: u64,
    pub single_island_substeps: u64,
    /// Sum of passes over individual islands, not a count of chronological substeps.
    pub island_iterations: u64,
    pub converged_islands: u64,
    pub capped_islands: u64,
    /// Counterfactual visits at the SAME maximum budget, minus actual row visits.
    pub skipped_constraint_visits: u64,
    pub scratch_growths: u64,
    /// Vec payload capacity only, excluding allocator/inline World/report overhead.
    pub scratch_retained_bytes: u64,
    pub epoch_resets: u64,
    /// Shared first two passes, allowing wholly easy substeps to avoid partitioning at all.
    pub shared_prefix_iterations: u64,
    pub prefix_converged_substeps: u64,
}

#[derive(Clone, Copy, Debug, Default)]
struct Node {
    epoch: u64,
    parent: usize,
    size: usize,
    first: usize,
    rows: usize,
    cursor: usize,
}

/// Reusable union-find plus a stable CSR row view; constraints themselves are never reordered.
#[derive(Clone, Debug, Default)]
pub(super) struct Scratch {
    epoch: u64,
    nodes: Vec<Node>,
    roots: Vec<usize>,
    offsets: Vec<usize>,
    rows: Vec<usize>,
}

fn reserve<T>(values: &mut Vec<T>, needed: usize, stats: &mut IslandStats) {
    if values.capacity() < needed {
        values.reserve(needed - values.len());
        stats.scratch_growths += 1;
    }
}
fn mutable_velocity(b: &Body) -> bool {
    b.movable() && !b.sleeping
}

impl Scratch {
    pub fn retained_bytes(&self) -> u64 {
        (self.nodes.capacity() * size_of::<Node>()
            + (self.roots.capacity() + self.offsets.capacity() + self.rows.capacity())
                * size_of::<usize>()) as u64
    }
    fn touch(&mut self, i: usize, stats: &mut IslandStats) {
        if self.nodes[i].epoch != self.epoch {
            self.nodes[i] = Node {
                epoch: self.epoch,
                parent: i,
                size: 1,
                first: i,
                rows: 0,
                cursor: 0,
            };
            stats.dynamic_nodes += 1;
        }
    }
    fn root(&mut self, mut i: usize, stats: &mut IslandStats) -> usize {
        // Path halving with size-weighted union; no recursive traversal or depth-dependent stack.
        while self.nodes[i].parent != i {
            stats.root_links_followed += 1;
            let parent = self.nodes[i].parent;
            self.nodes[i].parent = self.nodes[parent].parent;
            i = self.nodes[i].parent;
        }
        i
    }
    fn union(&mut self, a: usize, b: usize, stats: &mut IslandStats) {
        stats.union_attempts += 1;
        let (mut a, mut b) = (self.root(a, stats), self.root(b, stats));
        if a == b {
            return;
        }
        if self.nodes[a].size < self.nodes[b].size
            || (self.nodes[a].size == self.nodes[b].size
                && self.nodes[a].first > self.nodes[b].first)
        {
            std::mem::swap(&mut a, &mut b);
        }
        self.nodes[b].parent = a;
        self.nodes[a].size += self.nodes[b].size;
        self.nodes[a].first = self.nodes[a].first.min(self.nodes[b].first);
    }
    fn row_root(&mut self, c: &Constraint, bodies: &[Body], stats: &mut IslandStats) -> usize {
        let i = if mutable_velocity(&bodies[c.a]) {
            c.a
        } else {
            c.b
        };
        // Positive effective-mass rows always contain at least one movable, awake endpoint.
        debug_assert!(mutable_velocity(&bodies[i]));
        self.root(i, stats)
    }
    fn prepare(&mut self, bodies: &[Body], constraints: &[Constraint], stats: &mut IslandStats) {
        stats.partition_builds += 1;
        stats.rows_indexed += constraints.len() as u64;
        self.epoch = self.epoch.wrapping_add(1);
        if self.epoch == 0 {
            for node in &mut self.nodes {
                node.epoch = 0;
            }
            self.epoch = 1;
            stats.epoch_resets += 1;
        }
        reserve(&mut self.nodes, bodies.len(), stats);
        self.nodes.resize(bodies.len(), Node::default());
        self.roots.clear();
        self.offsets.clear();
        self.rows.clear();
        // Fresh constraints include current, speculative and admitted swept contacts, after waking.
        for c in constraints {
            stats.endpoints_checked += 2;
            let (a, b) = (
                mutable_velocity(&bodies[c.a]),
                mutable_velocity(&bodies[c.b]),
            );
            if a {
                self.touch(c.a, stats);
            }
            if b {
                self.touch(c.b, stats);
            }
            if a && b {
                self.union(c.a, c.b, stats);
            }
        }
        reserve(&mut self.roots, constraints.len().min(bodies.len()), stats);
        for c in constraints {
            let root = self.row_root(c, bodies, stats);
            if self.nodes[root].rows == 0 {
                self.roots.push(root);
            }
            self.nodes[root].rows += 1;
        }
        stats.islands += self.roots.len() as u64;
        stats.max_island_rows = stats.max_island_rows.max(
            self.roots
                .iter()
                .map(|&r| self.nodes[r].rows as u64)
                .max()
                .unwrap_or(0),
        );
        // No index-array construction (or indirect row access) for the common single-island case.
        if self.roots.len() <= 1 {
            return;
        }
        // Bodies are BodyId-sorted. Stable island order is minimum mutable BodyId, not union history.
        self.roots.sort_unstable_by_key(|&r| self.nodes[r].first);
        reserve(&mut self.offsets, self.roots.len() + 1, stats);
        self.offsets.push(0);
        for &root in &self.roots {
            let start = *self.offsets.last().expect("offset zero");
            self.nodes[root].cursor = start;
            self.offsets.push(start + self.nodes[root].rows);
        }
        reserve(&mut self.rows, constraints.len(), stats);
        self.rows.resize(constraints.len(), 0);
        for (index, c) in constraints.iter().enumerate() {
            let root = self.row_root(c, bodies, stats);
            let cursor = self.nodes[root].cursor;
            self.rows[cursor] = index;
            self.nodes[root].cursor += 1;
        }
    }
}

fn record_island(outcome: Outcome, rows: usize, limit: u8, stats: &mut IslandStats) {
    stats.island_iterations += u64::from(outcome.passes);
    stats.skipped_constraint_visits += u64::from(limit - outcome.passes) * rows as u64;
    match outcome.exit {
        Exit::Converged => stats.converged_islands += 1,
        Exit::Capped => stats.capped_islands += 1,
        _ => {}
    }
}

pub(super) fn solve<const PREPARED: bool>(
    bodies: &mut [Body],
    responses: &[PreparedResponse],
    constraints: &mut [Constraint],
    iterations: u8,
    tolerance: Convergence,
    scratch: &mut Scratch,
    report: &mut Report,
) {
    if constraints.is_empty() || iterations <= 4 {
        // After the shared two-pass attempt, the next local probe is at four. At a ceiling
        // of four or less there can be no further early exit: avoid all grouping cost.
        let visits = report.convergence.constraint_visits;
        convergence::solve::<PREPARED, true>(
            bodies,
            responses,
            constraints,
            iterations,
            tolerance,
            report,
        );
        report.islands.skipped_constraint_visits += constraints.len() as u64
            * u64::from(iterations)
            - (report.convergence.constraint_visits - visits);
        return;
    }
    // Most quiescent contact systems converge after two passes. Try the original global proof
    // first, without allocating/indexing anything. If it fails, build fresh partitions and resume
    // from pass three. The next unchanged per-island certificate is after total pass four.
    let prefix = convergence::solve_prefix::<PREPARED>(
        bodies,
        responses,
        constraints,
        iterations,
        tolerance,
        report,
    );
    report.islands.shared_prefix_iterations += u64::from(prefix.passes);
    if prefix.exit != Exit::Pending {
        report.islands.prefix_converged_substeps += u64::from(prefix.exit == Exit::Converged);
        report.islands.skipped_constraint_visits +=
            u64::from(iterations - prefix.passes) * constraints.len() as u64;
        convergence::record_substep(prefix, iterations, report);
        return;
    }
    scratch.prepare(bodies, constraints, &mut report.islands);
    let outcome = if scratch.roots.len() == 1 {
        report.islands.single_island_substeps += 1;
        let result = convergence::solve_all_after_prefix::<PREPARED>(
            bodies,
            responses,
            constraints,
            iterations,
            tolerance,
            report,
        );
        record_island(result, constraints.len(), iterations, &mut report.islands);
        result
    } else {
        let mut result = Outcome {
            passes: 0,
            exit: Exit::Converged,
        };
        for bounds in scratch.offsets.windows(2) {
            let selected = &scratch.rows[bounds[0]..bounds[1]];
            let island = convergence::solve_selected::<PREPARED>(
                bodies,
                responses,
                constraints,
                selected,
                iterations,
                tolerance,
                report,
            );
            record_island(island, selected.len(), iterations, &mut report.islands);
            result.passes = result.passes.max(island.passes);
            if island.exit == Exit::Capped {
                result.exit = Exit::Capped;
            }
        }
        result
    };
    convergence::record_substep(outcome, iterations, report);
}

#[cfg(test)]
mod tests;
