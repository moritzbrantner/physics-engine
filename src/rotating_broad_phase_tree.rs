use std::collections::{BTreeMap, BTreeSet};

use crate::{BodyId, BodyKind, RotationalSweepBounds3d};

use super::{BoundedBody3d, bounds_overlap, center_twice, union_bounds, widest_axis};

type NodeIndex = usize;

#[derive(Clone, Copy, Debug)]
enum ArenaNodeKind3d {
    Leaf(BoundedBody3d),
    Branch { left: NodeIndex, right: NodeIndex },
}

#[derive(Clone, Copy, Debug)]
struct ArenaNode3d {
    bounds: RotationalSweepBounds3d,
    has_dynamic: bool,
    height: u16,
    minimum_id: BodyId,
    parent: Option<NodeIndex>,
    kind: ArenaNodeKind3d,
}

impl ArenaNode3d {
    fn leaf(body: BoundedBody3d, parent: Option<NodeIndex>) -> Self {
        Self {
            bounds: body.bounds,
            has_dynamic: body.kind == BodyKind::Dynamic,
            height: 1,
            minimum_id: body.id,
            parent,
            kind: ArenaNodeKind3d::Leaf(body),
        }
    }
}

/// Indexed, parent-linked BVH storage used by the persistent rotating broad phase.
///
/// Leaves are addressable directly by [`BodyId`]. Removing a leaf therefore walks only its ancestor
/// chain instead of recursively scanning unrelated subtrees. Arena slots are recycled deterministically
/// so long-running worlds do not grow storage merely because bodies escape their fat envelopes.
#[derive(Clone, Debug, Default)]
pub(super) struct IndexedBvh3d {
    nodes: Vec<Option<ArenaNode3d>>,
    free: Vec<NodeIndex>,
    root: Option<NodeIndex>,
    leaf_by_id: BTreeMap<BodyId, NodeIndex>,
}

impl IndexedBvh3d {
    #[must_use]
    pub(super) fn len(&self) -> usize {
        self.leaf_by_id.len()
    }

    #[must_use]
    pub(super) fn has_leaf(&self, id: BodyId) -> bool {
        self.leaf_by_id.contains_key(&id)
    }

    #[must_use]
    pub(super) fn leaf_bounds(&self, id: BodyId) -> Option<RotationalSweepBounds3d> {
        let index = *self.leaf_by_id.get(&id)?;
        Some(self.node(index).bounds)
    }

    #[must_use]
    pub(super) fn height(&self) -> usize {
        self.root
            .map(|root| usize::from(self.node(root).height))
            .unwrap_or_default()
    }

    pub(super) fn rebuild(&mut self, bodies: &mut [BoundedBody3d]) {
        self.nodes.clear();
        self.free.clear();
        self.root = None;
        self.leaf_by_id.clear();
        self.root = self.build_subtree(bodies, None);
    }

    pub(super) fn reinsert(&mut self, body: BoundedBody3d, rotations: &mut u64) -> bool {
        if !self.remove_leaf(body.id, rotations) {
            return false;
        }

        let leaf = self.alloc_node(ArenaNode3d::leaf(body, None));
        self.leaf_by_id.insert(body.id, leaf);
        self.insert_leaf(leaf, rotations);
        true
    }

    #[must_use]
    pub(super) fn candidate_pairs(&self) -> Vec<(BodyId, BodyId)> {
        let Some(root) = self.root else {
            return Vec::new();
        };
        let mut pairs = Vec::new();
        self.collect_pairs_within(root, &mut pairs);
        pairs.sort_unstable();
        pairs
    }

    fn node(&self, index: NodeIndex) -> &ArenaNode3d {
        self.nodes[index]
            .as_ref()
            .expect("indexed BVH references only occupied arena slots")
    }

    fn node_mut(&mut self, index: NodeIndex) -> &mut ArenaNode3d {
        self.nodes[index]
            .as_mut()
            .expect("indexed BVH references only occupied arena slots")
    }

    fn alloc_node(&mut self, node: ArenaNode3d) -> NodeIndex {
        if let Some(index) = self.free.pop() {
            debug_assert!(self.nodes[index].is_none());
            self.nodes[index] = Some(node);
            index
        } else {
            let index = self.nodes.len();
            self.nodes.push(Some(node));
            index
        }
    }

    fn free_node(&mut self, index: NodeIndex) {
        let previous = self.nodes[index].take();
        debug_assert!(previous.is_some());
        self.free.push(index);
    }

    fn build_subtree(
        &mut self,
        bodies: &mut [BoundedBody3d],
        parent: Option<NodeIndex>,
    ) -> Option<NodeIndex> {
        if bodies.is_empty() {
            return None;
        }
        if let [body] = bodies {
            let index = self.alloc_node(ArenaNode3d::leaf(*body, parent));
            self.leaf_by_id.insert(body.id, index);
            return Some(index);
        }

        let aggregate = bodies
            .iter()
            .skip(1)
            .fold(bodies[0].bounds, |bounds, body| {
                union_bounds(bounds, body.bounds)
            });
        let axis = widest_axis(aggregate);
        bodies.sort_by_key(|body| (center_twice(body.bounds, axis), body.id));
        let middle = bodies.len() / 2;
        let (left_bodies, right_bodies) = bodies.split_at_mut(middle);

        let left = self
            .build_subtree(left_bodies, None)
            .expect("non-empty left BVH partition");
        let right = self
            .build_subtree(right_bodies, None)
            .expect("non-empty right BVH partition");
        let branch = self.alloc_branch(left, right, parent);
        self.node_mut(left).parent = Some(branch);
        self.node_mut(right).parent = Some(branch);
        Some(branch)
    }

    fn alloc_branch(
        &mut self,
        left: NodeIndex,
        right: NodeIndex,
        parent: Option<NodeIndex>,
    ) -> NodeIndex {
        let left_node = *self.node(left);
        let right_node = *self.node(right);
        self.alloc_node(ArenaNode3d {
            bounds: union_bounds(left_node.bounds, right_node.bounds),
            has_dynamic: left_node.has_dynamic || right_node.has_dynamic,
            height: left_node.height.max(right_node.height).saturating_add(1),
            minimum_id: left_node.minimum_id.min(right_node.minimum_id),
            parent,
            kind: ArenaNodeKind3d::Branch { left, right },
        })
    }

    fn remove_leaf(&mut self, id: BodyId, rotations: &mut u64) -> bool {
        let Some(leaf) = self.leaf_by_id.remove(&id) else {
            return false;
        };
        let parent = self.node(leaf).parent;
        let Some(parent) = parent else {
            debug_assert_eq!(self.root, Some(leaf));
            self.root = None;
            self.free_node(leaf);
            return true;
        };

        let (left, right) = self.branch_children(parent);
        let sibling = if left == leaf { right } else { left };
        let grandparent = self.node(parent).parent;

        if let Some(grandparent) = grandparent {
            self.replace_child(grandparent, parent, sibling);
            self.node_mut(sibling).parent = Some(grandparent);
        } else {
            self.root = Some(sibling);
            self.node_mut(sibling).parent = None;
        }

        self.free_node(leaf);
        self.free_node(parent);
        if let Some(grandparent) = grandparent {
            self.refit_and_rebalance_upwards(grandparent, rotations);
        }
        true
    }

    fn insert_leaf(&mut self, leaf: NodeIndex, rotations: &mut u64) {
        let Some(mut sibling) = self.root else {
            self.root = Some(leaf);
            self.node_mut(leaf).parent = None;
            return;
        };
        let leaf_bounds = self.node(leaf).bounds;

        loop {
            match self.node(sibling).kind {
                ArenaNodeKind3d::Leaf(_) => break,
                ArenaNodeKind3d::Branch { left, right } => {
                    let left_score = self.insertion_score(left, leaf_bounds);
                    let right_score = self.insertion_score(right, leaf_bounds);
                    sibling = if left_score <= right_score {
                        left
                    } else {
                        right
                    };
                }
            }
        }

        let old_parent = self.node(sibling).parent;
        let new_parent = self.alloc_branch(sibling, leaf, old_parent);
        self.node_mut(sibling).parent = Some(new_parent);
        self.node_mut(leaf).parent = Some(new_parent);

        if let Some(old_parent) = old_parent {
            self.replace_child(old_parent, sibling, new_parent);
        } else {
            self.root = Some(new_parent);
        }
        self.refit_and_rebalance_upwards(new_parent, rotations);
    }

    fn insertion_score(
        &self,
        node: NodeIndex,
        leaf_bounds: RotationalSweepBounds3d,
    ) -> (i128, u16, BodyId) {
        let node = self.node(node);
        let expanded = union_bounds(node.bounds, leaf_bounds);
        (
            bounds_measure(expanded) - bounds_measure(node.bounds),
            node.height,
            node.minimum_id,
        )
    }

    fn branch_children(&self, index: NodeIndex) -> (NodeIndex, NodeIndex) {
        match self.node(index).kind {
            ArenaNodeKind3d::Branch { left, right } => (left, right),
            ArenaNodeKind3d::Leaf(_) => panic!("indexed BVH branch operation reached a leaf"),
        }
    }

    fn replace_child(&mut self, parent: NodeIndex, previous: NodeIndex, replacement: NodeIndex) {
        let kind = self.node(parent).kind;
        let ArenaNodeKind3d::Branch { left, right } = kind else {
            panic!("indexed BVH parent must be a branch");
        };
        self.node_mut(parent).kind = if left == previous {
            ArenaNodeKind3d::Branch {
                left: replacement,
                right,
            }
        } else if right == previous {
            ArenaNodeKind3d::Branch {
                left,
                right: replacement,
            }
        } else {
            panic!("indexed BVH parent does not reference replaced child");
        };
    }

    fn refit(&mut self, index: NodeIndex) {
        let (left, right) = self.branch_children(index);
        let left_node = *self.node(left);
        let right_node = *self.node(right);
        let node = self.node_mut(index);
        node.bounds = union_bounds(left_node.bounds, right_node.bounds);
        node.has_dynamic = left_node.has_dynamic || right_node.has_dynamic;
        node.height = left_node.height.max(right_node.height).saturating_add(1);
        node.minimum_id = left_node.minimum_id.min(right_node.minimum_id);
    }

    fn refit_and_rebalance_upwards(&mut self, start: NodeIndex, rotations: &mut u64) {
        let mut current = Some(start);
        while let Some(index) = current {
            self.refit(index);
            let subtree_root = self.rebalance_at(index, rotations);
            current = self.node(subtree_root).parent;
        }
    }

    fn rebalance_at(&mut self, index: NodeIndex, rotations: &mut u64) -> NodeIndex {
        let (left, right) = self.branch_children(index);
        let left_height = self.node(left).height;
        let right_height = self.node(right).height;

        if left_height > right_height.saturating_add(1) {
            let (left_left, left_right) = self.branch_children(left);
            if self.node(left_right).height > self.node(left_left).height {
                self.rotate_left(left, rotations);
            }
            return self.rotate_right(index, rotations);
        }

        if right_height > left_height.saturating_add(1) {
            let (right_left, right_right) = self.branch_children(right);
            if self.node(right_left).height > self.node(right_right).height {
                self.rotate_right(right, rotations);
            }
            return self.rotate_left(index, rotations);
        }

        index
    }

    fn rotate_left(&mut self, index: NodeIndex, rotations: &mut u64) -> NodeIndex {
        let (left, pivot) = self.branch_children(index);
        let (middle, pivot_right) = self.branch_children(pivot);
        let old_parent = self.node(index).parent;

        self.node_mut(index).kind = ArenaNodeKind3d::Branch {
            left,
            right: middle,
        };
        self.node_mut(middle).parent = Some(index);

        self.node_mut(pivot).kind = ArenaNodeKind3d::Branch {
            left: index,
            right: pivot_right,
        };
        self.node_mut(index).parent = Some(pivot);
        self.node_mut(pivot).parent = old_parent;

        if let Some(old_parent) = old_parent {
            self.replace_child(old_parent, index, pivot);
        } else {
            self.root = Some(pivot);
        }

        self.refit(index);
        self.refit(pivot);
        *rotations = rotations.saturating_add(1);
        pivot
    }

    fn rotate_right(&mut self, index: NodeIndex, rotations: &mut u64) -> NodeIndex {
        let (pivot, right) = self.branch_children(index);
        let (pivot_left, middle) = self.branch_children(pivot);
        let old_parent = self.node(index).parent;

        self.node_mut(index).kind = ArenaNodeKind3d::Branch {
            left: middle,
            right,
        };
        self.node_mut(middle).parent = Some(index);

        self.node_mut(pivot).kind = ArenaNodeKind3d::Branch {
            left: pivot_left,
            right: index,
        };
        self.node_mut(index).parent = Some(pivot);
        self.node_mut(pivot).parent = old_parent;

        if let Some(old_parent) = old_parent {
            self.replace_child(old_parent, index, pivot);
        } else {
            self.root = Some(pivot);
        }

        self.refit(index);
        self.refit(pivot);
        *rotations = rotations.saturating_add(1);
        pivot
    }

    fn collect_pairs_within(&self, index: NodeIndex, pairs: &mut Vec<(BodyId, BodyId)>) {
        let ArenaNodeKind3d::Branch { left, right } = self.node(index).kind else {
            return;
        };
        self.collect_pairs_within(left, pairs);
        self.collect_pairs_within(right, pairs);
        self.collect_pairs_between(left, right, pairs);
    }

    fn collect_pairs_between(
        &self,
        left: NodeIndex,
        right: NodeIndex,
        pairs: &mut Vec<(BodyId, BodyId)>,
    ) {
        let left_node = *self.node(left);
        let right_node = *self.node(right);
        if (!left_node.has_dynamic && !right_node.has_dynamic)
            || !bounds_overlap(left_node.bounds, right_node.bounds)
        {
            return;
        }

        match (left_node.kind, right_node.kind) {
            (ArenaNodeKind3d::Leaf(left_body), ArenaNodeKind3d::Leaf(right_body)) => {
                if left_body.kind == BodyKind::Fixed && right_body.kind == BodyKind::Fixed {
                    return;
                }
                let pair = if left_body.id < right_body.id {
                    (left_body.id, right_body.id)
                } else {
                    (right_body.id, left_body.id)
                };
                pairs.push(pair);
            }
            (
                ArenaNodeKind3d::Branch {
                    left: ll,
                    right: lr,
                },
                ArenaNodeKind3d::Leaf(_),
            ) => {
                self.collect_pairs_between(ll, right, pairs);
                self.collect_pairs_between(lr, right, pairs);
            }
            (
                ArenaNodeKind3d::Leaf(_),
                ArenaNodeKind3d::Branch {
                    left: rl,
                    right: rr,
                },
            ) => {
                self.collect_pairs_between(left, rl, pairs);
                self.collect_pairs_between(left, rr, pairs);
            }
            (
                ArenaNodeKind3d::Branch {
                    left: ll,
                    right: lr,
                },
                ArenaNodeKind3d::Branch {
                    left: rl,
                    right: rr,
                },
            ) => {
                self.collect_pairs_between(ll, rl, pairs);
                self.collect_pairs_between(ll, rr, pairs);
                self.collect_pairs_between(lr, rl, pairs);
                self.collect_pairs_between(lr, rr, pairs);
            }
        }
    }

    #[cfg(test)]
    pub(super) fn validate_structure(&self) -> Result<(), String> {
        if let Some(root) = self.root {
            if self.node(root).parent.is_some() {
                return Err("root has a parent".to_owned());
            }
        } else if !self.leaf_by_id.is_empty() {
            return Err("empty root retains indexed leaves".to_owned());
        }

        let mut visited = BTreeSet::new();
        if let Some(root) = self.root {
            self.validate_subtree(root, None, &mut visited)?;
        }
        let live = self.nodes.iter().filter(|node| node.is_some()).count();
        if visited.len() != live {
            return Err(format!(
                "reachable node count {} differs from live arena count {live}",
                visited.len()
            ));
        }
        if self.leaf_by_id.len() * 2 != live.saturating_add(1) && !self.leaf_by_id.is_empty() {
            return Err("full binary tree node count invariant failed".to_owned());
        }
        for (id, index) in &self.leaf_by_id {
            match self.node(*index).kind {
                ArenaNodeKind3d::Leaf(body) if body.id == *id => {}
                _ => return Err(format!("leaf index for {} is stale", id.0)),
            }
        }
        Ok(())
    }

    #[cfg(test)]
    fn validate_subtree(
        &self,
        index: NodeIndex,
        parent: Option<NodeIndex>,
        visited: &mut BTreeSet<NodeIndex>,
    ) -> Result<(), String> {
        if !visited.insert(index) {
            return Err(format!("arena node {index} is reachable more than once"));
        }
        let node = *self.node(index);
        if node.parent != parent {
            return Err(format!("arena node {index} has incorrect parent"));
        }
        match node.kind {
            ArenaNodeKind3d::Leaf(body) => {
                if node.height != 1
                    || node.minimum_id != body.id
                    || node.bounds != body.bounds
                    || node.has_dynamic != (body.kind == BodyKind::Dynamic)
                {
                    return Err(format!("leaf metadata mismatch for {}", body.id.0));
                }
            }
            ArenaNodeKind3d::Branch { left, right } => {
                self.validate_subtree(left, Some(index), visited)?;
                self.validate_subtree(right, Some(index), visited)?;
                let left_node = *self.node(left);
                let right_node = *self.node(right);
                let expected_height = left_node.height.max(right_node.height).saturating_add(1);
                if node.bounds != union_bounds(left_node.bounds, right_node.bounds)
                    || node.has_dynamic != (left_node.has_dynamic || right_node.has_dynamic)
                    || node.height != expected_height
                    || node.minimum_id != left_node.minimum_id.min(right_node.minimum_id)
                {
                    return Err(format!("branch metadata mismatch at arena node {index}"));
                }
                if left_node.height.abs_diff(right_node.height) > 1 {
                    return Err(format!("arena node {index} is height-unbalanced"));
                }
            }
        }
        Ok(())
    }
}

fn bounds_measure(bounds: RotationalSweepBounds3d) -> i128 {
    (0..3)
        .map(|axis| i128::from(bounds.maximum[axis]) - i128::from(bounds.minimum[axis]))
        .sum()
}

#[cfg(test)]
mod tests {
    use crate::{BodyId, BodyKind, RotationalSweepBounds3d};

    use super::{ArenaNode3d, IndexedBvh3d};
    use crate::rotating_broad_phase::BoundedBody3d;

    fn body(id: u64, coordinate: i64) -> BoundedBody3d {
        BoundedBody3d {
            id: BodyId(id),
            kind: BodyKind::Dynamic,
            bounds: RotationalSweepBounds3d {
                minimum: [coordinate, 0, 0],
                maximum: [coordinate + 2, 2, 2],
            },
        }
    }

    #[test]
    fn forced_skew_rotates_and_preserves_parent_links() {
        let mut tree = IndexedBvh3d::default();
        let one = tree.alloc_node(ArenaNode3d::leaf(body(1, 0), None));
        let two = tree.alloc_node(ArenaNode3d::leaf(body(2, 4), None));
        let three = tree.alloc_node(ArenaNode3d::leaf(body(3, 8), None));
        let four = tree.alloc_node(ArenaNode3d::leaf(body(4, 12), None));
        for (id, index) in [(1, one), (2, two), (3, three), (4, four)] {
            tree.leaf_by_id.insert(BodyId(id), index);
        }

        let pair = tree.alloc_branch(one, two, None);
        tree.node_mut(one).parent = Some(pair);
        tree.node_mut(two).parent = Some(pair);
        let deep_left = tree.alloc_branch(pair, three, None);
        tree.node_mut(pair).parent = Some(deep_left);
        tree.node_mut(three).parent = Some(deep_left);
        let root = tree.alloc_branch(deep_left, four, None);
        tree.node_mut(deep_left).parent = Some(root);
        tree.node_mut(four).parent = Some(root);
        tree.root = Some(root);
        assert_eq!(tree.height(), 4);

        let mut rotations = 0_u64;
        let balanced = tree.rebalance_at(root, &mut rotations);
        tree.root = Some(balanced);

        assert!(rotations > 0);
        assert!(tree.height() < 4);
        tree.validate_structure().expect("balanced indexed tree");
    }

    #[test]
    fn arena_slots_are_recycled_during_repeated_reinsertion() {
        let mut bodies = (0..127_u64)
            .map(|id| body(id + 1, i64::try_from(id).expect("small coordinate") * 4))
            .collect::<Vec<_>>();
        let mut tree = IndexedBvh3d::default();
        tree.rebuild(&mut bodies);
        let initial_slots = tree.nodes.len();
        let mut rotations = 0_u64;

        for step in 0..128_i64 {
            let moved = body(1, 10_000 + step * 16);
            assert!(tree.reinsert(moved, &mut rotations));
            tree.validate_structure()
                .expect("valid indexed tree after churn");
        }

        assert_eq!(tree.nodes.len(), initial_slots);
        assert_eq!(tree.len(), 127);
        assert!(tree.has_leaf(BodyId(1)));
    }
}
