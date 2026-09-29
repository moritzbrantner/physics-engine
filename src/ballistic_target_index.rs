use super::bounds_overlap;

type Bounds = ([i64; 3], [i64; 3]);

#[derive(Debug)]
struct Node {
    bounds: Bounds,
    target: Option<usize>,
    /// First node after this subtree in the preorder array.
    end: usize,
}

/// Immutable index for one prepared ballistic interval. Leaves refer to derived target rows, never
/// owned rigid bodies. Preorder storage allows allocation-free queries without a traversal stack.
#[derive(Debug, Default)]
pub(super) struct BallisticTargetIndex3d {
    nodes: Vec<Node>,
}

impl BallisticTargetIndex3d {
    pub(super) fn build(bounds: impl Iterator<Item = Bounds>) -> Self {
        let mut leaves = bounds.enumerate().collect::<Vec<_>>();
        let mut index = Self {
            nodes: Vec::with_capacity(leaves.len().saturating_mul(2)),
        };
        if !leaves.is_empty() {
            index.build_subtree(&mut leaves);
        }
        index
    }

    fn build_subtree(&mut self, leaves: &mut [(usize, Bounds)]) {
        let bounds = leaves
            .iter()
            .skip(1)
            .fold(leaves[0].1, |mut bounds, (_, next)| {
                for axis in 0..3 {
                    bounds.0[axis] = bounds.0[axis].min(next.0[axis]);
                    bounds.1[axis] = bounds.1[axis].max(next.1[axis]);
                }
                bounds
            });
        let node = self.nodes.len();
        self.nodes.push(Node {
            bounds,
            target: (leaves.len() == 1).then_some(leaves[0].0),
            end: node + 1,
        });
        if leaves.len() > 1 {
            let axis = (0..3)
                .max_by_key(|&axis| i128::from(bounds.1[axis]) - i128::from(bounds.0[axis]))
                .expect("three spatial axes");
            let middle = leaves.len() / 2;
            leaves.select_nth_unstable_by_key(middle, |(index, bounds)| {
                (
                    i128::from(bounds.0[axis]) + i128::from(bounds.1[axis]),
                    *index,
                )
            });
            let (left, right) = leaves.split_at_mut(middle);
            self.build_subtree(left);
            self.build_subtree(right);
            self.nodes[node].end = self.nodes.len();
        }
    }

    pub(super) fn for_each_overlapping<E>(
        &self,
        bounds: Bounds,
        mut visit: impl FnMut(usize) -> Result<(), E>,
    ) -> Result<u64, E> {
        let mut cursor = 0;
        let mut bound_checks = 0_u64;
        while let Some(node) = self.nodes.get(cursor) {
            bound_checks = bound_checks.saturating_add(1);
            if !bounds_overlap(bounds, node.bounds) {
                cursor = node.end;
                continue;
            }
            if let Some(target) = node.target {
                visit(target)?;
            }
            cursor += 1;
        }
        Ok(bound_checks)
    }
}
