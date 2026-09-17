use std::collections::BTreeMap;

use crate::BodyId;

/// Stable, consumer-defined interaction category used to select pair behavior.
///
/// The engine deliberately does not assign gameplay meaning to category values. Consumers may reserve
/// categories for concepts such as characters, projectiles, crates, vehicles, terrain, or debris while the
/// physics engine remains authoritative only for how a resolved policy is consumed.
#[derive(Clone, Copy, Debug, Default, Eq, Ord, PartialEq, PartialOrd)]
pub struct InteractionCategory3d(u16);

impl InteractionCategory3d {
    pub const DEFAULT: Self = Self(0);

    #[must_use]
    pub const fn new(value: u16) -> Self {
        Self(value)
    }

    #[must_use]
    pub const fn value(self) -> u16 {
        self.0
    }
}

/// Pair-specific simulation budget and behavior owned by the physics engine.
///
/// `None` preserves the engine's existing fixed-boundary stabilization budget. `Some(limit)` caps that
/// stage for the resolved category pair; zero deliberately disables that post-step stabilization stage. A
/// pair policy may reduce an engine hard limit but can never raise it.
///
/// Additional pair-level knobs should be added here only when an existing physics stage can consume them.
/// Collision eligibility remains the responsibility of collision layers, and material coefficients remain
/// the responsibility of materials.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct InteractionPolicy3d {
    fixed_boundary_stabilization_pass_limit: Option<u8>,
}

impl InteractionPolicy3d {
    #[must_use]
    pub const fn with_fixed_boundary_stabilization_pass_limit(mut self, limit: u8) -> Self {
        self.fixed_boundary_stabilization_pass_limit = Some(limit);
        self
    }

    #[must_use]
    pub const fn configured_fixed_boundary_stabilization_pass_limit(self) -> Option<u8> {
        self.fixed_boundary_stabilization_pass_limit
    }

    /// Resolves this pair's configured budget against the engine-owned hard limit.
    #[must_use]
    pub const fn fixed_boundary_stabilization_pass_limit(self, engine_limit: u8) -> u8 {
        match self.fixed_boundary_stabilization_pass_limit {
            Some(limit) if limit < engine_limit => limit,
            _ => engine_limit,
        }
    }
}

/// Deterministic interaction-category and pair-policy registry.
///
/// Body/category assignment is kept outside rigid-body geometry so gameplay taxonomy does not become a
/// physical material or collision-layer concern. Symmetric pair overrides are the normal path. Directional
/// overrides are available for interactions such as character -> crate and take precedence for that ordered
/// pair only. Resolution order is directional override, symmetric pair override, then the default policy.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct InteractionPolicies3d {
    default_policy: InteractionPolicy3d,
    body_categories: BTreeMap<BodyId, InteractionCategory3d>,
    symmetric_pairs: BTreeMap<(InteractionCategory3d, InteractionCategory3d), InteractionPolicy3d>,
    directional_pairs: BTreeMap<(InteractionCategory3d, InteractionCategory3d), InteractionPolicy3d>,
}

impl InteractionPolicies3d {
    #[must_use]
    pub const fn default_policy(&self) -> InteractionPolicy3d {
        self.default_policy
    }

    pub fn set_default_policy(&mut self, policy: InteractionPolicy3d) {
        self.default_policy = policy;
    }

    /// Assigns one body to a category. Assigning [`InteractionCategory3d::DEFAULT`] removes explicit state.
    pub fn set_body_category(
        &mut self,
        body: BodyId,
        category: InteractionCategory3d,
    ) -> Option<InteractionCategory3d> {
        if category == InteractionCategory3d::DEFAULT {
            self.body_categories.remove(&body)
        } else {
            self.body_categories.insert(body, category)
        }
    }

    pub fn clear_body_category(&mut self, body: BodyId) -> Option<InteractionCategory3d> {
        self.body_categories.remove(&body)
    }

    #[must_use]
    pub fn body_category(&self, body: BodyId) -> InteractionCategory3d {
        self.body_categories
            .get(&body)
            .copied()
            .unwrap_or(InteractionCategory3d::DEFAULT)
    }

    /// Sets a symmetric category-pair policy. Resolution is independent of argument order.
    pub fn set_pair_policy(
        &mut self,
        left: InteractionCategory3d,
        right: InteractionCategory3d,
        policy: InteractionPolicy3d,
    ) -> Option<InteractionPolicy3d> {
        self.symmetric_pairs
            .insert(canonical_pair(left, right), policy)
    }

    pub fn clear_pair_policy(
        &mut self,
        left: InteractionCategory3d,
        right: InteractionCategory3d,
    ) -> Option<InteractionPolicy3d> {
        self.symmetric_pairs.remove(&canonical_pair(left, right))
    }

    /// Sets an ordered override for interactions whose initiator/target roles are meaningful.
    pub fn set_directional_policy(
        &mut self,
        source: InteractionCategory3d,
        target: InteractionCategory3d,
        policy: InteractionPolicy3d,
    ) -> Option<InteractionPolicy3d> {
        self.directional_pairs.insert((source, target), policy)
    }

    pub fn clear_directional_policy(
        &mut self,
        source: InteractionCategory3d,
        target: InteractionCategory3d,
    ) -> Option<InteractionPolicy3d> {
        self.directional_pairs.remove(&(source, target))
    }

    /// Resolves one ordered category interaction.
    #[must_use]
    pub fn policy(
        &self,
        source: InteractionCategory3d,
        target: InteractionCategory3d,
    ) -> InteractionPolicy3d {
        self.directional_pairs
            .get(&(source, target))
            .or_else(|| self.symmetric_pairs.get(&canonical_pair(source, target)))
            .copied()
            .unwrap_or(self.default_policy)
    }

    /// Resolves one ordered body interaction through the current body/category assignments.
    #[must_use]
    pub fn policy_for_bodies(&self, source: BodyId, target: BodyId) -> InteractionPolicy3d {
        self.policy(self.body_category(source), self.body_category(target))
    }
}

fn canonical_pair(
    left: InteractionCategory3d,
    right: InteractionCategory3d,
) -> (InteractionCategory3d, InteractionCategory3d) {
    if left <= right {
        (left, right)
    } else {
        (right, left)
    }
}

#[cfg(test)]
mod tests {
    use crate::BodyId;

    use super::{InteractionCategory3d, InteractionPolicies3d, InteractionPolicy3d};

    const CHARACTER: InteractionCategory3d = InteractionCategory3d::new(1);
    const CRATE: InteractionCategory3d = InteractionCategory3d::new(2);
    const GROUND: InteractionCategory3d = InteractionCategory3d::new(3);

    #[test]
    fn symmetric_pair_policy_is_order_independent() {
        let mut policies = InteractionPolicies3d::default();
        let cheap_resting =
            InteractionPolicy3d::default().with_fixed_boundary_stabilization_pass_limit(2);
        policies.set_pair_policy(CRATE, GROUND, cheap_resting);

        assert_eq!(policies.policy(CRATE, GROUND), cheap_resting);
        assert_eq!(policies.policy(GROUND, CRATE), cheap_resting);
    }

    #[test]
    fn directional_override_wins_only_for_its_ordered_pair() {
        let mut policies = InteractionPolicies3d::default();
        let ordinary =
            InteractionPolicy3d::default().with_fixed_boundary_stabilization_pass_limit(4);
        let character_push =
            InteractionPolicy3d::default().with_fixed_boundary_stabilization_pass_limit(1);
        policies.set_pair_policy(CHARACTER, CRATE, ordinary);
        policies.set_directional_policy(CHARACTER, CRATE, character_push);

        assert_eq!(policies.policy(CHARACTER, CRATE), character_push);
        assert_eq!(policies.policy(CRATE, CHARACTER), ordinary);
    }

    #[test]
    fn body_categories_drive_pair_resolution_without_becoming_material_state() {
        let character = BodyId(10);
        let crate_body = BodyId(20);
        let mut policies = InteractionPolicies3d::default();
        let tuned =
            InteractionPolicy3d::default().with_fixed_boundary_stabilization_pass_limit(3);
        policies.set_body_category(character, CHARACTER);
        policies.set_body_category(crate_body, CRATE);
        policies.set_pair_policy(CHARACTER, CRATE, tuned);

        assert_eq!(policies.policy_for_bodies(character, crate_body), tuned);
        assert_eq!(
            policies.body_category(BodyId(30)),
            InteractionCategory3d::DEFAULT
        );
    }

    #[test]
    fn pair_budget_can_only_reduce_the_engine_hard_limit() {
        let unrestricted = InteractionPolicy3d::default();
        let cheap = InteractionPolicy3d::default().with_fixed_boundary_stabilization_pass_limit(2);
        let oversized =
            InteractionPolicy3d::default().with_fixed_boundary_stabilization_pass_limit(24);

        assert_eq!(unrestricted.fixed_boundary_stabilization_pass_limit(16), 16);
        assert_eq!(cheap.fixed_boundary_stabilization_pass_limit(16), 2);
        assert_eq!(oversized.fixed_boundary_stabilization_pass_limit(16), 16);
    }

    #[test]
    fn resetting_body_to_default_category_removes_explicit_assignment() {
        let body = BodyId(7);
        let mut policies = InteractionPolicies3d::default();
        policies.set_body_category(body, CRATE);
        assert_eq!(policies.body_category(body), CRATE);

        policies.set_body_category(body, InteractionCategory3d::DEFAULT);
        assert_eq!(
            policies.body_category(body),
            InteractionCategory3d::DEFAULT
        );
    }
}
