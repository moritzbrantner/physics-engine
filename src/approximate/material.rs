//! Contact materials and the pairwise coefficient combine rules.

use super::{Body, Real};

/// How two bodies' friction or restitution coefficients combine at a contact, as Rapier's
/// `CoefficientCombineRule`. When the two bodies' rules differ, the later variant wins
/// (Average < Min < Multiply < Max), as in Rapier.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum CombineRule {
    /// `(a + b) / 2`, Rapier's default.
    #[default]
    Average,
    Min,
    Multiply,
    Max,
}
impl CombineRule {
    /// The pair coefficient of `a` (with `rule_a`) and `b` (with `rule_b`).
    #[must_use]
    pub fn combine(a: Real, rule_a: Self, b: Real, rule_b: Self) -> Real {
        match rule_a.max(rule_b) {
            Self::Average => (a + b) * 0.5,
            Self::Min => a.min(b),
            Self::Multiply => a * b,
            Self::Max => a.max(b),
        }
    }
    #[cfg(feature = "checkpoints")]
    pub(super) const fn bits(self) -> u16 {
        self as u16
    }
    #[cfg(feature = "checkpoints")]
    pub(super) const fn from_bits(bits: u16) -> Self {
        match bits & 3 {
            0 => Self::Average,
            1 => Self::Min,
            2 => Self::Multiply,
            _ => Self::Max,
        }
    }
}
/// A contact material: the [`Body`] fields of the same names, for bodies built by the world
/// (heightfields, [`HeightfieldDesc::material`](super::HeightfieldDesc::material)). The default
/// is Rapier's collider default.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Material {
    pub friction: Real,
    pub restitution: Real,
    pub friction_combine: CombineRule,
    pub restitution_combine: CombineRule,
}
impl Default for Material {
    fn default() -> Self {
        Self {
            friction: 0.5,
            restitution: 0.0,
            friction_combine: CombineRule::Average,
            restitution_combine: CombineRule::Average,
        }
    }
}
impl Body {
    /// Sets the body's contact material.
    pub fn set_material(&mut self, material: Material) {
        self.friction = material.friction;
        self.restitution = material.restitution;
        self.friction_combine = material.friction_combine;
        self.restitution_combine = material.restitution_combine;
    }
}
/// The contact friction of `a` against `b`.
pub(super) fn pair_friction(a: &Body, b: &Body) -> Real {
    CombineRule::combine(
        a.friction,
        a.friction_combine,
        b.friction,
        b.friction_combine,
    )
}
/// The contact restitution of `a` against `b`.
pub(super) fn pair_restitution(a: &Body, b: &Body) -> Real {
    CombineRule::combine(
        a.restitution,
        a.restitution_combine,
        b.restitution,
        b.restitution_combine,
    )
}
