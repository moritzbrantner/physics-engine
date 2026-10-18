/// Stable engine-local body identity.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct BodyId(pub u64);

/// Symmetric collision-layer membership and mask.
///
/// A pair is eligible only when each body's membership intersects the other body's mask. The default
/// lets every layer interact with every other layer. Layers affect collision discovery only.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CollisionLayers3d {
    memberships: u32,
    mask: u32,
}

impl CollisionLayers3d {
    pub const ALL: Self = Self::new(u32::MAX, u32::MAX);

    #[must_use]
    pub const fn new(memberships: u32, mask: u32) -> Self {
        Self { memberships, mask }
    }

    #[must_use]
    pub const fn memberships(self) -> u32 {
        self.memberships
    }

    #[must_use]
    pub const fn mask(self) -> u32 {
        self.mask
    }

    #[must_use]
    pub const fn collides_with(self, other: Self) -> bool {
        self.memberships & other.mask != 0 && other.memberships & self.mask != 0
    }
}

impl Default for CollisionLayers3d {
    fn default() -> Self {
        Self::ALL
    }
}
