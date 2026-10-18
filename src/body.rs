/// Engine-assigned body identity: a 32-bit slot and a 32-bit generation packed in a `u64`.
///
/// Ids are never reused: when a slot is reused its generation increases, so an id of a removed
/// body is rejected and never aliases a newer body. Lookup is a direct slot index.
///
/// Ids are handles, not an order: ties and traversal follow [`Body::key`](crate::approximate::Body)
/// order (keyed bodies first, by key; then keyless bodies by creation).
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct BodyId(u64);

impl BodyId {
    /// Placeholder carried by a body that has not been added to a world.
    pub const NONE: Self = Self(u64::MAX);

    pub(crate) const fn new(slot: u32, generation: u32) -> Self {
        Self(((generation as u64) << 32) | slot as u64)
    }
    #[must_use]
    pub const fn slot(self) -> u32 {
        self.0 as u32
    }
    #[must_use]
    pub const fn generation(self) -> u32 {
        (self.0 >> 32) as u32
    }
    /// The packed bits, for adapters that store ids outside the engine.
    #[must_use]
    pub const fn to_bits(self) -> u64 {
        self.0
    }
    #[must_use]
    pub const fn from_bits(bits: u64) -> Self {
        Self(bits)
    }
}

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
