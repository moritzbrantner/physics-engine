use crate::Vec3i;

pub const MATERIAL_SCALE: u16 = 1_000;

/// Stable engine-local identity. ECS entity IDs can be mapped to this type by an adapter.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct BodyId(pub u64);

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BodyKind {
    Fixed,
    Dynamic,
}

/// Collision response properties expressed without floating-point state.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Material {
    restitution_milli: u16,
}

impl Material {
    #[must_use]
    pub const fn new(restitution_milli: u16) -> Self {
        Self { restitution_milli }
    }

    #[must_use]
    pub const fn restitution_milli(self) -> u16 {
        self.restitution_milli
    }
}

/// Translational axis-aligned rigid body used by the first engine slice.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RigidBody {
    pub(crate) id: BodyId,
    pub(crate) kind: BodyKind,
    pub(crate) position: Vec3i,
    pub(crate) velocity: Vec3i,
    pub(crate) half_extents: Vec3i,
    pub(crate) mass_units: u32,
    pub(crate) material: Material,
}

impl RigidBody {
    #[must_use]
    pub const fn dynamic(
        id: BodyId,
        position: Vec3i,
        velocity: Vec3i,
        half_extents: Vec3i,
    ) -> Self {
        Self {
            id,
            kind: BodyKind::Dynamic,
            position,
            velocity,
            half_extents,
            mass_units: 1,
            material: Material::new(0),
        }
    }

    #[must_use]
    pub const fn fixed(id: BodyId, position: Vec3i, half_extents: Vec3i) -> Self {
        Self {
            id,
            kind: BodyKind::Fixed,
            position,
            velocity: Vec3i::ZERO,
            half_extents,
            mass_units: 0,
            material: Material::new(0),
        }
    }

    #[must_use]
    pub const fn with_mass(mut self, mass_units: u32) -> Self {
        self.mass_units = mass_units;
        self
    }

    #[must_use]
    pub const fn with_material(mut self, material: Material) -> Self {
        self.material = material;
        self
    }

    #[must_use]
    pub const fn id(&self) -> BodyId {
        self.id
    }

    #[must_use]
    pub const fn kind(&self) -> BodyKind {
        self.kind
    }

    #[must_use]
    pub const fn position(&self) -> Vec3i {
        self.position
    }

    #[must_use]
    pub const fn velocity(&self) -> Vec3i {
        self.velocity
    }

    #[must_use]
    pub const fn half_extents(&self) -> Vec3i {
        self.half_extents
    }

    #[must_use]
    pub const fn mass_units(&self) -> u32 {
        self.mass_units
    }

    #[must_use]
    pub const fn material(&self) -> Material {
        self.material
    }
}
