use std::{cell::RefCell, collections::BTreeMap, mem::size_of, sync::Arc};

use crate::oriented_box::{
    PreparedObb3d, obb_contact_seed as runtime_obb_contact_seed, obb_contact_seed_prepared,
};
use crate::{BodyId, BodyKind, ObbContactSeed3d, OrientedBox3d, OrientedBoxError3d, RigidBox3d};

/// Version of the retained fixed-geometry preparation representation.
///
/// Serialized baking is deliberately not part of v1. Increment this before any future persisted artifact
/// is allowed to reuse preparation produced by a different representation.
pub const FIXED_GEOMETRY_PREPARATION_VERSION: u32 = 1;

/// Controls whether immutable scene geometry is prepared only inside ordinary runtime queries or retained
/// once by the world for reuse across steps.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum FixedGeometryPreparationMode3d {
    /// Reference path. OBB preparation happens inside the existing contact query that needs it.
    #[default]
    Runtime,
    /// Prepare genuine fixed scene bodies once and retain that exact quantized geometry across steps.
    PrepareAtLoad,
}

/// Observable preparation evidence. Counts describe only retained genuine fixed bodies; temporary dynamic
/// preparation used by an individual contact query is intentionally excluded.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FixedGeometryPreparationStats3d {
    pub mode: FixedGeometryPreparationMode3d,
    pub representation_version: u32,
    pub prepared_body_count: usize,
    pub total_preparations: u64,
    pub retained_bytes: usize,
}

#[derive(Clone, Debug, Default)]
pub(crate) struct FixedGeometryPreparationCache3d {
    mode: FixedGeometryPreparationMode3d,
    prepared: Arc<BTreeMap<BodyId, PreparedObb3d>>,
    total_preparations: u64,
}

std::thread_local! {
    static ACTIVE_FIXED_GEOMETRY: RefCell<Option<FixedGeometryPreparationCache3d>> =
        const { RefCell::new(None) };
}

struct ActiveFixedGeometryGuard {
    previous: Option<FixedGeometryPreparationCache3d>,
}

impl Drop for ActiveFixedGeometryGuard {
    fn drop(&mut self) {
        let previous = self.previous.take();
        ACTIVE_FIXED_GEOMETRY.with(|active| *active.borrow_mut() = previous);
    }
}

/// Executes one world-owned operation with that world's immutable fixed preparation visible to the shared
/// SAT entry point. The cache clone shares immutable prepared data; no retained geometry is copied or rebuilt
/// per step. Scene mutation uses copy-on-write only when fixed geometry actually changes.
pub(crate) fn with_fixed_geometry_context<R>(
    cache: &FixedGeometryPreparationCache3d,
    callback: impl FnOnce() -> R,
) -> R {
    let previous = ACTIVE_FIXED_GEOMETRY.with(|active| active.replace(Some(cache.clone())));
    let _guard = ActiveFixedGeometryGuard { previous };
    callback()
}

/// Exact OBB SAT entry point with optional world-scoped fixed-geometry preparation.
///
/// Without an active prepare-at-load world this is exactly the existing runtime path. With preparation
/// enabled, only exact quantized shapes matching a genuine fixed body reuse retained vertices/edges/faces;
/// every other shape is prepared normally. Contact semantics, validation order and checked errors remain
/// those of `obb_contact_seed_prepared`.
pub fn obb_contact_seed(
    left: OrientedBox3d,
    right: OrientedBox3d,
) -> Result<Option<ObbContactSeed3d>, OrientedBoxError3d> {
    ACTIVE_FIXED_GEOMETRY.with(|active| {
        let active = active.borrow();
        let Some(cache) = active.as_ref() else {
            return runtime_obb_contact_seed(left, right);
        };
        cache.contact_shapes(left, right)
    })
}

impl FixedGeometryPreparationCache3d {
    pub(crate) fn set_mode<'a>(
        &mut self,
        mode: FixedGeometryPreparationMode3d,
        boxes: impl IntoIterator<Item = &'a RigidBox3d>,
    ) {
        self.mode = mode;
        Arc::make_mut(&mut self.prepared).clear();
        if mode == FixedGeometryPreparationMode3d::PrepareAtLoad {
            for rigid_box in boxes {
                if rigid_box.body().kind() == BodyKind::Fixed {
                    self.prepare_fixed(rigid_box);
                }
            }
        }
    }

    pub(crate) fn register_fixed(&mut self, rigid_box: &RigidBox3d) {
        if self.mode == FixedGeometryPreparationMode3d::PrepareAtLoad
            && rigid_box.body().kind() == BodyKind::Fixed
        {
            self.prepare_fixed(rigid_box);
        }
    }

    pub(crate) fn unregister(&mut self, id: BodyId) {
        Arc::make_mut(&mut self.prepared).remove(&id);
    }

    #[must_use]
    pub(crate) fn stats(&self) -> FixedGeometryPreparationStats3d {
        FixedGeometryPreparationStats3d {
            mode: self.mode,
            representation_version: FIXED_GEOMETRY_PREPARATION_VERSION,
            prepared_body_count: self.prepared.len(),
            total_preparations: self.total_preparations,
            retained_bytes: self
                .prepared
                .len()
                .saturating_mul(size_of::<PreparedObb3d>()),
        }
    }

    fn contact_shapes(
        &self,
        left: OrientedBox3d,
        right: OrientedBox3d,
    ) -> Result<Option<ObbContactSeed3d>, OrientedBoxError3d> {
        if self.mode == FixedGeometryPreparationMode3d::Runtime {
            return runtime_obb_contact_seed(left, right);
        }
        let left = self.prepared_shape(left);
        let right = self.prepared_shape(right);
        obb_contact_seed_prepared(&left, &right)
    }

    fn prepared_shape(&self, shape: OrientedBox3d) -> PreparedObb3d {
        self.prepared
            .values()
            .find(|prepared| prepared.shape == shape)
            .copied()
            .unwrap_or_else(|| PreparedObb3d::new(shape))
    }

    fn prepare_fixed(&mut self, rigid_box: &RigidBox3d) {
        let id = rigid_box.body().id();
        let prepared = PreparedObb3d::new(rigid_box.oriented_box());
        let changed = self
            .prepared
            .get(&id)
            .is_none_or(|previous| *previous != prepared);
        Arc::make_mut(&mut self.prepared).insert(id, prepared);
        if changed {
            self.total_preparations = self.total_preparations.saturating_add(1);
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::oriented_box::obb_contact_seed as runtime_obb_contact_seed;
    use crate::{
        AngularState3d, AngularVelocity3d, BodyId, Orientation3d, RigidBody, RigidBox3d, Vec3i,
    };

    use super::{
        FixedGeometryPreparationCache3d, FixedGeometryPreparationMode3d, obb_contact_seed,
        with_fixed_geometry_context,
    };

    fn fixed(id: u64, position: Vec3i) -> RigidBox3d {
        RigidBox3d::new(
            RigidBody::fixed(BodyId(id), position, Vec3i::new(2, 2, 2)),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid fixed box")
    }

    fn dynamic(id: u64, position: Vec3i) -> RigidBox3d {
        RigidBox3d::new(
            RigidBody::dynamic(BodyId(id), position, Vec3i::ZERO, Vec3i::new(2, 2, 2)),
            AngularState3d::new(Orientation3d::IDENTITY, AngularVelocity3d::default()),
        )
        .expect("valid dynamic box")
    }

    #[test]
    fn prepare_at_load_retains_only_genuine_fixed_inputs() {
        let floor = fixed(1, Vec3i::ZERO);
        let crate_box = dynamic(2, Vec3i::new(0, 4, 0));
        let mut cache = FixedGeometryPreparationCache3d::default();
        cache.set_mode(
            FixedGeometryPreparationMode3d::PrepareAtLoad,
            [&floor, &crate_box],
        );

        let stats = cache.stats();
        assert_eq!(stats.prepared_body_count, 1);
        assert_eq!(stats.total_preparations, 1);
        assert!(stats.retained_bytes > 0);
    }

    #[test]
    fn prepared_contact_is_bit_identical_to_runtime_contact() {
        let floor = fixed(1, Vec3i::ZERO);
        let crate_box = dynamic(2, Vec3i::new(0, 4, 0));
        let expected = runtime_obb_contact_seed(crate_box.oriented_box(), floor.oriented_box())
            .expect("runtime contact");
        let mut cache = FixedGeometryPreparationCache3d::default();
        cache.set_mode(FixedGeometryPreparationMode3d::PrepareAtLoad, [&floor]);
        assert_eq!(
            with_fixed_geometry_context(&cache, || {
                obb_contact_seed(crate_box.oriented_box(), floor.oriented_box())
            })
            .expect("scoped prepared contact"),
            expected
        );
    }

    #[test]
    fn preparation_scope_restores_reference_path() {
        let floor = fixed(1, Vec3i::ZERO);
        let crate_box = dynamic(2, Vec3i::new(0, 4, 0));
        let mut cache = FixedGeometryPreparationCache3d::default();
        cache.set_mode(FixedGeometryPreparationMode3d::PrepareAtLoad, [&floor]);
        let expected = runtime_obb_contact_seed(crate_box.oriented_box(), floor.oriented_box());
        let _ = with_fixed_geometry_context(&cache, || {
            obb_contact_seed(crate_box.oriented_box(), floor.oriented_box())
        });
        assert_eq!(
            obb_contact_seed(crate_box.oriented_box(), floor.oriented_box()),
            expected
        );
    }

    #[test]
    fn removal_and_readdition_refresh_changed_fixed_geometry() {
        let first = fixed(1, Vec3i::ZERO);
        let moved = fixed(1, Vec3i::new(3, 0, 0));
        let mut cache = FixedGeometryPreparationCache3d::default();
        cache.set_mode(FixedGeometryPreparationMode3d::PrepareAtLoad, [&first]);
        cache.unregister(BodyId(1));
        assert_eq!(cache.stats().prepared_body_count, 0);
        cache.register_fixed(&moved);
        assert_eq!(cache.stats().prepared_body_count, 1);
        assert_eq!(cache.stats().total_preparations, 2);
    }
}
