use std::{error::Error, fmt};

use crate::{
    BodyId, ContactNormal, RigidBody, TimeOfImpact, Vec3i, World, overlap_aabb, swept_aabb,
};

/// Axis-aligned query shape in the current world snapshot.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Aabb {
    pub center: Vec3i,
    pub half_extents: Vec3i,
}

impl Aabb {
    #[must_use]
    pub const fn new(center: Vec3i, half_extents: Vec3i) -> Self {
        Self {
            center,
            half_extents,
        }
    }
}

/// Parametric ray `origin + direction * t`, where `t` is measured in engine ticks.
///
/// The direction is intentionally not normalized. This keeps query time in the same deterministic
/// integer coordinate system as body velocity.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Ray {
    pub origin: Vec3i,
    pub direction: Vec3i,
}

impl Ray {
    #[must_use]
    pub const fn new(origin: Vec3i, direction: Vec3i) -> Self {
        Self { origin, direction }
    }
}

/// Deterministically ordered snapshot-query hit.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct QueryHit {
    pub body: BodyId,
    pub time: TimeOfImpact,
    /// `None` means the query shape already overlapped the body at `t = 0`.
    pub normal: Option<ContactNormal>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum QueryError {
    InvalidHalfExtents,
    NonPositiveTicks(i32),
}

impl fmt::Display for QueryError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidHalfExtents => write!(formatter, "query AABB has negative half extents"),
            Self::NonPositiveTicks(ticks) => {
                write!(
                    formatter,
                    "physics query requires positive ticks, got {ticks}"
                )
            }
        }
    }
}

impl Error for QueryError {}

impl World {
    /// Returns all bodies touching or overlapping `query` in stable `BodyId` order.
    pub fn overlap_query(&self, query: Aabb) -> Result<Vec<BodyId>, QueryError> {
        validate_query(query)?;
        let probe = query_body(query, Vec3i::ZERO);
        Ok(self
            .bodies()
            .filter_map(|body| {
                let snapshot = snapshot_body(body);
                overlap_aabb(&probe, &snapshot).then_some(body.id())
            })
            .collect())
    }

    /// Sweeps an AABB against the current world snapshot.
    ///
    /// Body velocities are deliberately ignored: this is a spatial query over the current world
    /// state, not another simulation step. Hits are ordered by time of impact and then `BodyId`.
    pub fn cast_aabb(
        &self,
        query: Aabb,
        velocity: Vec3i,
        ticks: i32,
    ) -> Result<Vec<QueryHit>, QueryError> {
        validate_query(query)?;
        validate_ticks(ticks)?;
        Ok(cast_snapshot(self, query, velocity, ticks))
    }

    /// Casts a zero-volume point along a parametric ray against the current world snapshot.
    ///
    /// Hits are ordered nearest-first by parametric tick time and then by `BodyId`.
    pub fn ray_cast(&self, ray: Ray, ticks: i32) -> Result<Vec<QueryHit>, QueryError> {
        self.cast_aabb(Aabb::new(ray.origin, Vec3i::ZERO), ray.direction, ticks)
    }

    /// Returns only the first deterministic ray hit without allocating or sorting the complete hit set.
    pub fn ray_cast_first(&self, ray: Ray, ticks: i32) -> Result<Option<QueryHit>, QueryError> {
        let query = Aabb::new(ray.origin, Vec3i::ZERO);
        validate_query(query)?;
        validate_ticks(ticks)?;
        Ok(first_snapshot_hit(self, query, ray.direction, ticks))
    }
}

fn cast_snapshot(world: &World, query: Aabb, velocity: Vec3i, ticks: i32) -> Vec<QueryHit> {
    let probe = query_body(query, velocity);
    let mut hits = world
        .bodies()
        .filter_map(|body| snapshot_hit(&probe, body, ticks))
        .collect::<Vec<_>>();

    hits.sort_by_key(hit_order_key);
    hits
}

fn first_snapshot_hit(
    world: &World,
    query: Aabb,
    velocity: Vec3i,
    ticks: i32,
) -> Option<QueryHit> {
    let probe = query_body(query, velocity);
    world
        .bodies()
        .filter_map(|body| snapshot_hit(&probe, body, ticks))
        .min_by_key(hit_order_key)
}

fn snapshot_hit(probe: &RigidBody, body: &RigidBody, ticks: i32) -> Option<QueryHit> {
    let snapshot = snapshot_body(body);
    if overlap_aabb(probe, &snapshot) {
        return Some(QueryHit {
            body: body.id(),
            time: TimeOfImpact::from_subticks(0),
            normal: None,
        });
    }

    swept_aabb(probe, &snapshot, ticks).map(|hit| QueryHit {
        body: body.id(),
        time: hit.time,
        normal: Some(hit.normal),
    })
}

fn hit_order_key(hit: &QueryHit) -> (u64, BodyId) {
    (hit.time.subticks(), hit.body)
}

fn validate_query(query: Aabb) -> Result<(), QueryError> {
    if query.half_extents.x < 0 || query.half_extents.y < 0 || query.half_extents.z < 0 {
        return Err(QueryError::InvalidHalfExtents);
    }
    Ok(())
}

fn validate_ticks(ticks: i32) -> Result<(), QueryError> {
    if ticks <= 0 {
        return Err(QueryError::NonPositiveTicks(ticks));
    }
    Ok(())
}

fn query_body(query: Aabb, velocity: Vec3i) -> RigidBody {
    RigidBody::dynamic(BodyId(u64::MAX), query.center, velocity, query.half_extents)
}

fn snapshot_body(body: &RigidBody) -> RigidBody {
    RigidBody::fixed(body.id(), body.position(), body.half_extents())
}
