//! The narrow phase of one candidate pair, shared by the step's broad-phase pairs and by the
//! children of compound bodies (#567).

use std::sync::Arc;

use super::{
    Body, Config, Error, PreparedCompound, Real, Report, Shape, Vector, ccd, compound, contact,
    convex_hull::PreparedHull, geometry::PairGeometry, step::step_drift, terrain::Terrain,
    triangle_mesh::TriangleMesh,
};

/// The world state and substep timing one narrow-phase pass reads.
pub(super) struct Narrow<'w> {
    pub scene: &'w Config,
    pub meshes: &'w [Option<TriangleMesh>],
    pub terrains: &'w [Terrain],
    pub hulls: &'w [Option<Arc<PreparedHull>>],
    pub compounds: &'w [Option<Arc<PreparedCompound>>],
    pub h: Real,
    pub interval: Real,
    pub lookahead: Real,
    pub gain: Real,
}

impl Narrow<'_> {
    /// The manifolds of candidate pair `a`, `b` (storage indices `i`, `j`, in pair order), each
    /// handed to `emit`. Sets `regenerate` when later substeps must search the pair again.
    #[allow(clippy::too_many_arguments)]
    pub(super) fn pair(
        &self,
        geometry: &mut PairGeometry<'_>,
        [i, j]: [usize; 2],
        a: &Body,
        b: &Body,
        report: &mut Report,
        regenerate: &mut bool,
        emit: &mut impl FnMut(contact::Manifold),
    ) -> Result<(), Error> {
        let n = self;
        let (h, interval, lookahead, gain) = (n.h, n.interval, n.lookahead, n.gain);
        let driven_contact_candidate = (a.movable()
            && b.external
            && (b.velocity != Vector::ZERO || b.angular_velocity != Vector::ZERO))
            || (b.movable()
                && a.external
                && (a.velocity != Vector::ZERO || a.angular_velocity != Vector::ZERO));
        let driven_wake_candidate = driven_contact_candidate && (a.sleeping || b.sleeping);
        // Rigid spheres take one-point contacts with fixed meshes, terrain and hulls. A
        // CCD sphere (and any sphere on a mesh) takes the current contact within the
        // slop, else the first along this substep's translation, so it keeps its swept
        // time of impact; a non-CCD sphere on terrain or a hull takes the deepest
        // contact within the slop plus the step's approach, speculatively (#519).
        if let Some((surface, surface_first)) = super::static_sphere::Surface::sphere_pair(
            a.shape, b.shape, n.meshes, n.terrains, n.hulls,
        ) {
            let moving = if surface_first { b } else { a };
            if moving.inverse_mass() == 0.0
                || a.sensor
                || b.sensor
                || moving.sleeping
                || !a.layers.collides_with(b.layers)
            {
                return Ok(());
            }
            report.narrow_tests += 1;
            let failure = |reason| Error::CollisionSearchFailed {
                bodies: [a.id, b.id],
                reason,
            };
            let deferred = ccd::deferred(a, b);
            if !deferred && !matches!(surface, super::static_sphere::Surface::Mesh(_)) {
                // Terrain and hulls: the step's approach is the speculative margin, as
                // for boxes on terrain. A sphere's one point needs no regeneration for
                // spin, so later substeps reuse the row as Rapier does (#378).
                let approach = moving.velocity.length() * interval
                    + step_drift(moving, n.scene.gravity, gain).length()
                    + moving.medium_reach(n.scene.gravity, gain, lookahead);
                if let Some(m) = super::static_sphere::manifold(
                    surface,
                    a,
                    b,
                    n.scene.contact_slop + approach,
                    None,
                    &mut report.geometry,
                )
                .map_err(failure)?
                {
                    emit(m);
                }
                return Ok(());
            }
            let current = super::static_sphere::manifold(
                surface,
                a,
                b,
                n.scene.contact_slop,
                None,
                &mut report.geometry,
            )
            .map_err(failure)?;
            let current = current
                .filter(|m| !deferred || (&m.points).into_iter().all(|p| p.separation <= 0.0));
            let m = if current.is_some() {
                current
            } else {
                *regenerate = true;
                super::static_sphere::manifold(
                    surface,
                    a,
                    b,
                    n.scene.contact_slop,
                    Some(h),
                    &mut report.geometry,
                )
                .map_err(failure)?
            };
            if let Some(m) = m {
                emit(m);
            }
            return Ok(());
        }
        // Box/hull rigid contacts use the shared SAT/clipping kernel and the existing
        // speculative response rows. Unsupported pairs/CCD still fail atomically.
        if matches!(a.shape, Shape::ConvexHull { .. })
            || matches!(b.shape, Shape::ConvexHull { .. })
        {
            let moving = if matches!(a.shape, Shape::ConvexHull { .. }) {
                b
            } else {
                a
            };
            if !a.sensor
                && !b.sensor
                && moving.inverse_mass() > 0.0
                && !moving.sleeping
                && a.layers.collides_with(b.layers)
            {
                if !matches!(moving.shape, Shape::Box(_)) || moving.ccd {
                    return Err(Error::CollisionSearchFailed {
                        bodies: [a.id, b.id],
                        reason: super::SweepFailure::InvalidGeometryInput,
                    });
                }
                let Shape::ConvexHull { hull, .. } = (if matches!(a.shape, Shape::ConvexHull { .. })
                {
                    a.shape
                } else {
                    b.shape
                }) else {
                    unreachable!()
                };
                let approach = moving.velocity.length() * interval
                    + step_drift(moving, n.scene.gravity, gain).length()
                    + moving.medium_reach(n.scene.gravity, gain, lookahead)
                    + moving.angular_velocity.length() * moving.shape.radius() * interval;
                report.narrow_tests += 1;
                *regenerate = true;
                if let Some(manifold) = geometry
                    .hull_box(
                        &n.hulls[hull.0 as usize]
                            .as_deref()
                            .expect("live convex hull")
                            .hull,
                        a,
                        b,
                        n.scene.contact_slop + approach,
                        &mut report.geometry,
                    )
                    .map_err(|reason| Error::CollisionSearchFailed {
                        bodies: [a.id, b.id],
                        reason,
                    })?
                {
                    emit(manifold);
                }
            }
            return Ok(());
        }
        // Boxes take rigid contacts with heightfield terrain, one manifold per triangle.
        // As with halfspaces, the approach over the step is the speculative margin.
        if let (Shape::Heightfield { terrain, .. }, body)
        | (body, Shape::Heightfield { terrain, .. }) = (a.shape, b.shape)
            && let Shape::Box(_) = body
        {
            let moving = if matches!(a.shape, Shape::Box(_)) {
                a
            } else {
                b
            };
            if moving.inverse_mass() == 0.0
                || moving.sensor
                || moving.sleeping
                || !a.layers.collides_with(b.layers)
            {
                return Ok(());
            }
            report.narrow_tests += 1;
            let approach = moving.velocity.length() * interval
                + step_drift(moving, n.scene.gravity, gain).length()
                + moving.medium_reach(n.scene.gravity, gain, lookahead)
                + moving.angular_velocity.length() * moving.shape.radius() * interval;
            contact::heightfield_box(
                a,
                b,
                n.terrains[terrain.0 as usize]
                    .prepared
                    .as_deref()
                    .expect("a live heightfield body's terrain"),
                n.scene.contact_slop + approach,
                interval,
                &mut *emit,
            );
            return Ok(());
        }
        if let (Shape::TriangleMesh { mesh, .. }, _) | (_, Shape::TriangleMesh { mesh, .. }) =
            (a.shape, b.shape)
        {
            let moving = if matches!(a.shape, Shape::TriangleMesh { .. }) {
                b
            } else {
                a
            };
            if moving.inverse_mass() == 0.0
                || moving.sensor
                || moving.sleeping
                || !a.layers.collides_with(b.layers)
            {
                return Ok(());
            }
            let prepared = &n.meshes[mesh.0 as usize]
                .as_ref()
                .expect("live mesh")
                .prepared;
            report.narrow_tests += 1;
            let failure = |reason| Error::CollisionSearchFailed {
                bodies: [a.id, b.id],
                reason,
            };
            if matches!(moving.shape, Shape::Box(_)) {
                if moving.ccd {
                    return Err(failure(super::SweepFailure::InvalidGeometryInput));
                }
                let approach = moving.velocity.length() * interval
                    + step_drift(moving, n.scene.gravity, gain).length()
                    + moving.medium_reach(n.scene.gravity, gain, lookahead)
                    + moving.angular_velocity.length() * moving.shape.radius() * interval;
                // A full-interval bound below the accepted contact tolerance cannot
                // introduce a feature farther than that tolerance. Reuse these rows
                // for the remaining substeps, as for the shared solver's small spins;
                // meaningful translation, force drift or rotation regenerates them.
                *regenerate |= approach > n.scene.contact_slop;
                super::primitive::box_mesh_manifolds(
                    prepared,
                    a,
                    b,
                    n.scene.contact_slop + approach,
                    &mut report.geometry,
                    &mut *emit,
                )
                .map_err(failure)?;
                return Ok(());
            }
            // Spheres were handled above; no other moving shape has mesh contacts.
            return Err(failure(super::SweepFailure::InvalidGeometryInput));
        }
        if a.sensor
            || b.sensor
            // Terrain answers queries and character moves; halfspace contacts with
            // dynamic bodies are generated after this sweep.
            || a.shape.query_only()
            || b.shape.query_only()
            || !a.layers.collides_with(b.layers)
            || (a.inverse_mass() == 0.0
                && b.inverse_mass() == 0.0
                && !driven_wake_candidate)
        {
            return Ok(());
        }
        let speed = (b.velocity - a.velocity).length();
        let travel = speed * h;
        let per_step = n.scene.contact_generation == super::ContactGeneration::PerStep;
        // Travel within this substep that could carry one shape through the other.
        let deep = travel
            > 0.5
                * a.shape
                    .half_extents()
                    .min_component()
                    .min(b.shape.half_extents().min_component());
        // Retain full-interval discovery after response preparation wakes the
        // body; the second geometry pass must not discard an admitted sweep.
        // A fixed boundary may be crossed below the shape-sized fast threshold.
        // The current margin only rules out travel no larger than that margin.
        // On the debris path (#543), a deep pair of two non-CCD bodies is admitted
        // speculatively over the rest of the step, this substep included, instead of
        // taking this substep's sweep, as Rapier's is.
        let fast = driven_contact_candidate
            || a.ccd
            || b.ccd
            || ((a.mass == 0.0 || b.mass == 0.0) && travel > n.scene.contact_slop)
            || (!per_step && deep);
        // A tolerance-only manifold is not wake evidence. Probe the actual surface
        // and full interval before replacing the parked body's zero response.
        let margin = if driven_wake_candidate {
            0.0
        } else {
            n.scene.contact_slop
        };
        // A separated pair slow enough to skip the sweep is admitted speculatively
        // when it can close before the step ends; a fast pair keeps this substep's
        // sweep instead.
        let window = if per_step && deep {
            interval
        } else {
            lookahead
        };
        let reach = if fast || window == 0.0 {
            0.0
        } else {
            (speed
                + a.angular_velocity.length() * a.shape.radius()
                + b.angular_velocity.length() * b.shape.radius())
                * window
                + (step_drift(b, n.scene.gravity, gain) - step_drift(a, n.scene.gravity, gain))
                    .length()
                + a.medium_reach(n.scene.gravity, gain, lookahead)
                + b.medium_reach(n.scene.gravity, gain, lookahead)
        };
        // Separated co-moving CCD pairs never get here: the broad phase drops them
        // (`skip_separated_ccd`).
        report.narrow_tests += 1;
        // The debris path takes current and speculative contacts in one query.
        let single = per_step && reach > 0.0;
        // GeometryCache shares read-only frames even for rotating pairs; it retains
        // complete pair results only when both orientation dependencies are stable.
        let current = geometry
            .query_interval(
                [i, j],
                [a, b],
                h,
                if single { margin + reach } else { margin },
                &mut report.geometry,
            )
            // A CCD body still separated by a gap it closes well within this
            // substep would stop at the surface on a speculative row and bounce
            // only after the substep, losing the rest of its travel. Its swept
            // time of impact and remaining-time response consume the substep
            // instead, as high-resolution Rapier does.
            .filter(|m| {
                !(ccd::deferred(a, b) && {
                    let gap = (&m.points)
                        .into_iter()
                        .map(|p| p.separation)
                        .fold(Real::INFINITY, Real::min);
                    gap > 0.0
                        && -(b.velocity - a.velocity).dot(m.normal) * h > gap + n.scene.contact_slop
                })
            });
        let m = if current.is_some() {
            current
        } else if fast {
            *regenerate = true;
            geometry
                .swept([i, j], [a, b], h, margin, &mut report.geometry)
                .map_err(|reason| Error::CollisionSearchFailed {
                    bodies: [a.id, b.id],
                    reason,
                })?
        } else if reach > 0.0 && !single {
            let margin = margin + reach;
            let speculative =
                geometry.query_interval([i, j], [a, b], h, margin, &mut report.geometry);
            if speculative.is_some() && a.mass > 0.0 && b.mass > 0.0 {
                // Speculative rows between two dynamic bodies make stacks rock: such
                // a pair is discovered by the next substep's regeneration instead.
                *regenerate = true;
                None
            } else {
                speculative
            }
        } else {
            None
        };
        if let Some(m) = m {
            emit(m);
        }
        Ok(())
    }

    /// The manifolds of a candidate pair with a compound side: every pair of children (a
    /// primitive side is its own one child) whose padded bounds can meet within the substep's
    /// interval goes through [`Self::pair`] as bodies of their own, uncached, in child order.
    /// Their arms are moved to the parents' centres, so the pair's rows act on the compound.
    #[allow(clippy::too_many_arguments)]
    pub(super) fn compound_pair(
        &self,
        [i, j]: [usize; 2],
        a: &Body,
        b: &Body,
        report: &mut Report,
        regenerate: &mut bool,
        emit: &mut impl FnMut(contact::Manifold),
    ) -> Result<(), Error> {
        let (ca, cb) = (
            compound::children(self.compounds, a),
            compound::children(self.compounds, b),
        );
        // Reach of a body's points over the rest of the step, beyond its bounds.
        let reach = |x: &Body| {
            if x.sleeping || x.mass == 0.0 {
                return 0.0;
            }
            (x.velocity.length() + x.angular_velocity.length() * x.shape.radius()) * self.interval
                + step_drift(x, self.scene.gravity, self.gain).length()
        };
        let pad = reach(a) + reach(b) + 2.0 * self.scene.contact_slop;
        for ka in 0..ca.len().max(1) {
            let child_a;
            let pa = match ca.get(ka) {
                Some(c) => {
                    child_a = compound::bounded(compound::child_body(a, c));
                    &child_a
                }
                None => a,
            };
            for kb in 0..cb.len().max(1) {
                let child_b;
                let pb = match cb.get(kb) {
                    Some(c) => {
                        child_b = compound::bounded(compound::child_body(b, c));
                        &child_b
                    }
                    None => b,
                };
                let ((lo, hi), (bl, bh)) = (pa.cached_bounds, pb.cached_bounds);
                // Rounding of the bounds' own magnitudes, as the broad phase pads them.
                let slack = pad
                    + 64.0
                        * Real::EPSILON
                        * (1.0
                            + lo.abs().max(hi.abs()).max_component()
                            + bl.abs().max(bh.abs()).max_component());
                if !slack.is_finite()
                    || (lo.0 - slack <= bh.0
                        && bl.0 <= hi.0 + slack
                        && lo.1 - slack <= bh.1
                        && bl.1 <= hi.1 + slack
                        && lo.2 - slack <= bh.2
                        && bl.2 <= hi.2 + slack)
                {
                    self.pair(
                        &mut PairGeometry::Direct,
                        [i, j],
                        pa,
                        pb,
                        report,
                        regenerate,
                        &mut |mut m: contact::Manifold| {
                            for p in &mut m.points {
                                p.ra = compound::parent_arm(pa, a, p.ra);
                                p.rb = compound::parent_arm(pb, b, p.rb);
                            }
                            emit(m);
                        },
                    )?;
                }
            }
        }
        Ok(())
    }
}

/// Whether a pair needs [`Narrow::compound_pair`].
pub(super) fn compound_pair(a: &Body, b: &Body) -> bool {
    matches!(a.shape, Shape::Compound(_)) || matches!(b.shape, Shape::Compound(_))
}
