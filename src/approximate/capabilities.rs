/// Versioned JSON capability ledger for this floating world and its primitive pairs.
///
/// Includes API availability, motion restrictions and partial evidence status. This is
/// read-only metadata, not collision configuration; consumers parse it outside physics
/// hot paths. Schema version 1 is documented in `docs/primitive-capabilities.md`.
pub const PRIMITIVE_CAPABILITIES_JSON: &str =
    include_str!("../../docs/primitive-capabilities.json");

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;

    use super::super::{
        Body, Quaternion, Report, Shape, Vector as V, contact, primitive::PrimitivePair,
    };
    use super::PRIMITIVE_CAPABILITIES_JSON;
    use crate::BodyId;
    use serde_json::Value;

    #[test]
    fn ledger_classification_and_manifold_limits_match_the_canonical_pairs() {
        let data: Value = serde_json::from_str(PRIMITIVE_CAPABILITIES_JSON).unwrap();
        assert_eq!(
            data["pairs"].as_array().unwrap().len(),
            PrimitivePair::ALL.len()
        );
        for pair in PrimitivePair::ALL {
            let (classification, limit, search) = match pair {
                PrimitivePair::SphereSphere | PrimitivePair::SphereBox => {
                    ("analytic-specialized", 1, "analytic-conservative")
                }
                PrimitivePair::SphereCapsule
                | PrimitivePair::BoxCapsule
                | PrimitivePair::CapsuleCapsule => ("analytic-specialized", 1, "bounded-iterative"),
                PrimitivePair::SphereWedge | PrimitivePair::CapsuleWedge => {
                    ("fixed-topology-feature", 1, "bounded-iterative")
                }
                PrimitivePair::BoxBox => ("fixed-topology-sat", 4, "fixed-axis-sat"),
                PrimitivePair::BoxWedge | PrimitivePair::WedgeWedge => {
                    ("fixed-topology-sat", 1, "fixed-axis-sat")
                }
            };
            let cell = &data["pairs"][pair.index()];
            assert_eq!(cell["dispatchIndex"], pair.index());
            assert_eq!(cell["classification"], classification);
            assert_eq!(cell["manifoldPointLimit"], limit);
            assert_eq!(cell["translationSearch"], search);
            assert_eq!(
                cell["shapeCastSearch"],
                if search == "fixed-axis-sat" {
                    search
                } else {
                    "bounded-iterative"
                }
            );
        }
    }

    #[test]
    fn advertised_work_fields_exist_in_public_reports() {
        let data: Value = serde_json::from_str(PRIMITIVE_CAPABILITIES_JSON).unwrap();
        let report = Report::default();
        let geometry = report.geometry;
        let fields = [
            (
                "geometry.specialized_pair_dispatches",
                geometry.specialized_pair_dispatches.iter().sum(),
            ),
            (
                "geometry.generic_fallback_calls",
                geometry.generic_fallback_calls,
            ),
            ("geometry.support_evaluations", geometry.support_evaluations),
            ("geometry.sat_axes_tested", geometry.sat_axes_tested),
            ("geometry.clip_passes", geometry.clip_passes),
            ("geometry.primitive_queries", geometry.primitive_queries),
            (
                "geometry.primitive_axes_tested",
                geometry.primitive_axes_tested,
            ),
            (
                "geometry.primitive_vertex_tests",
                geometry.primitive_vertex_tests,
            ),
            (
                "geometry.primitive_segment_distance_evaluations",
                geometry.primitive_segment_distance_evaluations,
            ),
            (
                "geometry.primitive_segment_feature_tests",
                geometry.primitive_segment_feature_tests,
            ),
            ("geometry.manifold_candidates", geometry.manifold_candidates),
            (
                "geometry.primitive_sweep_iterations",
                geometry.primitive_sweep_iterations,
            ),
            (
                "geometry.primitive_sweep_failures",
                geometry.primitive_sweep_failures,
            ),
            ("geometry.retained_bytes", geometry.retained_bytes),
            (
                "bookkeeping.scratch_growths",
                report.bookkeeping.scratch_growths,
            ),
            (
                "bookkeeping.scratch_retained_bytes",
                report.bookkeeping.scratch_retained_bytes,
            ),
        ];
        let expected: BTreeSet<_> = fields.iter().map(|(name, _)| *name).collect();
        let advertised: BTreeSet<_> = data["workCounters"]
            .as_array()
            .unwrap()
            .iter()
            .map(|field| field.as_str().unwrap())
            .collect();
        assert_eq!(
            advertised.len(),
            data["workCounters"].as_array().unwrap().len()
        );
        assert_eq!(advertised, expected);
    }

    #[test]
    fn advertised_conservative_bounds_enclose_rotated_surface_extrema() {
        let data: Value = serde_json::from_str(PRIMITIVE_CAPABILITIES_JSON).unwrap();
        assert_eq!(data["capabilities"]["bounds"], "internal-conservative");
        for scale in [1e-6, 1.0, 1e6] {
            for shape in [
                Shape::Sphere(scale),
                Shape::Box(V(scale, 2.0 * scale, 3.0 * scale)),
                Shape::capsule(2.0 * scale, scale),
                Shape::wedge(V(scale, 2.0 * scale, 3.0 * scale)),
            ] {
                let mut body = Body::new(
                    BodyId(1),
                    shape,
                    V(-11.0 * scale, 4.0 * scale, 3.0 * scale),
                    0.0,
                );
                body.orientation = Quaternion(0.2, 0.3, 0.1, 0.9).normalized();
                let (low, high) = contact::bounds(&body);
                let mut points = Vec::new();
                match shape {
                    Shape::Sphere(radius) => {
                        for direction in [V::X, V::Y, V::Z] {
                            points.extend([
                                body.position + direction * radius,
                                body.position - direction * radius,
                            ]);
                        }
                    }
                    Shape::Capsule {
                        half_segment,
                        radius,
                    } => {
                        for half in [-half_segment, half_segment] {
                            let center = body.position + body.orientation.rotate(V::Y * half);
                            for direction in [V::X, V::Y, V::Z] {
                                points.extend([
                                    center + direction * radius,
                                    center - direction * radius,
                                ]);
                            }
                        }
                    }
                    Shape::Box(half) | Shape::Wedge(half) => {
                        for x in [-half.0, half.0] {
                            for y in [-half.1, half.1] {
                                if matches!(shape, Shape::Wedge(_)) && x > 0.0 && y > 0.0 {
                                    continue;
                                }
                                for z in [-half.2, half.2] {
                                    points
                                        .push(body.position + body.orientation.rotate(V(x, y, z)));
                                }
                            }
                        }
                    }
                }
                for point in points {
                    for axis in 0..3 {
                        assert!(
                            low.at(axis) <= point.at(axis) && point.at(axis) <= high.at(axis),
                            "scale={scale} shape={shape:?} axis={axis} low={low:?} high={high:?} point={point:?}"
                        );
                    }
                }
            }
        }
    }
}
