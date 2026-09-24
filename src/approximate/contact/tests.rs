use super::*;

// Independent pre-change allocating clipper; this is a test oracle, not a production path.
fn reference_clip(input: &[V], n: V, limit: Scalar) -> Vec<V> {
    let mut out = Vec::with_capacity(8);
    if input.is_empty() {
        return out;
    }
    let mut a = *input.last().unwrap();
    let mut da = a.dot(n) - limit;
    for &b in input {
        let db = b.dot(n) - limit;
        if (da <= 0.0) != (db <= 0.0) {
            out.push(a + (b - a) * (da / (da - db)));
        }
        if db <= 0.0 {
            out.push(b);
        }
        a = b;
        da = db;
    }
    out
}

#[test]
fn scratch_clipping_preserves_vertices_and_reuses_capacity() {
    let mut scratch = ClipScratch::default();
    let mut seed = 97_u64;
    let mut value = || {
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
        (seed >> 32) as f64 / u32::MAX as f64 * 2.0 - 1.0
    };
    let mut inputs = vec![vec![], vec![V::ZERO], vec![V::ZERO; 4]];
    for _ in 0..600 {
        let q = super::super::Quaternion(value(), value(), value(), value()).normalized();
        let origin = V(value() * 10.0, value() * 3.0, value() * 10.0);
        let u = q.rotate(V::X) * (value().abs() + 0.1);
        let v = q.rotate(V::Z) * (value().abs() + 0.1);
        inputs.push(vec![
            origin + u + v,
            origin - u + v,
            origin - u - v,
            origin + u - v,
        ]);
    }
    let planes = [(V::X, 5.0), (-V::X, 5.0), (V::Z, 5.0), (-V::Z, 5.0)];
    let mut capacity_after_warmup = 0;
    for pass in 0..2 {
        for input in &inputs {
            scratch.polygon.clone_from(input);
            let mut expected = input.clone();
            for (n, limit) in planes {
                clip_into(&scratch.polygon, n, limit, &mut scratch.clipped);
                std::mem::swap(&mut scratch.polygon, &mut scratch.clipped);
                expected = reference_clip(&expected, n, limit);
                let bits = |vertices: &[V]| {
                    vertices
                        .iter()
                        .flat_map(|v| [v.0.to_bits(), v.1.to_bits(), v.2.to_bits()])
                        .collect::<Vec<_>>()
                };
                assert_eq!(bits(&scratch.polygon), bits(&expected));
            }
        }
        if pass == 0 {
            capacity_after_warmup = scratch.retained_bytes();
        } else {
            assert_eq!(scratch.retained_bytes(), capacity_after_warmup);
        }
    }
    assert!(capacity_after_warmup > 0);
}

#[test]
fn inline_points_preserve_the_original_bounded_selection_and_iteration_order() {
    for count in 0..20 {
        let input = (0..count)
            .map(|i| Point {
                ra: V(i as f64, -0.0, 2.0),
                rb: V(1.0, i as f64, 3.0),
                separation: -(i as f64),
            })
            .collect::<Vec<_>>();
        let expected = if count > 4 {
            (0..4).map(|i| input[i * count / 4]).collect::<Vec<_>>()
        } else {
            input.clone()
        };
        let mut actual = Points::selected(&input);
        assert_eq!(actual.len(), expected.len());
        assert_eq!(actual.clone().into_iter().collect::<Vec<_>>(), expected);
        assert_eq!((&actual).into_iter().copied().collect::<Vec<_>>(), expected);
        for point in &mut actual {
            point.separation += 1.0;
        }
        for (actual, expected) in (&actual).into_iter().zip(expected) {
            assert_eq!(actual.ra.1.to_bits(), expected.ra.1.to_bits());
            assert_eq!(actual.separation, expected.separation + 1.0);
        }
    }
}

fn primitive_shape(kind: primitive::PrimitiveKind) -> Shape {
    match kind {
        primitive::PrimitiveKind::Sphere => Shape::Sphere(1.0),
        primitive::PrimitiveKind::Box => Shape::Box(V(1.0, 0.8, 1.2)),
        primitive::PrimitiveKind::Capsule => Shape::capsule(0.7, 0.6),
        primitive::PrimitiveKind::Wedge => Shape::wedge(V(1.2, 0.9, 1.1)),
    }
}

fn pair_bodies(pair: primitive::PrimitivePair) -> (Body, Body) {
    let (left, right) = pair.kinds();
    let mut a = Body::new(
        crate::BodyId(100 + pair.index() as u64 * 2),
        primitive_shape(left),
        V::ZERO,
        1.0,
    );
    let mut b = Body::new(
        crate::BodyId(101 + pair.index() as u64 * 2),
        primitive_shape(right),
        V(1.0, 0.2, -0.1),
        1.0,
    );
    a.orientation = super::super::Quaternion(0.0, 0.0, 0.13052619222005157, 0.9914448613738104);
    b.orientation = super::super::Quaternion(0.0, 0.25881904510252074, 0.0, 0.9659258262890683);
    if matches!(a.shape, Shape::Wedge(_)) {
        a.rotation_locked = true;
    }
    if matches!(b.shape, Shape::Wedge(_)) {
        b.rotation_locked = true;
    }
    (a, b)
}

fn separations(manifold: &Manifold) -> Vec<Scalar> {
    let mut values = manifold
        .points
        .clone()
        .into_iter()
        .map(|point| point.separation)
        .collect::<Vec<_>>();
    values.sort_by(Scalar::total_cmp);
    values
}

#[test]
fn canonical_dispatch_matches_pre_matrix_reference_for_every_current_pair() {
    for pair in primitive::PrimitivePair::ALL {
        let (a, b) = pair_bodies(pair);
        let tolerance = 1e-9 * (1.0 + a.shape.radius() + b.shape.radius());
        for (left, right) in [(&a, &b), (&b, &a)] {
            let mut production_work = GeometryStats::default();
            let production = current_counted(left, right, 0.02, &mut production_work);
            let mut reference_work = GeometryStats::default();
            let reference = reference_current_counted(left, right, 0.02, &mut reference_work);

            assert_eq!(
                production.is_some(),
                reference.is_some(),
                "presence mismatch for {pair:?}"
            );
            if let (Some(production), Some(reference)) = (production, reference) {
                assert!(
                    (production.normal - reference.normal).length() <= tolerance,
                    "normal mismatch for {pair:?}: {:?} vs {:?}",
                    production.normal,
                    reference.normal
                );
                let production_sep = separations(&production);
                let reference_sep = separations(&reference);
                assert_eq!(production_sep.len(), reference_sep.len(), "{pair:?}");
                for (actual, expected) in production_sep.into_iter().zip(reference_sep) {
                    assert!(
                        (actual - expected).abs() <= tolerance,
                        "separation mismatch for {pair:?}: {actual} vs {expected}"
                    );
                }
            }
            assert_eq!(
                production_work.specialized_pair_dispatches[pair.index()],
                1,
                "{pair:?}"
            );
            assert_eq!(production_work.generic_fallback_calls, 0, "{pair:?}");
        }
    }
}

#[test]
fn reversing_primitive_pair_flips_contact_orientation_without_changing_separation() {
    for pair in primitive::PrimitivePair::ALL {
        let (a, b) = pair_bodies(pair);
        let tolerance = 1e-9 * (1.0 + a.shape.radius() + b.shape.radius());
        let mut forward_work = GeometryStats::default();
        let forward = current_counted(&a, &b, 0.02, &mut forward_work)
            .unwrap_or_else(|| panic!("forward fixture must contact for {pair:?}"));
        let mut reverse_work = GeometryStats::default();
        let reverse = current_counted(&b, &a, 0.02, &mut reverse_work)
            .unwrap_or_else(|| panic!("reverse fixture must contact for {pair:?}"));

        assert!(
            (forward.normal + reverse.normal).length() <= tolerance,
            "normal orientation mismatch for {pair:?}"
        );
        let forward_sep = separations(&forward);
        let reverse_sep = separations(&reverse);
        assert_eq!(forward_sep.len(), reverse_sep.len(), "{pair:?}");
        for (left, right) in forward_sep.into_iter().zip(reverse_sep) {
            assert!(
                (left - right).abs() <= tolerance,
                "separation changed on reverse for {pair:?}: {left} vs {right}"
            );
        }
    }
}

#[test]
fn sphere_and_box_fast_paths_have_deterministic_work_budgets() {
    let scenarios = [
        primitive::PrimitivePair::SphereSphere,
        primitive::PrimitivePair::SphereBox,
        primitive::PrimitivePair::BoxBox,
    ];
    for pair in scenarios {
        let (a, b) = pair_bodies(pair);
        let mut work = GeometryStats::default();
        let manifold = current_counted(&a, &b, 0.02, &mut work);
        assert!(manifold.is_some(), "{pair:?}");
        assert_eq!(
            work.specialized_pair_dispatches[pair.index()],
            1,
            "{pair:?}"
        );
        assert_eq!(
            work.specialized_pair_dispatches.into_iter().sum::<u64>(),
            1,
            "{pair:?}"
        );
        assert_eq!(work.generic_fallback_calls, 0, "{pair:?}");
        assert_eq!(work.manifold_candidates, 1, "{pair:?}");
        assert_eq!(work.primitive_queries, 0, "{pair:?}");

        match pair {
            primitive::PrimitivePair::SphereSphere | primitive::PrimitivePair::SphereBox => {
                assert_eq!(work.sat_queries, 0, "{pair:?}");
                assert_eq!(work.sat_axes_tested, 0, "{pair:?}");
                assert_eq!(work.clip_passes, 0, "{pair:?}");
                assert_eq!(work.support_evaluations, 0, "{pair:?}");
            }
            primitive::PrimitivePair::BoxBox => {
                assert_eq!(work.sat_queries, 1);
                assert!(work.sat_axes_tested <= 15, "{work:?}");
                assert!(work.clip_passes <= 4, "{work:?}");
                assert!(work.support_evaluations <= 2, "{work:?}");
            }
            _ => unreachable!(),
        }
    }
}
