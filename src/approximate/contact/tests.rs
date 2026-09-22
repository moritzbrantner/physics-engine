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
