//! Vertex-by-vertex SAT oracle retained for exact kernel optimization parity.

use super::*;
use std::{hint::black_box, time::Instant};

fn shapes() -> Vec<OrientedBox3d> {
    let mut shapes = Vec::new();
    let mut state = 0x62ac_450e_u32;
    let mut next = || {
        state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        (state >> 1) as i32
    };
    for index in 0..512 {
        let center = Vec3i::new(next() % 200 - 100, next() % 200 - 100, next() % 200 - 100);
        let half_extents = Vec3i::new(next() % 100 + 1, next() % 100 + 1, next() % 100 + 1);
        let orientation = match index % 4 {
            0 => Orientation3d::IDENTITY,
            1 => Orientation3d::new(0, 0, next(), next()),
            _ => Orientation3d::new(next(), -next(), next(), next()),
        };
        shapes.push(OrientedBox3d::new(center, half_extents, orientation));
    }
    for center in [Vec3i::ZERO, Vec3i::new(i32::MAX, i32::MIN, 0)] {
        for half_extents in [
            Vec3i::new(1, 1, 1),
            Vec3i::new(1, 0, 1),
            Vec3i::new(i32::MAX, i32::MAX, i32::MAX),
            Vec3i::new(900_000_000, 700_000_000, 600_000_000),
        ] {
            for orientation in [
                Orientation3d::IDENTITY,
                Orientation3d::new(0, 0, 0, 0),
                Orientation3d::new(123, -456, 789, 321),
            ] {
                shapes.push(OrientedBox3d::new(center, half_extents, orientation));
            }
        }
    }
    shapes
}

#[test]
fn contact_seeds_and_errors_match_vertex_sat() {
    let shapes = shapes();
    for (index, left) in shapes.iter().copied().enumerate() {
        for right in [
            left,
            shapes[(index + 1) % shapes.len()],
            shapes[(index * 31) % shapes.len()],
        ] {
            for (left, right) in [(left, right), (right, left)] {
                assert_eq!(
                    obb_contact_seed(left, right),
                    vertex_contact_reference(&PreparedObb3d::new(left), &PreparedObb3d::new(right)),
                    "contact or error changed for {left:?}, {right:?}"
                );
            }
        }
    }
}

#[test]
fn touching_and_containment_keep_complete_contact_evidence() {
    let cube = OrientedBox3d::new(Vec3i::ZERO, Vec3i::new(10, 10, 10), Orientation3d::IDENTITY);
    for center in [
        Vec3i::ZERO,
        Vec3i::new(20, 0, 0),
        Vec3i::new(20, 20, 0),
        Vec3i::new(20, 20, 20),
        Vec3i::new(-20, -20, -20),
        Vec3i::new(21, 0, 0),
    ] {
        for half_extents in [cube.half_extents, Vec3i::new(1, 1, 1)] {
            let right = OrientedBox3d::new(center, half_extents, cube.orientation);
            assert_eq!(
                obb_contact_seed(cube, right),
                vertex_contact_reference(&PreparedObb3d::new(cube), &PreparedObb3d::new(right))
            );
        }
    }
}

#[test]
fn box_projections_match_all_eight_vertices() {
    // SAT axes are cross products of differences between i32 vertices, bounded by 65 bits.
    // Include that upper bound as well as the small axes common in aligned scenes.
    for shape in shapes() {
        let Ok(vertices) = oriented_box_vertices(shape) else {
            continue;
        };
        for axis in [
            [1, 0, 0],
            [0, -1, 0],
            [3, -5, 7],
            [1_234_567, -7_654_321, 3_456_789],
            [i128::from(i64::MAX), 1, -1],
            [(1_i128 << 65) - 1, -(1_i128 << 65) + 1, 1_i128 << 64],
        ] {
            assert_eq!(
                projection(&vertices, axis),
                projection_wide_reference(&vertices, axis)
            );
        }
    }
}

#[test]
fn projection_work_ratchet() {
    let vertices = oriented_box_vertices(shapes()[7]).unwrap();
    let axis = [3, -5, 7];
    crate::performance_ratchet::PROJECTION_DOTS.set(0);
    let actual = projection(&vertices, axis).unwrap();
    let dots = crate::performance_ratchet::PROJECTION_DOTS.get();
    assert_eq!(actual, projection_wide_reference(&vertices, axis).unwrap());
    assert_eq!(dots, 4);
    crate::performance_ratchet::record(
        "sat-projection",
        &[("dot_products", dots)],
        &[("projections", 1)],
        &[],
    );
}

#[test]
#[ignore = "release-mode prepared SAT kernel benchmark"]
fn prepared_contact_benchmark() {
    let prepared: Vec<_> = shapes()
        .into_iter()
        .take(512)
        .map(PreparedObb3d::new)
        .collect();
    let mut samples = [Vec::new(), Vec::new()];
    let mut contacts = [0; 2];
    for trial in 0..15 {
        // Alternate order so both paths share the same warm process and changing host load.
        for candidate in [trial % 2, 1 - trial % 2] {
            let start = Instant::now();
            contacts[candidate] = if candidate == 0 {
                measure_contacts(&prepared, obb_contact_seed_prepared)
            } else {
                measure_contacts(&prepared, vertex_contact_reference)
            };
            samples[candidate].push(start.elapsed());
        }
        assert_eq!(contacts[0], contacts[1]);
    }
    for samples in &mut samples {
        samples.sort_unstable();
    }
    crate::performance_ratchet::record(
        "sat-prepared",
        &[],
        &[
            ("queries", (40 * prepared.len()) as u64),
            ("contacts", contacts[0] as u64),
        ],
        &[
            ("median_ms", samples[0][7].as_secs_f64() * 1_000.0),
            ("reference_median_ms", samples[1][7].as_secs_f64() * 1_000.0),
        ],
    );
    println!(
        "prepared SAT: queries={}, contacts={}, candidate={:?}, eight_vertex_baseline={:?}",
        40 * prepared.len(),
        contacts[0],
        samples[0][7],
        samples[1][7]
    );
}

fn measure_contacts(
    prepared: &[PreparedObb3d],
    query: impl Fn(
        &PreparedObb3d,
        &PreparedObb3d,
    ) -> Result<Option<ObbContactSeed3d>, OrientedBoxError3d>,
) -> usize {
    let mut contacts = 0;
    for _ in 0..40 {
        for index in 0..prepared.len() {
            let left = black_box(&prepared[index]);
            let right = black_box(&prepared[(index + 1) % prepared.len()]);
            contacts += usize::from(black_box(query(left, right).unwrap()).is_some());
        }
    }
    contacts
}

// The pre-optimization projection, including its existing bounded multiplication fast path.
fn vertex_projection_reference(
    vertices: &[Vec3i; 8],
    axis: [i128; 3],
) -> Result<(i128, i128), OrientedBoxError3d> {
    let first = dot_position(vertices[0], axis)?;
    vertices[1..]
        .iter()
        .try_fold((first, first), |range, vertex| {
            let value = dot_position(*vertex, axis)?;
            Ok((range.0.min(value), range.1.max(value)))
        })
}

fn vertex_contact_reference(
    left: &PreparedObb3d,
    right: &PreparedObb3d,
) -> Result<Option<ObbContactSeed3d>, OrientedBoxError3d> {
    let left_vertices = left.vertices?;
    let right_vertices = right.vertices?;
    let left_edges = left.edges?;
    let right_edges = right.edges?;
    let left_faces = left.faces?;
    let right_faces = right.faces?;

    let mut candidate_axes = Vec::with_capacity(MAX_SAT_AXES);
    for (index, direction) in left_faces.into_iter().enumerate() {
        push_required_axis(
            &mut candidate_axes,
            direction,
            ObbAxisFeature3d::LeftFace(
                u8::try_from(index).map_err(|_| OrientedBoxError3d::ArithmeticOverflow)?,
            ),
        )?;
    }
    for (index, direction) in right_faces.into_iter().enumerate() {
        push_required_axis(
            &mut candidate_axes,
            direction,
            ObbAxisFeature3d::RightFace(
                u8::try_from(index).map_err(|_| OrientedBoxError3d::ArithmeticOverflow)?,
            ),
        )?;
    }
    for (left_index, left_edge) in left_edges.into_iter().enumerate() {
        for (right_index, right_edge) in right_edges.into_iter().enumerate() {
            push_optional_axis(
                &mut candidate_axes,
                cross(left_edge, right_edge)?,
                ObbAxisFeature3d::EdgeEdge {
                    left_axis: u8::try_from(left_index)
                        .map_err(|_| OrientedBoxError3d::ArithmeticOverflow)?,
                    right_axis: u8::try_from(right_index)
                        .map_err(|_| OrientedBoxError3d::ArithmeticOverflow)?,
                },
            )?;
        }
    }

    if candidate_axes.len() < 3 || candidate_axes.len() > MAX_SAT_AXES {
        return Err(OrientedBoxError3d::DegenerateGeometry);
    }

    let mut best: Option<EvaluatedAxis> = None;
    let mut minimum_axis_ties = 0_u8;
    for seed in candidate_axes.iter().copied() {
        let left_projection = vertex_projection_reference(&left_vertices, seed.axis)?;
        let right_projection = vertex_projection_reference(&right_vertices, seed.axis)?;
        if left_projection.1 < right_projection.0 || right_projection.1 < left_projection.0 {
            return Ok(None);
        }

        let overlap_start = left_projection.0.max(right_projection.0);
        let overlap_end = left_projection.1.min(right_projection.1);
        let overlap = u128::try_from(
            overlap_end
                .checked_sub(overlap_start)
                .ok_or(OrientedBoxError3d::ArithmeticOverflow)?,
        )
        .map_err(|_| OrientedBoxError3d::ArithmeticOverflow)?;
        let length_squared = squared_length(seed.axis)?;
        let evaluated = EvaluatedAxis {
            seed,
            overlap,
            length_squared,
        };

        match best {
            None => {
                best = Some(evaluated);
                minimum_axis_ties = 1;
            }
            Some(current) => match compare_normalized_overlap(evaluated, current)? {
                Ordering::Less => {
                    best = Some(evaluated);
                    minimum_axis_ties = 1;
                }
                Ordering::Equal => {
                    minimum_axis_ties = minimum_axis_ties
                        .checked_add(1)
                        .ok_or(OrientedBoxError3d::ArithmeticOverflow)?;
                }
                Ordering::Greater => {}
            },
        }
    }

    let best = best.ok_or(OrientedBoxError3d::DegenerateGeometry)?;
    let contact_axis = orient_toward_right(left.shape.center, right.shape.center, best.seed.axis)?;
    let tested_axes =
        u8::try_from(candidate_axes.len()).map_err(|_| OrientedBoxError3d::ArithmeticOverflow)?;
    Ok(Some(ObbContactSeed3d {
        axis: contact_axis,
        overlap_numerator: best.overlap,
        axis_length_squared: best.length_squared,
        feature: best.seed.feature,
        left_support_mask: support_mask(&left_vertices, contact_axis, true)?,
        right_support_mask: support_mask(&right_vertices, contact_axis, false)?,
        tested_axes,
        minimum_axis_ties,
    }))
}
