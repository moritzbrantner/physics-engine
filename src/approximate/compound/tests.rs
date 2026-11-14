use super::*;

fn close(a: Real, b: Real, tolerance: Real) -> bool {
    (a - b).abs() <= tolerance * (1.0 + a.abs().max(b.abs()))
}

#[test]
fn one_centred_box_is_the_box() {
    let shape = Shape::Box(Vector(0.2, 0.3, 0.4));
    let c = PreparedCompound::new(&[CompoundChild::new(shape, Vector::ZERO, 2.0)]).unwrap();
    assert_eq!(c.mass(), 2.0);
    assert_eq!(c.center_of_mass(), Vector::ZERO);
    assert_eq!(c.principal_orientation(), Quaternion::IDENTITY);
    let own = shape.local_inverse_inertia(2.0).unwrap();
    let mine = Shape::Compound(c.shape())
        .local_inverse_inertia(2.0)
        .unwrap();
    for (a, b) in [(own.0, mine.0), (own.1, mine.1), (own.2, mine.2)] {
        assert!(close(a, b, 1e-6), "{own:?} {mine:?}");
    }
}

#[test]
fn parallel_axis_moves_the_centre_and_adds_m_d_squared() {
    // A 1 kg and a 3 kg unit cube 2 m apart along X: the centre of mass is 0.5 m from the
    // heavy one; the Y and Z moments gain m d² for each.
    let cube = Shape::Box(Vector(0.5, 0.5, 0.5));
    let c = PreparedCompound::new(&[
        CompoundChild::new(cube, Vector(-1.0, 0.0, 0.0), 1.0),
        CompoundChild::new(cube, Vector(1.0, 0.0, 0.0), 3.0),
    ])
    .unwrap();
    assert_eq!(c.mass(), 4.0);
    assert!(close(c.center_of_mass().0, 0.5, 1e-7));
    let own = |m: f64| m * (0.25 + 0.25) / 3.0;
    let x = own(1.0) + own(3.0);
    let yz = x + 1.0 * 1.5 * 1.5 + 3.0 * 0.5 * 0.5;
    let i = c.principal_inertia();
    assert_eq!(c.principal_orientation(), Quaternion::IDENTITY);
    assert!(close(i.0, x as Real, 1e-6), "{i:?}");
    assert!(close(i.1, yz as Real, 1e-6), "{i:?}");
    assert!(close(i.2, yz as Real, 1e-6), "{i:?}");
    // Children are re-expressed about the centre of mass.
    assert!(close(c.children()[0].position.0, -1.5, 1e-7));
    assert!(close(c.children()[1].position.0, 0.5, 1e-7));
    assert!(c.shape().radius() >= 1.5 + 0.75f32.sqrt());
}

#[test]
fn an_off_diagonal_tensor_turns_the_body_frame_onto_its_principal_axes() {
    // Two spheres on the XY diagonal: the line between them is a principal axis.
    let ball = Shape::Sphere(0.1);
    let c = PreparedCompound::new(&[
        CompoundChild::new(ball, Vector(1.0, 1.0, 0.0), 1.0),
        CompoundChild::new(ball, Vector(-1.0, -1.0, 0.0), 1.0),
        CompoundChild::new(
            Shape::Box(Vector(0.1, 0.2, 0.3)),
            Vector(0.0, 0.0, 0.5),
            0.5,
        ),
    ])
    .unwrap();
    let q = c.principal_orientation();
    assert_ne!(q, Quaternion::IDENTITY);
    // The tensor rebuilt from the principal frame agrees with the authored one.
    let r = rotation(q);
    let d = c.principal_inertia();
    let diag: Matrix = [
        [f64::from(d.0), 0.0, 0.0],
        [0.0, f64::from(d.1), 0.0],
        [0.0, 0.0, f64::from(d.2)],
    ];
    let rebuilt = rotated(&r, &diag);
    let com = wide(c.center_of_mass());
    let mut authored: Matrix = [[0.0; 3]; 3];
    for child in c.authored_children() {
        let p = child.shape.mass_properties(child.mass).unwrap();
        let own = rotated(
            &rotation(child.orientation),
            &p.local_inertia.map(|row| row.map(f64::from)),
        );
        let x: [f64; 3] = std::array::from_fn(|k| wide(child.position)[k] - com[k]);
        let dd = x.iter().map(|v| v * v).sum::<f64>();
        let m = f64::from(child.mass);
        for i in 0..3 {
            for j in 0..3 {
                authored[i][j] += own[i][j] + m * (if i == j { dd } else { 0.0 } - x[i] * x[j]);
            }
        }
    }
    for i in 0..3 {
        for j in 0..3 {
            assert!(
                (rebuilt[i][j] - authored[i][j]).abs() <= 1e-5 * (1.0 + authored[i][j].abs()),
                "{rebuilt:?} vs {authored:?}"
            );
        }
    }
    // A child's world pose is the same from either frame.
    let (origin, turn) = (
        Vector(3.0, 1.0, -2.0),
        Quaternion(0.1, 0.7, 0.2, 0.6).normalized(),
    );
    let (position, orientation) = c.body_pose(origin, turn);
    for (body, authored) in c.children().iter().zip(c.authored_children()) {
        let from_body = position + orientation.rotate(body.position);
        let from_origin = origin + turn.rotate(authored.position);
        assert!(
            (from_body - from_origin).length() < 1e-5,
            "{from_body:?} {from_origin:?}"
        );
    }
    let (back, back_turn) = c.origin_pose(position, orientation);
    assert!((back - origin).length() < 1e-5);
    assert!(back_turn.rotation_to(turn).length() < 1e-5);
}

#[test]
fn rejects_invalid_children() {
    let cube = Shape::Box(Vector(0.5, 0.5, 0.5));
    assert_eq!(PreparedCompound::new(&[]), Err(Error::InvalidInput));
    for child in [
        CompoundChild::new(Shape::wedge(Vector(1.0, 1.0, 1.0)), Vector::ZERO, 1.0),
        CompoundChild::new(Shape::HalfSpace, Vector::ZERO, 1.0),
        CompoundChild::new(cube, Vector::ZERO, 0.0),
        CompoundChild::new(cube, Vector::ZERO, Real::NAN),
        CompoundChild::new(cube, Vector(Real::INFINITY, 0.0, 0.0), 1.0),
        CompoundChild::new(Shape::Box(Vector(0.0, 1.0, 1.0)), Vector::ZERO, 1.0),
        CompoundChild {
            orientation: Quaternion(0.0, 0.0, 0.0, 2.0),
            ..CompoundChild::new(cube, Vector::ZERO, 1.0)
        },
    ] {
        assert_eq!(
            PreparedCompound::new(&[child]),
            Err(Error::InvalidInput),
            "{child:?}"
        );
    }
    let many = vec![CompoundChild::new(cube, Vector::ZERO, 1.0); MAX_COMPOUND_CHILDREN + 1];
    assert_eq!(PreparedCompound::new(&many), Err(Error::InvalidInput));
}

#[test]
fn preparation_is_deterministic() {
    let parts: Vec<_> = (0..32)
        .map(|k| {
            let k = k as Real;
            CompoundChild {
                orientation: Quaternion(0.1 * k, 0.3, -0.2, 1.0).normalized(),
                ..CompoundChild::new(
                    Shape::Box(Vector(0.1, 0.1 + 0.01 * k, 0.1)),
                    Vector(0.2 * (k % 4.0), 0.2 * (k / 8.0).floor(), -0.2 * (k % 3.0)),
                    0.5 + 0.1 * k,
                )
            }
        })
        .collect();
    assert_eq!(
        PreparedCompound::new(&parts).unwrap(),
        PreparedCompound::new(&parts).unwrap()
    );
}
