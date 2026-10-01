use physics_engine::approximate::Real;
use physics_engine::approximate::{MassPropertiesError, Shape, Vector};

fn close(actual: Real, expected: Real) {
    assert!(
        (actual - expected).abs() <= 2e-14 * expected.abs().max(1e-30),
        "{actual:e} != {expected:e}"
    );
}

#[cfg_attr(test, test)]
pub fn sphere_and_zero_skeleton_capsule_share_uniform_products() {
    for radius in [1e-6, 0.25, 1.0, 1e6, 1e11] {
        for mass in [1e-6, 2.0, 1e11] {
            let sphere = Shape::Sphere(radius).mass_properties(mass).unwrap();
            let capsule = Shape::capsule(0.0, radius).mass_properties(mass).unwrap();
            assert_eq!(sphere, capsule);
            close(
                sphere.volume,
                4.0 * (std::f64::consts::PI as Real) * radius.powi(3) / 3.0,
            );
            assert_eq!(sphere.local_center_of_mass, Vector::ZERO);
            for axis in 0..3 {
                close(
                    sphere.local_inertia[axis][axis],
                    0.4 * mass * radius * radius,
                );
            }
        }
    }
}

#[cfg_attr(test, test)]
pub fn capsule_tensor_matches_cylinder_and_hemisphere_mass_integration() {
    for (h, r) in [(0.001, 1.0), (1.0, 0.01), (3.0, 2.0), (1e6, 1e-6)] {
        let mass = 7.0;
        let properties = Shape::capsule(h, r).mass_properties(mass).unwrap();
        let cylinder_volume = (std::f64::consts::PI as Real) * r * r * 2.0 * h;
        let sphere_volume = 4.0 * (std::f64::consts::PI as Real) * r * r * r / 3.0;
        let cylinder_mass = mass * cylinder_volume / (cylinder_volume + sphere_volume);
        let cap_mass = mass * sphere_volume / (cylinder_volume + sphere_volume);
        let hemisphere_centroid = h + 3.0 * r / 8.0;
        // Radial hemisphere COM inertia plus the parallel-axis displacement.
        let radial = cylinder_mass * (3.0 * r * r + 4.0 * h * h) / 12.0
            + cap_mass * (83.0 * r * r / 320.0 + hemisphere_centroid * hemisphere_centroid);
        let axial = cylinder_mass * r * r / 2.0 + cap_mass * 2.0 * r * r / 5.0;
        close(properties.volume, cylinder_volume + sphere_volume);
        assert_eq!(properties.local_center_of_mass, Vector::ZERO);
        close(properties.local_inertia[0][0], radial);
        close(properties.local_inertia[1][1], axial);
        close(properties.local_inertia[2][2], radial);
        assert_eq!(properties.local_inertia[0][1], 0.0);
    }
}

#[cfg_attr(test, test)]
pub fn box_and_wedge_report_com_and_complete_symmetric_tensors() {
    let mass = 9.0;
    let cuboid = Shape::Box(Vector(1.0, 2.0, 3.0))
        .mass_properties(mass)
        .unwrap();
    assert_eq!(cuboid.volume, 48.0);
    assert_eq!(cuboid.local_center_of_mass, Vector::ZERO);
    let expected = [[39.0, 0.0, 0.0], [0.0, 30.0, 0.0], [0.0, 0.0, 15.0]];
    for (actual, expected) in cuboid
        .local_inertia
        .iter()
        .flatten()
        .zip(expected.iter().flatten())
    {
        close(*actual, *expected);
    }
    let wedge = Shape::wedge(Vector(3.0, 6.0, 2.0))
        .mass_properties(mass)
        .unwrap();
    assert_eq!(wedge.volume, 144.0);
    assert_eq!(wedge.local_center_of_mass, Vector(-1.0, -2.0, 0.0));
    close(wedge.local_inertia[0][0], 84.0);
    close(wedge.local_inertia[1][1], 30.0);
    close(wedge.local_inertia[2][2], 90.0);
    close(wedge.local_inertia[0][1], 18.0);
    assert_eq!(wedge.local_inertia[0][1], wedge.local_inertia[1][0]);
    assert_eq!(wedge.local_inertia[0][2], 0.0);
    assert_eq!(wedge.local_inertia[1][2], 0.0);
    // Positive angular energy for axes and coupled directions, about the reported COM.
    for omega in [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, -1.0, 1.0],
    ] {
        let product: Real = (0..3)
            .flat_map(|i| (0..3).map(move |j| omega[i] * wedge.local_inertia[i][j] * omega[j]))
            .sum();
        assert!(product > 0.0);
    }
}

#[cfg_attr(test, test)]
pub fn scaling_and_mass_changes_have_the_declared_units() {
    let base = Shape::capsule(3.0, 2.0).mass_properties(5.0).unwrap();
    for scale in [(1e-6 as Real), 10.0, 1e10] {
        let scaled = Shape::capsule(3.0 * scale, 2.0 * scale)
            .mass_properties(15.0)
            .unwrap();
        close(scaled.volume, base.volume * scale.powi(3));
        assert_eq!(scaled.mass, 15.0);
        for i in 0..3 {
            for j in 0..3 {
                close(
                    scaled.local_inertia[i][j],
                    base.local_inertia[i][j] * 3.0 * scale * scale,
                );
            }
        }
    }
}

#[cfg_attr(test, test)]
pub fn invalid_mass_and_shape_products_fail_at_the_boundary() {
    for mass in [0.0, -1.0, 1e-7, 1e12, Real::NAN, Real::INFINITY] {
        assert_eq!(
            Shape::capsule(1.0, 1.0).mass_properties(mass),
            Err(MassPropertiesError::InvalidMass)
        );
    }
    for shape in [
        Shape::Sphere(0.0),
        Shape::Sphere(Real::NAN),
        Shape::capsule(-1.0, 1.0),
        Shape::capsule(1e12, 1.0),
        Shape::capsule(1.0, 1e-7),
        Shape::Box(Vector(1.0, Real::INFINITY, 1.0)),
        Shape::wedge(Vector(1.0, -1.0, 1.0)),
    ] {
        assert_eq!(
            shape.mass_properties(1.0),
            Err(MassPropertiesError::InvalidDimensions)
        );
    }
    assert!(Shape::capsule(0.0, 1e-6).mass_properties(1e-6).is_ok());
    assert!(
        Shape::wedge(Vector(9e11, 1e-6, 9e11))
            .mass_properties(9e11)
            .is_ok()
    );
}
