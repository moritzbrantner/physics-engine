use physics_engine::approximate::{Body, Shape, Vector as V};

// Independent geometric screening, not the engine's clipped response manifold.
pub fn penetration(a: &Body, b: &Body) -> f64 {
    let Shape::Box(ha) = a.shape else {
        panic!("box fixture")
    };
    let Shape::Box(hb) = b.shape else {
        panic!("box fixture")
    };
    let aa = a.orientation.axes();
    let bb = b.orientation.axes();
    let delta = b.position - a.position;
    let mut depth = f64::INFINITY;
    for axis in aa
        .into_iter()
        .chain(bb)
        .chain(aa.into_iter().flat_map(|u| bb.map(|v| u.cross(v))))
    {
        if axis.dot(axis) <= 1e-12 {
            continue;
        }
        let n = axis / axis.length();
        let extent = |basis: [V; 3], half: V| {
            basis[0].dot(n).abs() * half.0
                + basis[1].dot(n).abs() * half.1
                + basis[2].dot(n).abs() * half.2
        };
        let overlap = extent(aa, ha) + extent(bb, hb) - delta.dot(n).abs();
        if overlap < 0.0 {
            return 0.0;
        }
        depth = depth.min(overlap);
    }
    depth
}
