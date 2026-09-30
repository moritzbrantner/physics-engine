# Uniform shape mass properties

`approximate::Shape::mass_properties(mass)` returns checked uniform-solid volume,
mass, local center of mass and the complete symmetric local inertia tensor for
sphere, box, local-Y capsule and wedge shapes. It uses `numeric::Scalar` (f64),
with volume in scene length cubed and inertia in mass times scene length squared.
The tensor is about the returned COM, with rows/columns following local X/Y/Z.

Dimensions and positive mass use floating bodies' `[1e-6, 1e12)` range. Capsule
half-segment permits zero and remains below 1e12. Invalid dimensions/mass return
separate `MassPropertiesError` variants; unrepresentable arithmetic is explicit.
The query does not mutate a body or world. Fixed-body zero mass remains an engine
participation policy rather than a uniform-solid mass input.

```rust
use physics_engine::approximate::{Shape, Vector};
let capsule = Shape::capsule(2.0, 0.5).mass_properties(3.0)?;
let wedge = Shape::wedge(Vector(3.0, 6.0, 2.0)).mass_properties(9.0)?;
assert_eq!(wedge.local_center_of_mass, Vector(-1.0, -2.0, 0.0));
assert!(wedge.local_inertia[0][1] > 0.0);
# Ok::<(), physics_engine::approximate::MassPropertiesError>(())
```

| Existing shape | Volume/COM | Full COM inertia | Solver-owned rotation |
| --- | --- | --- | --- |
| Sphere | Available; centered | Available; isotropic | Available |
| Box | Available; centered | Available; diagonal | Available |
| Capsule | Available; centered | Available; diagonal, includes both caps | Available |
| Wedge | Available; offset `[-half_x/3,-half_y/3,0]` | Available; nonzero XY entries | Rotation locked |

Rust-kernels owns domain-neutral volume, centroid and normalized second central
moments for the existing shape authority. Physics Engine scales those products
by mass: diagonal inertia sums the other two central moments; off-diagonal inertia
negates the central product. Direct diagonal sums avoid cancellation for thin
shapes. No new primitive representation or geometry authority is introduced.

For wedges, the COM is offset from the existing bounding-box pose origin. These
products support inspection and consumer calculations; they do not migrate body
poses, contact lever arms or world tensors. Dynamic wedges remain rotation locked
until that solver contract is implemented. Existing solver coefficients and
physical evolution are unchanged by this API slice.

Five public fixtures run natively and on WASM across three replays, covering
sphere limits, capsule cylinder/hemisphere mass decomposition and parallel-axis
terms, box/wedge tensors, units/scaling and invalid boundaries. A native unit
fixture compares public inertia against existing sphere/box/capsule solver inverse
coefficients across dimensional and mass scales using explicit relative tolerance.
Upstream tests independently integrate capsule cross-sections and wedge tetrahedra
and ratchet 4096 allocation-free calls. Queries have bounded scalar work and no
scratch. This completes the public mass products requested by #173, but the full
contact-pair independent reference/work acceptance remains separate. Wedge COM
pose migration and cylinder/cone primitives in #176 remain open.

Run `cargo test --test mass_properties` and
`cargo test public_inertia_agrees_with_existing_solver_coefficients --lib`.
The Pages build executes a separate `mass-properties-contract` WASM acceptance
module; the module is not copied to the shipped demo.
