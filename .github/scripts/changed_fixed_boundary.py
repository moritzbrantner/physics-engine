from pathlib import Path


def replace_once(path: str, old: str, new: str) -> None:
    target = Path(path)
    text = target.read_text()
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{path}: expected one occurrence, found {count}")
    target.write_text(text.replace(old, new, 1))


def replace_function(path: str, marker: str, replacement: str) -> None:
    target = Path(path)
    text = target.read_text()
    start = text.index(marker)
    brace = text.index("{", start)
    depth = 0
    end = None
    for index in range(brace, len(text)):
        char = text[index]
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                end = index + 1
                break
    if end is None:
        raise SystemExit(f"{path}: unterminated function for {marker!r}")
    target.write_text(text[:start] + replacement.rstrip() + text[end:])


# Production no longer owns a full-world fixed-boundary broad phase. Keep the previous helper only as a test oracle.
replace_once(
    "src/stabilized_rotating_world.rs",
    "    rigid_box_free_flight_sweep_bounds,\n    rotating_broad_phase::{RotatingBroadPhase3d, RotatingBroadPhaseError3d},\n    rotating_world::RotatingWorld3d as InnerRotatingWorld3d,\n",
    "    rigid_box_free_flight_sweep_bounds,\n    rotating_world::RotatingWorld3d as InnerRotatingWorld3d,\n",
)
replace_once(
    "src/stabilized_rotating_world.rs",
    "use std::collections::{BTreeMap, BTreeSet};\n\n",
    "use std::collections::{BTreeMap, BTreeSet};\n\n#[cfg(test)]\nuse crate::rotating_broad_phase::{RotatingBroadPhase3d, RotatingBroadPhaseError3d};\n\n",
)
replace_once(
    "src/stabilized_rotating_world.rs",
    "/// solver is preserved. A retained zero-time broad phase prunes separated fixed/dynamic pairs before exact\n/// OBB response work; it is refreshed after every position-correction pass, so a newly introduced contact\n/// is visible on the next pass without returning to an all-pairs world scan.\n",
    "/// solver is preserved. Fixed-boundary stabilization consumes the exact changed-body set from the step\n/// contract: each changed dynamic queries only its own current overlap neighborhood, and iterative projection\n/// re-queries only that candidate geometry. Unchanged dynamics are not cloned, indexed, or traversed.\n",
)
replace_once(
    "src/stabilized_rotating_world.rs",
    "    sleep_stable_time_q64: BTreeMap<BodyId, u128>,\n    fixed_boundary_broad_phase: RotatingBroadPhase3d,\n",
    "    sleep_stable_time_q64: BTreeMap<BodyId, u128>,\n    pending_fixed_boundary_body_ids: BTreeSet<BodyId>,\n",
)
replace_once(
    "src/stabilized_rotating_world.rs",
    "            sleep_stable_time_q64: BTreeMap::new(),\n            fixed_boundary_broad_phase: RotatingBroadPhase3d::default(),\n",
    "            sleep_stable_time_q64: BTreeMap::new(),\n            pending_fixed_boundary_body_ids: BTreeSet::new(),\n",
)

add_box = '''    pub fn add_box(&mut self, rigid_box: RigidBox3d) -> Result<(), RotatingWorldError3d> {
        let id = rigid_box.body.id;
        let kind = rigid_box.body.kind;
        let affected_dynamic_ids = if kind == BodyKind::Fixed {
            let layers = rigid_box.collision_layers();
            self.inner
                .overlap_query(rigid_box.oriented_box())?
                .into_iter()
                .filter(|other_id| {
                    self.inner.box_by_id(*other_id).is_some_and(|other| {
                        other.body.kind == BodyKind::Dynamic
                            && layers.collides_with(other.collision_layers())
                    })
                })
                .collect::<Vec<_>>()
        } else {
            Vec::new()
        };

        self.inner.add_box(rigid_box)?;
        if kind == BodyKind::Dynamic {
            self.pending_fixed_boundary_body_ids.insert(id);
        } else {
            self.pending_fixed_boundary_body_ids
                .extend(affected_dynamic_ids);
            self.wake_all_sleepers();
        }
        Ok(())
    }'''
replace_function("src/stabilized_rotating_world.rs", "    pub fn add_box(\n", add_box)

remove_box = '''    pub fn remove_box(&mut self, id: BodyId) -> Option<RigidBox3d> {
        let removed = self.inner.remove_box(id)?;
        self.pending_fixed_boundary_body_ids.remove(&id);
        self.wake_all_sleepers();
        Some(removed)
    }'''
replace_function("src/stabilized_rotating_world.rs", "    pub fn remove_box(\n", remove_box)

set_velocity = '''    pub fn set_linear_velocity(
        &mut self,
        id: BodyId,
        velocity: Vec3i,
    ) -> Result<(), RotatingWorldError3d> {
        self.inner.set_linear_velocity(id, velocity)?;
        self.sleeping.remove(&id);
        self.sleep_stable_time_q64.remove(&id);
        self.pending_fixed_boundary_body_ids.insert(id);
        Ok(())
    }'''
replace_function("src/stabilized_rotating_world.rs", "    pub fn set_linear_velocity(\n", set_velocity)

# Feed the precise changed-body set into the boundary stabilizer, including newly-added/touched subjects.
replace_once(
    "src/stabilized_rotating_world.rs",
    "        let mut changed_body_ids =\n            self.wake_sleepers_for_sweeps(timestep_numerator, timestep_denominator)?;\n",
    "        let mut changed_body_ids =\n            self.wake_sleepers_for_sweeps(timestep_numerator, timestep_denominator)?;\n        changed_body_ids.extend(self.pending_fixed_boundary_body_ids.iter().copied());\n",
)
replace_once(
    "src/stabilized_rotating_world.rs",
    "        match self.stabilize_fixed_boundaries() {\n            Ok(stabilized) => changed_body_ids.extend(stabilized),\n",
    "        match self.stabilize_fixed_boundaries(&changed_body_ids) {\n            Ok(stabilized) => {\n                changed_body_ids.extend(stabilized);\n                self.pending_fixed_boundary_body_ids.clear();\n            }\n",
)

precise_stabilizer = '''    fn stabilize_fixed_boundaries(
        &mut self,
        subjects: &BTreeSet<BodyId>,
    ) -> Result<BTreeSet<BodyId>, RotatingWorldError3d> {
        let mut changed_body_ids = BTreeSet::new();

        for id in subjects.iter().copied() {
            let Some(current) = self.inner.box_by_id(id) else {
                return Err(RotatingWorldError3d::MissingBody(id));
            };
            if current.body.kind != BodyKind::Dynamic {
                continue;
            }

            let mut candidate = current.clone();
            let original_position = candidate.body.position;
            let mut converged = false;

            for _ in 0..MAX_FIXED_POSITION_STABILIZATION_PASSES {
                let before = candidate.body.position;
                let mut corrections = PositionCorrectionAccumulator::default();

                for fixed_id in self.inner.overlap_query(candidate.oriented_box())? {
                    if fixed_id == id {
                        continue;
                    }
                    let fixed = self
                        .inner
                        .box_by_id(fixed_id)
                        .ok_or(RotatingWorldError3d::MissingBody(fixed_id))?;
                    if fixed.body.kind != BodyKind::Fixed
                        || !candidate
                            .collision_layers()
                            .collides_with(fixed.collision_layers())
                    {
                        continue;
                    }

                    let response = if id < fixed_id {
                        resolve_obb_contact(candidate.clone(), fixed.clone(), false)
                    } else {
                        resolve_obb_contact(fixed.clone(), candidate.clone(), false)
                    }
                    .map_err(|error| {
                        RotatingWorldError3d::Response(RotatingContactResponseError3d::Pair(error))
                    })?;
                    let projected = if id < fixed_id {
                        response.left.body.position
                    } else {
                        response.right.body.position
                    };
                    if projected != before {
                        corrections.accumulate(id, before, projected)?;
                    }
                }

                if corrections.is_empty() {
                    converged = true;
                    break;
                }
                let projected = corrections.target_position(id, before)?;
                if projected == before {
                    break;
                }
                candidate.body.position = projected;
            }

            if !converged {
                return Err(RotatingWorldError3d::PersistentTailResolutionLimit(
                    u32::from(MAX_FIXED_POSITION_STABILIZATION_PASSES),
                ));
            }
            if candidate.body.position == original_position {
                continue;
            }

            let mut rigid_box = self
                .inner
                .remove_box(id)
                .ok_or(RotatingWorldError3d::MissingBody(id))?;
            rigid_box.body.position = candidate.body.position;
            self.inner.add_box(rigid_box)?;
            changed_body_ids.insert(id);
        }

        Ok(changed_body_ids)
    }'''
replace_function(
    "src/stabilized_rotating_world.rs",
    "    fn stabilize_fixed_boundaries(&mut self) -> Result<BTreeSet<BodyId>, RotatingWorldError3d> {\n",
    precise_stabilizer,
)

# Previous full-world fixed/dynamic pair discovery remains only as a deterministic oracle/benchmark.
replace_once(
    "src/stabilized_rotating_world.rs",
    "fn fixed_dynamic_pairs(\n",
    "#[cfg(test)]\nfn fixed_dynamic_pairs(\n",
)
replace_once(
    "src/stabilized_rotating_world.rs",
    "fn map_fixed_boundary_broad_phase_error(error: RotatingBroadPhaseError3d) -> RotatingWorldError3d {\n",
    "#[cfg(test)]\nfn map_fixed_boundary_broad_phase_error(error: RotatingBroadPhaseError3d) -> RotatingWorldError3d {\n",
)

# Test that unrelated dynamics are not part of the precise boundary subject work.
p = Path("src/stabilized_rotating_world.rs")
text = p.read_text()
replace_once(
    "src/stabilized_rotating_world.rs",
    "    use std::{hint::black_box, time::Instant};\n",
    "    use std::{collections::BTreeSet, hint::black_box, time::Instant};\n",
)
marker = "    #[test]\n    fn low_motion_body_sleeps_and_remains_stationary() {\n"
test = '''    #[test]
    fn fixed_boundary_stabilization_consumes_only_named_dynamic_subjects() {
        let target = BodyId(1);
        let mut world = zero_gravity_world();
        world
            .add_box(fixed(1000, Vec3i::ZERO))
            .expect("fixed obstacle");
        world
            .add_box(dynamic(target.0, Vec3i::ZERO, Vec3i::ZERO))
            .expect("overlapping target");
        for id in 2..=66 {
            world
                .add_box(dynamic(
                    id,
                    Vec3i::new(i32::try_from(id).expect("small id") * 32, 0, 0),
                    Vec3i::ZERO,
                ))
                .expect("unrelated dynamic");
        }

        let before_unrelated = world
            .box_by_id(BodyId(66))
            .expect("unrelated body")
            .clone();
        let changed = world
            .stabilize_fixed_boundaries(&BTreeSet::from([target]))
            .expect("precise fixed-boundary stabilization");

        assert_eq!(changed, BTreeSet::from([target]));
        assert_ne!(
            world.box_by_id(target).expect("target remains").body.position,
            Vec3i::ZERO
        );
        assert_eq!(world.box_by_id(BodyId(66)), Some(&before_unrelated));
    }

    #[test]
    #[ignore = "release-mode evidence that fixed-boundary work follows changed subjects"]
    fn fixed_boundary_changed_subject_scaling_benchmark() {
        for unrelated in [32_u64, 128, 512] {
            let target = BodyId(1);
            let mut world = zero_gravity_world();
            world
                .add_box(fixed(1000, Vec3i::ZERO))
                .expect("fixed obstacle");
            world
                .add_box(dynamic(target.0, Vec3i::ZERO, Vec3i::ZERO))
                .expect("overlapping target");
            for id in 2..=(unrelated + 1) {
                world
                    .add_box(dynamic(
                        id,
                        Vec3i::new(i32::try_from(id).expect("small id") * 32, 0, 0),
                        Vec3i::ZERO,
                    ))
                    .expect("unrelated dynamic");
            }

            let started = Instant::now();
            let changed = black_box(
                world
                    .stabilize_fixed_boundaries(&BTreeSet::from([target]))
                    .expect("precise stabilization"),
            );
            let elapsed = started.elapsed();
            assert_eq!(changed, BTreeSet::from([target]));
            println!(
                "precise fixed-boundary stabilization: unrelated_dynamics={unrelated}, subjects=1, elapsed={elapsed:?}"
            );
        }
    }

'''
if text.count(marker) != 1:
    raise SystemExit("test insertion marker missing")
p.write_text(text.replace(marker, test + marker, 1))
