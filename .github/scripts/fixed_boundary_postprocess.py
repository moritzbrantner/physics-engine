from pathlib import Path

path = Path("src/stabilized_rotating_world.rs")
text = path.read_text()

old = """        let mut changed_body_ids =
            self.wake_sleepers_for_sweeps(timestep_numerator, timestep_denominator)?;
        changed_body_ids.extend(self.pending_fixed_boundary_body_ids.iter().copied());
        self.freeze_sleeping_bodies()?;
"""
new = """        let mut changed_body_ids =
            self.wake_sleepers_for_sweeps(timestep_numerator, timestep_denominator)?;
        let mut fixed_boundary_subjects = changed_body_ids.clone();
        fixed_boundary_subjects.extend(self.pending_fixed_boundary_body_ids.iter().copied());
        self.freeze_sleeping_bodies()?;
"""
if text.count(old) != 1:
    raise SystemExit("initial subject-set block not found exactly once")
text = text.replace(old, new, 1)

old = """        changed_body_ids.extend(report.changed_body_ids.iter().copied());
        match self.stabilize_fixed_boundaries(&changed_body_ids) {
"""
new = """        changed_body_ids.extend(report.changed_body_ids.iter().copied());
        fixed_boundary_subjects.extend(changed_body_ids.iter().copied());
        match self.stabilize_fixed_boundaries(&fixed_boundary_subjects) {
"""
if text.count(old) != 1:
    raise SystemExit("stabilizer subject call not found exactly once")
text = text.replace(old, new, 1)

old_import = "    use std::{hint::black_box, time::Instant};\n"
new_import = "    use std::{collections::BTreeSet, hint::black_box, time::Instant};\n"
if text.count(old_import) != 1:
    raise SystemExit("test import not found exactly once")
text = text.replace(old_import, new_import, 1)

old_clear = "    fn clear(&mut self) {\n        self.groups.clear();\n    }\n"
if text.count(old_clear) != 1:
    raise SystemExit("obsolete accumulator clear method not found exactly once")
text = text.replace(old_clear, "", 1)

path.write_text(text)
