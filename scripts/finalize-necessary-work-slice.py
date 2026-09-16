from pathlib import Path

repeated = Path("src/repeated_rotating_events.rs")
text = repeated.read_text()
old = "use std::{collections::BTreeMap, error::Error, fmt};\n"
new = "use std::{error::Error, fmt};\n"
if old not in text:
    raise RuntimeError("expected repeated_rotating_events import not found")
repeated.write_text(text.replace(old, new, 1))

performance_test = Path("site/performance-log.test.mjs")
text = performance_test.read_text()
old = '''    broad_phase_reuses: 12,\n  });'''
new = '''    broad_phase_reuses: 12,
    broad_phase_incremental_updates: 0,
    broad_phase_reinserts: 0,
    broad_phase_rotations: 0,
    broad_phase_partial_queries: 0,
    broad_phase_partial_body_updates: 0,
    event_response_passes: 0,
    stabilization_passes: 0,
    stabilizations_hitting_limit: 0,
    stabilization_candidate_pairs: 0,
    stabilization_exact_contacts: 0,
    stabilization_active_bodies: 0,
  });'''
if text.count(old) != 1:
    raise RuntimeError("expected performance-work assertion marker not found exactly once")
performance_test.write_text(text.replace(old, new, 1))
