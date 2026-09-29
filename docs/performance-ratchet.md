# Performance ratchet

The ratchet prevents known architectural work from creeping back and preserves a reviewable sequence
of measurements. It is a required step inside the existing `validate` CI job, on every pull request and
main push, independent of changed-file routing or availability of a comparison build.

## Run it

Install the repository's pinned Rust toolchain and its WASM target once:

```bash
rustup target add wasm32-unknown-unknown
```

The runner also requires Node.js and `tar` (available on the CI runner). Then run the policy tests and
the ratchet from the repository root:

```bash
node --test scripts/performance-ratchet.test.mjs
node scripts/performance-ratchet.mjs check
```

The commands above assume the repository root when locating the script. Once invoked, the runner
resolves every input against its own repository, not the caller's working directory. No dependency is
added to the physics library. The normal fast tier and dedicated replay tests remain required; a lower
work count is never a substitute for correctness.

`check` runs seven selected native fixtures in release mode and the six sandbox scenes twice in WASM.
It compares every work count with the latest Git-tracked entry in `.performance/ratchet/history/`.
Missing metrics/scenarios, failed/skipped fixtures, changed correctness evidence, nondeterministic
replays, and any increased work count fail. A zero budget stays zero. Timings are advisory because
shared runners and changing host load do not support reliable absolute millisecond gates.

## What is protected

| Fixture | Blocking evidence |
| --- | --- |
| Sparse ballistic targets: 32 / 256 / 2,048 | Actual target-bound checks; 256 successful hits |
| Stationary query cache: 32 / 256 / 2,048 | Bound preparations and subject rebuilds over 64 changing generations; fixed query volume |
| Touching wake chains: 8 / 32 | Wake queries and number of awakened bodies |
| Local wake: 32 / 128 sleepers | Unchanged bounds prepared, contact-island query count, remaining sleepers |
| Repeated wake/park: 32 / 128 / 512 bodies | Subject and graph rebuilds over four cycles; every paired contact remains exact |
| SAT projection | Actual dot-product calls and parity with eight-vertex projection |
| Prepared SAT | Query/contact counts; candidate and reference timings recorded |
| Six sandbox scenes | Every recorded cost counter, step/body/event counts, exact replay hashes |

Reuse counters are retained in raw sandbox evidence but are not ceilings: more reuse can be an
improvement. Native timing samples run in test builds with test-only instrumentation; WASM measurements
use the production release build. The SAT benchmark includes an alternating eight-vertex reference.
The native geometry/query fixtures keep their existing correctness assertions and oracle comparisons.

Fixture names and required outputs live in `.performance/ratchet-fixtures.json`; the manifest hash is
part of each baseline. The legacy sandbox contract also has absolute ceilings, so its standalone check
no longer silently skips all relative checks when a comparison build is unavailable. The latest ratchet
entry is the stricter evolving ceiling.

## Record an improvement

After implementation and correctness checks pass:

```bash
node scripts/performance-ratchet.mjs record \
  --id reuse-unchanged-contact-islands \
  --reason "Reuse unchanged contact islands after local body motion; no replay changes."
```

This reruns the fixtures, requires the existing correctness fingerprints and work ceilings, and appends
one uniquely named entry. Include that entry with the implementation in the same reviewed commit/PR.
The command never edits old entries; running it again with the same ID fails. Checks never promote
their own measurements. An improvement without a recorded entry is reported but does not silently
change the budget.

The ledger stores the reason, timestamp, source commit, dirty flag, source-content hash, fixture hash,
WASM hash, exact environment, every work/correctness metric, and advisory timings. Each entry hashes
its predecessor. CI also compares history with the base commit, so rewriting old entries and regenerating
their hashes cannot conceal a regression. Reviewers can inspect each improvement with:

```bash
node scripts/performance-ratchet.mjs history
git log -- .performance/ratchet/history
```

Correctness repairs can legitimately change replay hashes or require more collision work. For that
case, `record --new-epoch --id ... --reason ...` explicitly starts a new correctness/workload baseline
while preserving the old one. The reason must explain the semantic change and its correctness evidence.
Review any relaxed ceilings as part of that change; update the legacy sandbox contract if necessary.
Do not use a new epoch to hide a performance regression. Ordinary records cannot raise budgets or drop
metrics. Version the fixture manifest when workload semantics change.

## Evidence and provenance

Every run writes to a unique ignored `performance-evidence/ratchet/` directory, including on benchmark
or comparison failure. It contains the report, native logs, raw sandbox trials, source file hashes,
an archive of measured source inputs (including untracked files), and the tracked diff. Source copies
stay in `source.tar.gz` so repository discovery cannot mistake archived Cargo manifests for active
projects. Extract it outside the working repository when reproducing an old run. The runner rejects
source edits during collection. Dirty measurements are explicitly labeled; a commit SHA alone never
pretends to identify uncommitted implementations. Set `CONVENTIONS_SOURCE_REVISION` when the shared
policy resolver is available; otherwise its absence is recorded as `null`.

CI uploads these artifacts for 90 days. The compact ledger is an intentional versioned performance
contract and remains in Git after artifact expiration. Keep large raw measurements out of Git.
`check --base <full-commit-sha>` performs the same append-only history check locally as CI.

The initial entry captures the corrected architecture, SAT, and sleeping-cache implementation together.
Earlier working-tree iterations were not separate commits; their measured before/after results and
correctness changes are documented in [the architecture review](architecture-review.md). They are
historical evidence, not fabricated ledger revisions or interchangeable replay baselines. From this
point onward, record each accepted improvement as its own ledger entry to trace it precisely.

The sleep-transition coverage entry adds three scenarios before optimizing them. Its new workload epoch
preserves all original ceilings and replay fingerprints; the following optimization entry tightens the
new counters within that epoch. This keeps the measured before/after evidence in the ledger.

## Contact-triggered wake reconciliation

The first two epochs measured the pre-migration proximity-wake implementation. Its wake-chain and
local-wake tests were lost when #152 replaced swept-proximity waking with contact-triggered activation;
the sleep-transition fixture was listed but omitted from the committed source. The archived source
snapshot for entry 0003 restores the original paired-contact fixture and its unchanged ceilings:
zero graph builds and 12 subject rebuilds over four cycles at each population size.

The replacement wake-chain fixture exercises the current contact-island traversal after contact
admission, visits each 8/32-body chain member once, and keeps a disconnected body asleep. Local waking
of one body among 32/128 sleepers performs no contact-island traversal for its subsequent miss. Parking,
activation and removal now insert/remove affected BVH leaves instead of rebuilding all parked bounds;
explicit waking therefore prepares zero unchanged bounds. Real-impact, shielding, ricochet, independent
island and support-removal acceptance remains in `tests/projectile_wake_admission.rs`.

A new epoch records the already-landed #152 numerical/contact-wake semantics. The previous archived
WASM replay differs from current main in five active scenarios; settled idle is identical. This is not
a numerical- or wake-policy change made by this reconciliation. Against current main, all six candidate
replay hashes, events and previously exported work counters must match. Additional raw activation and
probe counters make discarded work independently inspectable instead of hiding it in combined totals.

The legacy query ceilings change only where the admitted-contact repair already introduced additional
probe work: walking 776 → 782, three shots while walking 3,052 → 3,064, and six idle shots 2,718 → 2,727.
The walking/three-shot differences are exactly 6/12 discarded probe queries. Six idle shots now
perform 2,706 committed queries plus 21 probe queries, compared with the historical 2,718 committed
queries. Other absolute ceilings remain unchanged. The append-only history preserves earlier evidence, records
all increases/decreases explicitly, and enforces the current completed workload thereafter. The runner
now verifies every exact native test name before collecting release measurements, so a removed/renamed
fixture fails before the expensive benchmark rather than succeeding with zero tests.

## Proportional sphere mass (epoch 4)

PR150 changes only the explicit analytic-sphere stress lane's mass, from one to
`2 * PI / 1296`, matching the legacy sandbox's nominal sphere/crate density.
Twenty of 21 rows retain their exact correctness and work values. The sphere trace
still completes all 240 steps twice with 18 bodies and identical within-build replay;
its event total changes from 2029 to 1942. The unchanged pyramid regression requires
visible contact response, one retirement, bounded excursion and eventual sleep.

This is a behavior correction with a performance tradeoff. In the persistent-sphere
stress control, real wake admissions increase from 6 to 28, broad-phase queries from
2839 to 3503, partial queries from 710 to 2124, and stabilization-active bodies from
3664 to 6462. The three exceeded stress ceilings now equal those measured counts;
all unrelated ceilings and simulation budgets remain unchanged. Sampled events fall
from 901 to 458, but that does not make this trace an overall performance improvement.
Both raw runs include discarded wake-probe work. The earlier evidence remains in the
append-only history; epoch 4 protects the corrected trace against future regressions.
