# Performance ratchet

The ratchet prevents known architectural work from creeping back and preserves a reviewable sequence
of measurements. It is a required step inside the existing `validate` CI job, on every pull request and
main push, independent of changed-file routing or availability of a comparison build.

## Run it

Install the repository's pinned Rust toolchain and its WASM target once:

```bash
rustup target add wasm32-unknown-unknown
```

Then run the policy tests and the ratchet from the repository root:

```bash
node --test scripts/performance-ratchet.test.mjs
node scripts/performance-ratchet.mjs check
```

The commands above assume the repository root when locating the script. Once invoked, the runner
resolves every input against its own repository, not the caller's working directory. No dependency is
added to the physics library. The normal fast tier and dedicated replay tests remain required; a lower
work count is never a substitute for correctness.

`check` runs six selected native fixtures in release mode and the six sandbox scenes twice in WASM.
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
| Local wake: 32 / 128 sleepers | Neighbor bounds prepared, awake sources visited, query count, remaining sleepers |
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
copies of measured source inputs (including untracked files), and the tracked diff. The runner rejects
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
