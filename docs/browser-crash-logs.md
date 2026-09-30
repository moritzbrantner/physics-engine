# Automatic scenario crash logs

Every scenario starts a local recording before initialization. The shared scenario
bootstrap and the separate fixed-step comparison both use `site/scenario-log.mjs`,
which owns one run lifecycle around the existing performance recorder. No solver
or authoritative physics behavior is changed.

On a returned engine failure, JavaScript exception, WASM trap, or unhandled promise
rejection, the first failure stops simulation work, finalizes the log, and requests
a JSON download. The error stays visible and a **Download crash log (JSON)** link
remains available if automatic downloading is blocked. Reporting does not query a
trapped engine. Secondary errors cannot replace the first captured failure.

The report includes the existing build/environment/scenario metadata, recent
frame/physics diagnostics, ordered operation markers and inputs, and the original
error message/stack, returned engine code/detail where available, and in-flight
operation. Completed calls in a failing callback are retained under the failure's
`pending_physics_steps_ms` / `pending_physics_step_stats` where collected; an
unfinished callback is not fabricated as a successfully completed raw frame.

**Download current log (JSON)** takes a snapshot without stopping recording.
An explicit scenario reset starts a new run and leaves the previous crash report
available. The most recent caught crash is also retained in session storage when
storage is available, so reload offers its download link without downloading it
again automatically. Logs are never uploaded automatically.

## Limits and interpretation

The automatic recorder retains the **latest 3,600 frames and 8,192 markers** in
bounded circular buffers. Older entries are counted as omitted. Initial metadata
remains available, frame/marker offsets are relative to run start, and timing/work
summaries explicitly cover **retained frames**, not the omitted history. Run
start/end and duration describe the whole run. This prevents a long session from
keeping only its startup frames and discarding the crash lead-up.

This is diagnostic evidence, **not a continuation-complete physical replay** or a
deterministic benchmark. It does not implement all of issue #203. Browser/tab
process termination, out-of-memory termination, and a main-thread hang may prevent
JavaScript from reporting anything. There is no claim of a final download or
complete recovery for those cases. Only caught failures are persisted; ordinary
frames do not perform synchronous storage writes. Storage/download failures are
reported separately, and the in-memory report remains at `window.physicsCrashLog`
when capture itself succeeds.

The existing default `createPerformanceSessionRecorder` mode still retains the
first bounded window for deliberate diagnostic callers. Automatic scenarios select
`retention: "latest"`; schema v2 receives additive retention/failure metadata.

## Verification

The existing Pages build already runs the expanded logging test suite:

```sh
node --test site/performance-log.test.mjs
node --test site/*.test.mjs
```

The browser boundary has a network-independent acceptance check, using the same
Playwright dependency as `scripts/test-tower-browser.py`:

```sh
python scripts/test-scenario-log-browser.py --output scenario-log-browser-evidence
```

It checks automatic startup, actual JSON downloads, first-error retention, manual
fallback after a delivery error, global exception/rejection handlers, reset, and
URL retention on a simulated BFCache pagehide. Full scenario navigation, real
BFCache navigation, and session-storage recovery across reload remain separate
browser acceptance requirements; the isolated boundary check does not prove them.
