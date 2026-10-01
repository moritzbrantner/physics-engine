---
name: Agent task
about: One PR-sized task for a coding agent (see docs/AGENT_TASKS.md)
title: "<Roadmap slice>: <what the consumer or engine gains>"
labels: ["agent-task", "spec:draft"]
---

Slice of #<parent roadmap issue> (roadmap #191). Consumer waiting: <owner/repo#N or "none">. Intended implementer: **<Opus|Sol|Sonnet>**. Start after: <#N or "nothing">. One branch (`agent/<topic>`), one PR; follows the `AGENTS.md` **Execution scope** rules.

## Goal

<Two or three sentences: what a consumer or the engine can do afterwards.>

## Decisions already made (do not reopen)

- **Semantics and numbers:** <physical behaviour, tolerances, limits>
- **API surface:** <world API(s) and exact public signatures; or "no public API change">
- **Versions and evidence:** <checkpoint format/algorithm bumps, ratchet entry, capability-ledger change; or "none">
- **Compatibility:** <what happens to existing consumers, checkpoints and replay hashes>
- **Left to the implementer:** <explicitly delegated choices, recorded in the PR>

## Acceptance

- <tests, reference/oracle comparisons, work counters>
- <Playwright scripts when browser-visible>
- CI green and every Codex review finding addressed or answered.

## Expected changes

- <modules/tests/scripts/docs>

## Out of scope

- <…>
- Tooling, CI, pin-refresh and new-ratchet work.

## Parallel work

- <open tasks touching the same files, or "none">
