# Agent tasks

How work reaches the coding agents. A task is one GitHub issue that one agent turns into one PR (see `AGENTS.md`, Execution scope). Anyone may draft an issue, including a person or a chat assistant. An issue becomes implementable only once it is `spec:ready`.

The source of next steps is the roadmap issue #191 and its open child issues (PE-xx slices, primitive and contact-quality issues), plus the "Near-term implementation sequence" in `README.md`. Roadmap issues are umbrellas: a task is a bounded slice of one of them, filed as its own `agent-task` issue that names its parent.

## Roles

| Agent | Does |
| --- | --- |
| Claude Opus | Runs the loop (`/agent-loop`). Turns drafts into ready specs, writes new specs from the roadmap, reviews PRs against their spec and merges them. Implements critical-path and cross-cutting engine work itself (`agent:opus`): anything a consumer repository is blocked on, public API and world-contract changes, checkpoint format/algorithm changes. |
| ChatGPT Sol | Implements narrow, technically deep `agent:sol` tasks via the Codex `implementer-loop` skill. The spec should settle the API surface, solver semantics, versions and scope so Sol can spend depth on correctness rather than redesigning adjacent systems. Typical work: contact-quality controls, primitive collider kernels, measured maintenance slices. Runs occasionally, separately from `/agent-loop`, through a backlog of up to three tasks that nothing else waits on. |
| Claude Sonnet | Implements `agent:sonnet` tasks: Pages scenario UI (`site/`), docs, and mechanical follow-ups. |
| GitHub Actions | The full deterministic gate on every PR (`validate.yml`: fast tier, long replays, performance ratchet, Pages build with WASM contract tests; `performance-evidence.yml`: advisory evidence for path-matched PRs). |
| Codex review | Reviews each PR automatically when it is opened or marked ready; `@codex review` re-triggers it. |

## Labels

- `agent-task`: every task issue.
- `spec:draft`: written but not yet checked against the code. Do not implement.
- `spec:ready`: checked and implementable.
- `spec:needs-input`: blocked on a question for the owner, asked in a comment.
- `agent:opus`, `agent:sol`, `agent:sonnet`: the intended implementer.
- `in-progress`: an implementer has started; the PR will reference the issue. The `agent:*` label partitions issues, so the loop driver (Opus/Sonnet) and Sol never pick up the same issue; `in-progress` only marks a started task.

## Picking up a task (implementers)

When asked to "pick up work", take the oldest open issue labeled `spec:ready` plus your `agent:*` label that has no `in-progress` label and whose "Start after" dependencies are merged. Add `in-progress`, branch `agent/<topic>` (or the branch the issue names) and follow the issue and `AGENTS.md`. Open the PR only when the branch is complete, with `Closes #N`. Never implement `spec:draft` or `spec:needs-input` issues. If the spec turns out to be wrong or impossible, comment on the issue, replace `spec:ready` with `spec:needs-input`, remove `in-progress` and stop; do not silently re-scope it.

## Implementer loop

An implementer run (Codex: the `implementer-loop` skill in `.agents/skills/`; Sonnet: dispatched by `/agent-loop`) takes exactly one action, in this priority order, then reports and exits.

1. **Fix your own open PR.** A PR of yours (its issue carries your `agent:*` label) needs work when:
   - a CI check failed;
   - a Codex review finding is neither fixed nor answered;
   - the loop driver posted a "changes needed" comment newer than your last push.

   Fix it on the same branch, push, and reply to each finding. After substantial fixes, comment `@codex review`. After three failed attempts on the same failure, comment what blocks you on the PR and stop touching it.
2. **Otherwise, wait if your PR is still in review.** If a PR of yours is open and only waiting on CI, Codex or the loop driver's merge, do nothing. One task in flight per implementer.
3. **Otherwise, start the next task** per "Picking up a task". Work in a fresh worktree from `origin/main`. Commit in small steps. Run the focused checks plus what the issue lists that CI does not run. Push, then open the PR with `Closes #N`. Wait for CI and the first Codex review, and handle them as in step 1 within the same run.
4. **Otherwise, exit.** Do not invent work: no new issues, no tooling, CI or cleanup tasks.

An implementer never merges, never edits issue bodies, never writes specs and never changes a `spec:*` label except to replace `spec:ready` with `spec:needs-input` when the spec is wrong. That last case always comes with a comment explaining why and removal of `in-progress`.

## Writing an issue

**Title:** `<Roadmap slice>: <what the consumer or engine gains>`, for example `PE-02c: Reuse translational broad-phase rows for unchanged bodies`.

**Sizing:**
- One PR. Big enough to deliver a whole roadmap slice (or its measurement half or its repair half), small enough that one agent finishes it in one session.
- Split only along these seams: measurement before change (a measured baseline lands before the improvement that relies on it), and engine before Pages demo (a `site/` task starts after its engine task merges).
- At most one checkpoint format bump and one algorithm bump per task.
- Pick the implementer by the table above: ambiguous, cross-cutting, consumer-blocking or public-contract work → `agent:opus`; narrow but technically deep work with settled decisions, strong deterministic acceptance and no downstream waiters → `agent:sol`; Pages UI/docs and mechanical follow-ups → `agent:sonnet`.
- For `agent:sol`, keep breadth narrow even when implementation depth is high: pin the important decisions, name explicit out-of-scope boundaries, and do not rely on the implementer to decompose or redesign neighboring systems.

**Body:** use these sections in this order (the "Agent task" issue template has them):

1. **Header line:** roadmap slice and parent issue, consumer issue if a consumer is waiting, implementer, branch name, `Start after #N` if it depends on another task.
2. **Goal:** two or three sentences on the observable result for a consumer or the engine.
3. **Decisions already made (do not reopen):**
   - physical semantics, tolerances and numbers (tables welcome);
   - which world API(s) change (`World`, `RotatingWorld3d`, `approximate::World`) and the exact public signatures;
   - checkpoint format/algorithm version bumps, ratchet effects and capability-ledger changes;
   - compatibility behaviour for existing consumers, checkpoints and replay hashes;
   - deliberate simplifications.

   Anything left open says so explicitly ("implementer decides X; record it in the PR").
4. **Acceptance:** concrete tests, reference/oracle comparisons and work counters. Name the checks CI does not run (the Playwright scripts for browser-visible changes). Always end with "CI green and every Codex finding addressed".
5. **Expected changes:** modules, tests, scripts and docs likely touched.
6. **Out of scope:** what a thorough implementer might otherwise add. Always includes tooling, CI, pin and new-ratchet work.
7. **Parallel work:** open tasks touching the same files, and how to stay out of their way.

**Quality bar for `spec:ready`:**
- Consistent with `AGENTS.md` (authority, numerical policy, determinism, CCD).
- No unresolved design question that would change a public API, a checkpoint version or an authority boundary.
- Acceptance checks can be verified from the PR.
- Matches the current code: version numbers, module paths, API names and ratchet entries are checked on `main`.

## Drafting with a chat assistant

To hash out an issue in a chat (e.g. ChatGPT) and have it filed, paste this into the chat:

> You are helping me specify a task for the `moritzbrantner/physics-engine` repository. Before proposing anything, read `AGENTS.md`, `docs/AGENT_TASKS.md`, `README.md`, the roadmap issue #191 and the docs relevant to the topic (`docs/world-api-contract.md` for any public API change, `docs/floating-checkpoints.md` for any checkpoint change, `docs/numerics.md` for numerical policy). Discuss the task with me first: challenge scope that is too large for one PR, ask about decisions that would change public APIs, checkpoint versions or authority, and propose concrete numbers. When I say "file it", create a GitHub issue in `moritzbrantner/physics-engine` with the title and body sections exactly as in `docs/AGENT_TASKS.md` "Writing an issue", and the labels `agent-task`, `spec:draft` and the `agent:*` label we agreed on. Never label it `spec:ready`; Claude checks drafts against the code first. If you cannot create issues, output the title and the body as a Markdown code block instead.

If the chat cannot create issues, open a new issue with the "Agent task" template and paste the body. The next `/agent-loop` run checks the draft against the code, completes or corrects it, and flips it to `spec:ready` (or asks its questions under `spec:needs-input`).
