---
name: agent-loop
description: Run one iteration of the physics-engine multi-agent loop — review and merge open agent PRs, promote drafted issues to ready specs, queue the next roadmap slices (consumer-blocking first), implement Opus tasks, dispatch Sonnet tasks and keep a backlog for Sol, which runs separately and occasionally. Use when the user says "start/run the loop" or invokes /agent-loop; wrap in /loop for continuous pacing.
---

# Agent loop

You are the loop driver (Claude Opus). The contract for issues, labels and roles is `docs/AGENT_TASKS.md`; the rules every implementer follows are `AGENTS.md`. Read both at the start of every run, and the roadmap issue #191 plus `README.md` before writing a new spec.

**Sol is offline by default.** The user runs Sol's Codex loop occasionally and never needs to run it alongside this one. Never wait for Sol: keep the engine moving with Opus and Sonnet, and treat `agent:sol` issues as a backlog Sol works through whenever it is started. If Sol does run at the same time, the `in-progress` label is the lock; never touch an `in-progress` Sol issue or push to a Sol branch.

One run = the steps below, in order, then a short report. Keep chat output to the report; put spec content into issues and review content into PR comments.

## 0. Baseline

- `git fetch` and work from `origin/main`. Never edit the user's checked-out branch; use a worktree for any change you make yourself.
- Make sure the labels in `docs/AGENT_TASKS.md` exist (`gh label create … || true`).
- Collect state:
  - `gh pr list --state open --json number,title,headRefName,author,labels,isDraft,url`
  - `gh issue list --label agent-task --state open --json number,title,labels,body`
  - `gh issue list --state open --limit 100 --json number,title,labels,body` (roadmap issues without `agent-task`)

## 1. Review open PRs

For each open, non-draft PR that closes an `agent-task` issue:

1. **CI:** `gh pr checks <n>`. If pending, skip it this run. If red, comment the failing check and log excerpt, then stop on this PR.
2. **Codex:** read the review comments and threads from `chatgpt-codex-connector` (`gh api repos/{owner}/{repo}/pulls/<n>/comments`, `.../reviews`, and the issue comments). Require a completed connector review covering the current head commit; the review-summary issue comment may record completion even when there are no findings. Skip this PR while that review is absent or running. Every finding must be fixed or answered in the thread. If the head changed after the completed review, comment `@codex review` when no current-head review is running and skip until it completes.
3. **Spec:** compare the diff with the issue's Decisions, Acceptance and Out of scope:
   - public signatures and checkpoint format/algorithm versions match exactly, with at most one bump each;
   - nothing out of scope slipped in;
   - acceptance tests and reference/oracle comparisons exist;
   - ratchet history entries exist where work counts improved, and no ceiling or physical threshold was relaxed;
   - the Playwright scripts were claimed where the change is browser-visible.

   Also check the `AGENTS.md` rules (authority boundary with `rust-kernels` and consumers, numerical policy, stable `BodyId` ordering, swept CCD, failure atomicity).
4. **Verdict:**
   - **Ready:** `gh pr merge <n> --merge --delete-branch`. If auto mode denies the merge, do not work around it; list the PR as "ready for you to merge" in the report. If the PR closes an issue a consumer is waiting on, name the consumer issue and the merge commit in the report so the consumer can bump its pin.
   - **Changes needed:** one PR comment with a numbered, concrete list. For a PR by Sonnet, dispatch Sonnet again with that list (step 4). For a PR by Opus, re-dispatch the Opus agent with that list (step 4); never fix it inline as well. For Sol, leave the comment; Sol's next run fixes its own PRs first.

Merge only physics-engine PRs. Never merge PRs in consumer or other foundation repositories (mmorpg, arpg, ecs-lab, rust-kernels, …); list them for the user.

## 2. Promote drafts

For each `spec:draft` issue (often drafted in a ChatGPT chat):

- Check it against the current code on `origin/main`: public API names, module paths, checkpoint format/algorithm versions, ratchet history, capability ledger, open parallel tasks.
- Check it against `docs/AGENT_TASKS.md`: sizing, one version bump, the implementer label, every section present.
- If you can complete it by deciding things yourself, edit the body (`gh issue edit <n> --body-file …`), summarise what you changed in a comment, and swap `spec:draft` for `spec:ready`.
- If a decision belongs to the owner (scope, a public contract a consumer depends on, numerical policy, authority boundaries), ask in a comment and label it `spec:needs-input`. Re-check those issues for answers on every run.

## 3. Refresh and fill the queues

**Refresh the Sol backlog.** For each `agent:sol` + `spec:ready` issue not `in-progress`, re-check it against current `origin/main`: API names, versions, ratchet entries and "Parallel work". Edit the body when merges have moved them, with a one-line comment.

**Fill the queues.** A startable task is an open `spec:ready` issue whose "Start after" dependencies are merged.
- **Opus and Sonnet:** each keeps exactly one startable task.
- **Sol:** keeps up to three. Give Sol only work that nothing else will depend on soon. Examples: contact-quality controls, primitive collider kernels, measured maintenance slices no consumer is waiting on. Put consumer-blocking and public-contract work on `agent:opus`.
- Never make an Opus or Sonnet task "Start after" an unstarted Sol task.
- If a Sol task already blocks queued work (or a consumer) and has not been started for 24 hours, reassign it to `agent:opus`: swap the label, update the header's "Intended implementer" line in the issue body, and comment why.

For each agent below its target, pick the next slice in this order:

1. **Consumer-blocking work first.** Issues a downstream consumer is waiting on: requests filed for moritzbrantner/mmorpg and other sibling repos (arpg, battle-royale, ecs-lab, collision-lab, medieval). Find them by checking issue bodies and links for "consumer", "mmorpg", "dogfood", a sibling-repo issue reference, or the consumer migration tickets listed in #191. Check the consumer issue too: if it is waiting on this engine slice, the slice is critical path.
2. **The roadmap.** The open child issues of #191 in its milestone order (M1 → M2 → M3, respecting its dependency rules), then the README "Near-term implementation sequence".

Rules while queueing:
- Respect dependencies: a measured baseline lands before the improvement that relies on it, and a `site/` task waits for its engine task. Queue only a slice whose own dependencies are already merged; if no such slice exists for that agent, queue nothing and say so in the report.
- Avoid conflicts: never queue two tasks that bump the checkpoint format or algorithm version, append ratchet history, or edit the same solver module concurrently, including Sol backlog tasks that may start at any time.
- Roadmap issues are umbrellas; file a new bounded issue per slice rather than labelling the umbrella. Write it exactly per `docs/AGENT_TASKS.md` "Writing an issue", with labels `agent-task`, `spec:ready` and the `agent:*` label. Verify every number and name you cite against the code first.
- Link it from the parent roadmap issue with a one-line comment.

Write at most three new specs per run.

## 4. Dispatch

- **`agent:sonnet`** (ready, not in progress, and every "Start after" issue closed by a merged PR): add `in-progress`, then launch a background Agent:
  - `model: "sonnet"`, `isolation: "worktree"`;
  - prompt: "Implement issue #N of moritzbrantner/physics-engine. Read AGENTS.md, docs/AGENT_TASKS.md and the issue. Work on the branch the issue names, commit in small steps, run the focused checks plus whatever the issue lists that CI does not run, push, and open the PR with `Closes #N` only when the branch is complete. Report the PR URL and anything you could not verify."

  For a "changes needed" re-dispatch, give the PR number and the numbered list instead. Run at most one Sonnet task at a time.
- **`agent:opus`** (startable, not in progress): add `in-progress` and launch a background Agent so this loop keeps running:
  - `model: "opus"`, `isolation: "worktree"`;
  - the same prompt as for Sonnet.

  Run at most one Opus implementation at a time. For "changes needed" on an Opus PR, re-dispatch with the list.
- **`agent:sol`**: never dispatched from here. Sol runs the Codex `implementer-loop` skill (`.agents/skills/implementer-loop/`) whenever the user starts it. It fixes its own PRs first, then works through the backlog. Do not nag: mention the Sol backlog in the report only when it changed this run.

## 5. Report

End with a compact table: each PR (merged / changes requested / waiting for CI or Codex / ready for the user to merge), each issue (promoted / needs input / newly queued / dispatched), and a "For you" list naming only the user's actions (merges auto mode refused, questions, the Sol backlog when it changed). When a consumer-blocking PR merged, add a "Consumers can bump" line: consumer issue, merged PR and the merge commit SHA for the new pin.

## Pacing

- A single invocation does one run.
- For continuous operation the user runs `/loop /agent-loop`. Schedule the next wakeup around 1800 s while PRs wait on CI or Codex. While an unstarted Sol task blocks queued work, also schedule a wakeup no later than its 24-hour reassignment deadline (the clamp is 3600 s, so keep waking hourly until then).
- Stop the loop when no Opus or Sonnet work is in flight or startable and no roadmap slices remain for them. A non-empty Sol backlog alone is not a reason to keep looping, but a Sol task that blocks queued work or a consumer is.
- A finished background Sonnet or Opus agent re-invokes you; continue from step 1 for its PR.
