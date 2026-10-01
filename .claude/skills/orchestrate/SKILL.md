---
name: orchestrate
description: Work through every open physics-engine issue until only Sol tasks (or nothing) remain — classify each issue for Opus, Sonnet or Sol, write the missing specs (consumer-blocking first), implement Opus and Sonnet tasks with those models, review and merge their PRs, and continue with the next issue. Use when the user says "orchestrate", "start/run the loop" or invokes /orchestrate.
---

# Orchestrate

You are the orchestrator (Claude Opus). The contract for issues, labels and roles is `docs/AGENT_TASKS.md`; the rules every implementer follows are `AGENTS.md`. Read both at the start, and the roadmap issue #191 plus `README.md` before writing a new spec.

**One `/orchestrate` runs to completion.** Repeat passes (steps 0-5) until the end condition holds. Do not stop after one pass, and do not use timers (`ScheduleWakeup`, `/loop`).

**End condition.** Every open issue is in one of these states:
- merged and closed;
- `agent:sol`;
- a tracker whose children are all in one of these states (the roadmap issue #191 is one);
- blocked, with the blocker reported: an owner decision (`spec:needs-input`), an unmerged change in a consumer or sibling repo, or a Sol task.

Then write the step 6 report and stop.

**Sol is offline by default.** The user runs Sol's Codex loop occasionally, never alongside this one. Never wait for Sol. The `agent:*` label partitions issues, so the orchestrator (Opus/Sonnet) and Sol never pick up the same issue; `in-progress` only marks a started task. Never touch `in-progress` on a Sol issue or push to a Sol branch.

Keep chat output to short progress lines and the final report. Spec content goes into issues and review content into PR comments.

## 0. Baseline

- `git fetch` and work from `origin/main`. Never edit the user's checked-out branch; use a worktree for any change you make yourself.
- Make sure the labels in `docs/AGENT_TASKS.md` exist (`gh label create … || true`).
- Collect state:
  - `gh pr list --state open --limit 200 --json number,title,headRefName,author,labels,isDraft,url`
  - `gh issue list --state open --limit 200 --json number,title,labels,body` (all open issues, including roadmap issues without `agent-task`)
- **Recover stale locks.** Clear `in-progress` on an `agent:opus` or `agent:sonnet` issue, with a one-line comment, only when all of these hold:
  - no open PR references it;
  - the label was added more than 6 hours ago (issue timeline);
  - the issue's branch has no push in the last 6 hours, or does not exist;
  - no background agent from this session is working on it.

  Another session's agent may be invisible, so age and branch activity are the evidence. Report a Sol issue that has been `in-progress` for over 48 hours with no PR or branch push.

## 1. Classify every open issue

Give each open issue without an `agent:*` label exactly one classification, using the roles table in `docs/AGENT_TASKS.md`:

- **Tracker:** a parent or roadmap issue whose work lives in child issues (#191 and its milestones), or a bot dashboard. Leave it unlabelled. Close a tracker once all its children are merged.
- **`agent:opus`:** critical-path or cross-cutting engine work, including anything a consumer repository is blocked on, public API and world-contract changes, checkpoint format/algorithm changes, and anything an Opus or Sonnet task waits on.
- **`agent:sonnet`:** Pages scenario UI (`site/`), docs or mechanical follow-ups.
- **`agent:sol`:** narrow, technically deep work that nothing queued waits on (contact-quality controls, primitive collider kernels, measured maintenance slices no consumer is waiting on).
- **Blocked:** it needs an owner decision, or an unmerged change in a consumer or sibling repo. Comment the blocker once, and label `spec:needs-input` only for owner decisions.

Add the `agent:*` label. Consumer-blocking issues (requests filed for moritzbrantner/mmorpg and other sibling repos such as arpg, battle-royale, ecs-lab, collision-lab, medieval, or the consumer migration tickets listed in #191) go to `agent:opus`.

A Sol issue that an Opus or Sonnet task waits on, or that a consumer waits on, and that is not `in-progress`, moves to `agent:opus`: swap the label, update the header's "Intended implementer" line in the issue body, and comment why. Never make an Opus or Sonnet task "Start after" an unstarted Sol task.

Issues that are not yet specs (no `agent-task` label) become specs when they come up in step 4.

## 2. Review open PRs

For each open, non-draft PR that closes an `agent-task` issue:

1. **CI:** `gh pr checks <n>`.
   - Pending: move on to other work this pass.
   - A check that concluded `cancelled` (for example, superseded by a concurrency group) is not a failure: re-run it (`gh run rerun <run-id>`) and treat the PR as pending.
   - Any other non-success conclusion means "changes needed": comment the failing check and log excerpt, then re-dispatch the owning agent with that list (step 5).
   - For a Sol PR, leave the comment; Sol's next run fixes its own PRs first.
2. **Codex:** read the review comments and threads from `chatgpt-codex-connector` (`gh api repos/{owner}/{repo}/pulls/<n>/comments`, `.../reviews`, and the issue comments).
   - Require a completed connector review covering the current head commit. The review-summary issue comment may record completion even when there are no findings.
   - Every finding must be fixed or answered in its thread.
   - If the head changed after the completed review, comment `@codex review` when no current-head review is running.
3. **Spec:** compare the diff with the issue's Decisions, Acceptance and Out of scope:
   - public signatures and checkpoint format/algorithm versions match exactly, with at most one bump each;
   - nothing out of scope slipped in;
   - acceptance tests and reference/oracle comparisons exist;
   - ratchet history entries exist where work counts improved, and no ceiling or physical threshold was relaxed;
   - the Playwright scripts were claimed where the change is browser-visible.

   Also check the `AGENTS.md` rules (authority boundary with `rust-kernels` and consumers, numerical policy, stable `BodyId` ordering, swept CCD, failure atomicity).
4. **Verdict:**
   - **Ready:** merge with `gh pr merge <n> --merge --delete-branch --match-head-commit <sha>`, using the head SHA that CI, Codex and the spec review covered. If the head moved, re-review. If auto mode denies the merge, do not work around it; list the PR as "ready for you to merge". If the PR closes an issue a consumer is waiting on, name the consumer issue and the merge commit in the report so the consumer can bump its pin.
   - **Changes needed:** one PR comment with a numbered, concrete list, then re-dispatch the owning Opus or Sonnet agent with it (step 5). Never fix it inline as well.
   - **Retry limit:** after three rounds on the same failure, stop re-dispatching and report the PR as blocked.

Merge only physics-engine PRs. Never merge PRs in consumer or other foundation repositories (mmorpg, arpg, ecs-lab, rust-kernels, 3d-lab, game-server, …); list them for the user.

## 3. Promote drafts and answered questions

For each `spec:draft` issue, and each `spec:needs-input` issue whose question has been answered:

- **Check against the code** on `origin/main`: public API names, module paths, checkpoint format/algorithm versions, ratchet history, capability ledger, open parallel tasks.
- **Check against `docs/AGENT_TASKS.md`:** sizing, one version bump, the implementer label, every section present.
- **If you can complete it** by deciding things yourself: edit the body (`gh issue edit <n> --body-file …`), summarise what you changed in a comment, and swap its `spec:*` label for `spec:ready`.
- **If a decision belongs to the owner** (scope, a public contract a consumer depends on, numerical policy, authority boundaries): ask in a comment and swap to `spec:needs-input`. Exactly one `spec:*` label remains either way.

## 4. Pick the next Opus and Sonnet task

For Opus and for Sonnet separately, when that agent has nothing in flight:

1. Take the next startable issue with its label. Startable means:
   - `spec:ready`;
   - not `in-progress`;
   - every "Start after" dependency is merged;
   - no conflict with work in flight: no two tasks bump the checkpoint format or algorithm version, append ratchet history, or edit the same solver module at the same time, including Sol tasks that are `in-progress`.

   Prefer, in this order:
   - **consumer-blocking work first:** issues a downstream consumer is waiting on. Check the consumer issue too: if it is waiting on this engine slice, the slice is critical path;
   - **the roadmap:** the open child issues of #191 in milestone order (M1 → M2 → M3, respecting its dependency rules), then the README "Near-term implementation sequence".
2. If none is startable but a classified issue for that agent has no spec yet, write the spec now. Follow `docs/AGENT_TASKS.md` "Writing an issue":
   - Either convert the issue in place (edit its body, add `agent-task` and `spec:ready`), or file a new bounded `agent-task` issue linked from it when it must be split, as roadmap umbrellas are. Link a new issue from its parent with a one-line comment.
   - Verify every number and name against the current code first.
   - Write specs just in time: a spec whose versions depend on unmerged work waits until that work merges, and a measured baseline lands before the improvement that relies on it.
3. Keep `agent:sol` + `spec:ready` issues current against `origin/main` (API names, versions, ratchet entries, "Parallel work"). Edit the body when merges have moved them, with a one-line comment. Keep up to three startable Sol tasks, only work nothing else will depend on soon.

## 5. Implement

- **Sonnet** (one task at a time): add `in-progress`, then launch a background Agent with `model: "sonnet"` and `isolation: "worktree"`. The prompt:

  > Implement issue #N of moritzbrantner/physics-engine. Read AGENTS.md, docs/AGENT_TASKS.md and the issue. Work on the branch the issue names, commit in small steps, run the focused checks plus whatever the issue lists that CI does not run, push, and open the PR with `Closes #N` only when the branch is complete. Report the PR URL and anything you could not verify.

  For a "changes needed" re-dispatch, give the PR number and the numbered list instead.
- **Opus** (one task at a time): add `in-progress`, then launch a background Agent with `model: "opus"`, `isolation: "worktree"` and the same prompt.
  - If the spec turns out to need an owner decision, the agent comments on the issue, swaps to `spec:needs-input` and stops. Ask the user (AskUserQuestion) when the session is interactive, record the answer on the issue, and resume the agent.
- **`agent:sol`:** never dispatched from here. Sol runs the Codex `implementer-loop` skill (`.agents/skills/implementer-loop/`) whenever the user starts it.

## 6. Continue, or finish

- **Keep going.** After each pass, start the next one immediately while there is work this session can do: a PR to review, a draft to promote, a spec to write or a task to dispatch.
- **Waiting.** When everything left waits on CI, Codex or a running agent:
  - End the turn only if something will wake you: a running background agent re-invokes you when it finishes, and a Monitor reports CI or Codex completion for open PRs. Arm one Monitor (`timeout_ms` 1800000) that prints a line when a watched PR's checks finish or its Codex review for the current head completes, and re-arm it when it expires.
  - Otherwise block in the foreground with a bounded command, for example `gh pr checks <n> --watch --interval 60` with a Bash timeout of up to 10 minutes.
- **Finish** when the end condition holds. Stop any Monitor you armed, then report:
  - a compact table of PRs (merged / changes requested / ready for the user to merge) and issues (classified / spec written / implemented / blocked / left for Sol);
  - a "Consumers can bump" line for each merged consumer-blocking PR: consumer issue, merged PR and the merge commit SHA for the new pin;
  - a "For you" list naming only the user's actions: questions, merges auto mode refused, blocked consumer or foundation work, and the Sol backlog.
