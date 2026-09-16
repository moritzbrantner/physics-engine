import { readFileSync } from "node:fs";

const [contractPath, resultsPath] = process.argv.slice(2);
if (!contractPath || !resultsPath) {
  throw new Error(
    "usage: node scripts/validate-performance-contract.mjs <contract.json> <results.json>",
  );
}

const contract = JSON.parse(readFileSync(contractPath, "utf8"));
const results = JSON.parse(readFileSync(resultsPath, "utf8"));

if (contract.schemaVersion !== 1) {
  throw new Error(`unsupported performance contract schema ${contract.schemaVersion}`);
}
const workloadMatches =
  contract.suite === results.workload || contract.suite.endsWith(`/${results.workload}`);
if (!workloadMatches) {
  throw new Error(
    `performance contract suite ${contract.suite} does not match workload ${results.workload}`,
  );
}

const caseByName = (measurement, name) =>
  measurement?.cases?.find((entry) => entry.name === name) ?? null;

function metricValue(trial, name) {
  if (name === "mean_step_ms") return trial.steps?.mean_ms ?? null;
  if (name === "p95_step_ms") return trial.steps?.p95_ms ?? null;
  if (name === "event_sum") return trial.event_sum ?? null;
  if (Object.hasOwn(trial.work ?? {}, name)) return trial.work[name];
  return null;
}

function requireFiniteMetric(value, scenarioId, metricName, side) {
  if (typeof value !== "number" || !Number.isFinite(value)) {
    throw new Error(
      `${scenarioId}: blocking metric ${metricName} is unavailable for ${side}`,
    );
  }
}

function validateCorrectnessEvidence(scenario, headCase) {
  for (const [trialIndex, trial] of headCase.trials.entries()) {
    for (const evidence of scenario.correctnessEvidence ?? []) {
      if (evidence === "collision/event evidence" && !(trial.event_sum > 0)) {
        throw new Error(
          `${scenario.id}: trial ${trialIndex} requires collision/event evidence, got event_sum=${trial.event_sum}`,
        );
      }
      if (evidence === "zero collision events" && trial.event_sum !== 0) {
        throw new Error(
          `${scenario.id}: trial ${trialIndex} requires zero collision events, got event_sum=${trial.event_sum}`,
        );
      }
      if (
        evidence === "body-count validity" &&
        (!Number.isInteger(trial.body_count) || trial.body_count <= 0)
      ) {
        throw new Error(
          `${scenario.id}: trial ${trialIndex} has invalid body_count=${trial.body_count}`,
        );
      }
    }
  }
}

function validateMetricBudget(scenario, metric, headCase, baselineCase) {
  if (!metric.blocking || !metric.budget) return;

  for (const [trialIndex, headTrial] of headCase.trials.entries()) {
    const headValue = metricValue(headTrial, metric.name);
    requireFiniteMetric(headValue, scenario.id, metric.name, "head");

    if (Object.hasOwn(metric.budget, "max")) {
      if (headValue > metric.budget.max) {
        throw new Error(
          `${scenario.id}: ${metric.name}=${headValue} exceeds max=${metric.budget.max}`,
        );
      }
      continue;
    }

    if (Object.hasOwn(metric.budget, "relativeRegressionPercent")) {
      if (!baselineCase) {
        console.warn(
          `${scenario.id}: no baseline available; skipping relative budget for ${metric.name}`,
        );
        continue;
      }
      const baselineTrial = baselineCase.trials[trialIndex];
      if (!baselineTrial) {
        throw new Error(`${scenario.id}: missing baseline trial ${trialIndex}`);
      }
      const baselineValue = metricValue(baselineTrial, metric.name);
      requireFiniteMetric(baselineValue, scenario.id, metric.name, "baseline");

      const regression = metric.budget.relativeRegressionPercent / 100;
      if (metric.direction === "lower") {
        const limit = baselineValue === 0 ? 0 : baselineValue * (1 + regression);
        if (headValue > limit) {
          throw new Error(
            `${scenario.id}: ${metric.name}=${headValue} exceeds relative limit=${limit} from baseline=${baselineValue}`,
          );
        }
      } else if (metric.direction === "higher") {
        const limit = baselineValue * (1 - regression);
        if (headValue < limit) {
          throw new Error(
            `${scenario.id}: ${metric.name}=${headValue} is below relative limit=${limit} from baseline=${baselineValue}`,
          );
        }
      } else {
        throw new Error(`${scenario.id}: unsupported direction ${metric.direction}`);
      }
      continue;
    }

    throw new Error(`${scenario.id}: unsupported blocking budget for ${metric.name}`);
  }
}

for (const scenario of contract.scenarios ?? []) {
  const headCase = caseByName(results.head, scenario.id);
  if (!headCase) throw new Error(`missing head scenario ${scenario.id}`);
  const baselineCase = caseByName(results.baseline, scenario.id);

  validateCorrectnessEvidence(scenario, headCase);
  for (const metric of scenario.metrics ?? []) {
    validateMetricBudget(scenario, metric, headCase, baselineCase);
  }
}

console.log(`performance contract passed: ${contract.suite}`);
