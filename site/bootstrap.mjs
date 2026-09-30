import { scenarioLog } from "./scenario-log.mjs";

try {
  if (document.body.dataset.scenario === "fixed-step") {
    await import("./fixed-step-lab.js");
  } else {
    await import("./physics-settings.mjs");
    await import("./app.js");
  }
} catch (error) {
  scenarioLog.fail(error, { source: "module-initialization" });
}
