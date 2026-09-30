import { createPerformanceSessionRecorder, serializePerformanceSession } from "./performance-log.mjs";

const copy = value => JSON.parse(JSON.stringify(value));
const resolve = value => typeof value === "function" ? value() : value;

// One browser-run lifecycle around the existing recorder, never an engine-state owner.
export function createScenarioLog({
  environment = {}, scenario = {}, now = () => performance.now(), wallClock,
  maxFrames = 3_600, maxMarkers = 8_192,
  onCrash = () => {}, onDiagnosticError = error => console.error("Crash-log delivery failed", error),
} = {}) {
  let environmentSource = environment;
  let scenarioSource = scenario;
  let onFailure = () => {};
  let failed = false;
  let operation = null;
  let lastCrash = null;
  const recorder = createPerformanceSessionRecorder({
    environment: () => resolve(environmentSource), scenario: () => resolve(scenarioSource),
    now, wallClock, maxFrames, maxMarkers, retention: "latest",
  });
  const log = {
    recorder,
    get failed() { return failed; },
    get lastCrash() { return lastCrash; },
    configure(options) {
      if (options.environment) environmentSource = options.environment;
      if (options.scenario) scenarioSource = options.scenario;
      if (options.onFailure) onFailure = options.onFailure;
      recorder.refreshContext();
    },
    start() {
      recorder.start({ restart: true });
      failed = false;
      operation = null;
      recorder.recordMarker("scenario-start");
    },
    begin(name, detail = {}) {
      if (failed) return;
      operation = { name, detail: copy(detail), started_ms: now() };
      recorder.recordMarker(name, detail);
    },
    complete() { operation = null; },
    fail(error, detail = {}) {
      if (failed) return lastCrash;
      // Latch before callbacks or export, so a secondary error cannot replace the evidence.
      failed = true;
      const failure = {
        ...copy(detail),
        name: error instanceof Error ? error.name : "Error",
        message: error instanceof Error ? error.message : String(error),
        stack: error instanceof Error ? error.stack ?? null : null,
        operation: operation === null ? null : {
          name: operation.name, detail: operation.detail, duration_ms: now() - operation.started_ms,
        },
      };
      recorder.recordMarker("crash", failure);
      lastCrash = { ...recorder.finish(), outcome: "crashed", failure };
      // These are deliberate diagnostics/delivery boundaries. Neither failure resumes physics.
      try { onFailure(failure); } catch (secondary) { onDiagnosticError(secondary); }
      try { onCrash(lastCrash); } catch (secondary) { onDiagnosticError(secondary); }
      return lastCrash;
    },
  };
  log.start();
  return log;
}

function browserEnvironment() {
  return {
    build: null,
    browser: navigator.userAgent,
    platform: navigator.userAgentData?.platform ?? navigator.platform ?? null,
    hardware_concurrency: navigator.hardwareConcurrency ?? null,
    device_memory_gib: navigator.deviceMemory ?? null,
    device_pixel_ratio: window.devicePixelRatio ?? 1,
    renderer: null,
  };
}

function browserScenario() {
  return {
    id: document.body.dataset.scenario ?? "fixed-step",
    parameters: Object.fromEntries(new URLSearchParams(window.location.search)),
  };
}

function filename(session) {
  const prefix = session.outcome === "crashed" ? "physics-crash" : "physics-browser-session";
  return `${prefix}-${session.started_at.replaceAll(":", "-")}.json`;
}

function createBrowserScenarioLog() {
  let crashUrl = null;
  let crashPanel = null;
  const storageKey = "physics-engine:last-crash";
  const log = createScenarioLog({
    environment: browserEnvironment,
    scenario: browserScenario,
    onCrash(session) {
      window.physicsCrashLog = session;
      console.error("Physics scenario crashed", session.failure);
      // Persist only at failure, not synchronously on the simulation hot path.
      try { sessionStorage.setItem(storageKey, serializePerformanceSession(session)); }
      catch (error) { console.warn("Crash log could not be retained across reload", error); }
      publishCrash(session, true);
    },
  });

  function publishCrash(session, automatic) {
    if (crashUrl) URL.revokeObjectURL(crashUrl);
    crashPanel?.remove();
    crashPanel = document.createElement("section");
    crashPanel.className = "status";
    crashPanel.setAttribute("aria-label", "Crash report");
    const heading = document.createElement("p");
    heading.textContent = `${automatic ? "Captured crash" : "Previous crash report"}: ${session.failure.message}`;
    const link = document.createElement("a");
    crashUrl = URL.createObjectURL(new Blob([serializePerformanceSession(session)], { type: "application/json" }));
    link.href = crashUrl;
    link.download = filename(session);
    link.textContent = "Download crash log (JSON)";
    crashPanel.append(heading, link);
    const target = document.querySelector("#status");
    if (target) target.insertAdjacentElement("afterend", crashPanel);
    else document.body.append(crashPanel);
    // Keep the link and object URL alive: an automatic download can be blocked by browser policy.
    if (automatic) {
      try { link.click(); }
      catch (error) { console.warn("Automatic crash-log download failed; use the download link", error); }
    }
  }

  log.download = session => {
    const url = URL.createObjectURL(new Blob([serializePerformanceSession(session)], { type: "application/json" }));
    const link = document.createElement("a");
    link.href = url;
    link.download = filename(session);
    document.body.append(link);
    try { link.click(); }
    finally { link.remove(); setTimeout(() => URL.revokeObjectURL(url), 1_000); }
  };

  window.addEventListener("error", event => {
    if (event instanceof ErrorEvent) {
      log.fail(event.error ?? new Error(event.message), {
        source: "window.error", filename: event.filename, line: event.lineno, column: event.colno,
      });
    }
  });
  window.addEventListener("unhandledrejection", event => {
    log.fail(event.reason, { source: "unhandledrejection" });
  });
  window.addEventListener("pagehide", event => {
    if (event.persisted) return; // BFCache restores this document and its download link.
    if (crashUrl) URL.revokeObjectURL(crashUrl);
    crashUrl = null;
  });
  // A caught engine failure survives reload. A killed browser process is not a catchable error.
  try {
    const saved = JSON.parse(sessionStorage.getItem(storageKey) ?? "null");
    if (saved?.kind === "physics-engine-browser-session" && saved.outcome === "crashed" && saved.failure) {
      window.physicsCrashLog = saved;
      publishCrash(saved, false);
    }
  } catch (error) { console.warn("Previous crash log could not be restored", error); }
  log.begin("initialization");
  return log;
}

// Explicit page-wide diagnostics owner, imported before settings and application initialization.
export const scenarioLog = typeof window === "undefined" ? null : createBrowserScenarioLog();
