import { createSettingsSession } from "./vendor/settings/settings-browser.js";
import {
  COLLISION_PAIRS,
  PROJECTILE_IMPACT_POLICIES,
  STABILIZATION_PAIRS,
  STABILIZATION_PASS_CHOICES,
  encodeInteractionPolicyPack,
  encodeScenarioRules,
  enabledPairsFromQuery,
  enabledPairsToQuery,
  stabilizationFromQuery,
  stabilizationToQuery,
} from "./simulation-rules-config.mjs";

const STORAGE_KEYS = Object.freeze({
  save: "physics-engine.settings.save.v1",
  device: "physics-engine.settings.device.v1",
});

const collisionSettingId = (pair) => `interaction.${pair.replaceAll("-", "_")}.collision`;
const stabilizationSettingId = (pair) =>
  `interaction.${pair.replaceAll("-", "_")}.stabilization_passes`;

const SETTINGS_DEFINITIONS = [
  {
    id: "simulation.character_response",
    kind: { type: "choice", options: ["linear", "physical"] },
    default: { type: "choice", value: "linear" },
    scope: "save",
    apply_mode: "immediate",
  },
  {
    id: "simulation.crate_motion",
    kind: { type: "choice", options: ["free", "upright"] },
    default: { type: "choice", value: "free" },
    scope: "save",
    apply_mode: "immediate",
  },
  {
    id: "simulation.projectile_impact",
    kind: { type: "choice", options: [...PROJECTILE_IMPACT_POLICIES.keys()] },
    default: { type: "choice", value: "physical" },
    scope: "save",
    apply_mode: "immediate",
  },
  {
    id: "engine.fixed_geometry",
    kind: { type: "choice", options: ["runtime", "load"] },
    default: { type: "choice", value: "load" },
    scope: "device",
    apply_mode: "immediate",
  },
  {
    id: "display.fullscreen",
    kind: { type: "bool" },
    default: { type: "bool", value: false },
    scope: "session",
    apply_mode: "immediate",
  },
  ...COLLISION_PAIRS.map(([pair]) => ({
    id: collisionSettingId(pair),
    kind: { type: "bool" },
    default: { type: "bool", value: true },
    scope: "save",
    apply_mode: "immediate",
  })),
  ...STABILIZATION_PAIRS.map((pair) => ({
    id: stabilizationSettingId(pair),
    kind: { type: "choice", options: STABILIZATION_PASS_CHOICES },
    default: { type: "choice", value: "default" },
    scope: "save",
    apply_mode: "immediate",
  })),
];

const definitionById = new Map(SETTINGS_DEFINITIONS.map((definition) => [definition.id, definition]));
const status = document.querySelector("#status");

let session;
try {
  session = await createSettingsSession(SETTINGS_DEFINITIONS);
} catch (error) {
  status.textContent = `Unable to initialize shared settings: ${error.message}`;
  throw error;
}

function effectiveValue(id) {
  const value = session.effectiveValues()[id];
  if (!value) throw new Error(`settings session did not expose ${id}`);
  return value.value;
}

function setChoice(id, value) {
  session.set(id, { type: "choice", value });
}

function setBoolean(id, value) {
  session.set(id, { type: "bool", value });
}

function persistScope(scope) {
  const storageKey = STORAGE_KEYS[scope];
  if (!storageKey) return;
  try {
    localStorage.setItem(storageKey, session.exportScope(scope));
  } catch (error) {
    console.warn(`Unable to persist ${scope} settings`, error);
  }
}

function persistSetting(id) {
  const scope = definitionById.get(id)?.scope;
  if (scope === "save" || scope === "device") persistScope(scope);
}

function restoreScope(scope) {
  const storageKey = STORAGE_KEYS[scope];
  if (!storageKey) return;
  let snapshot;
  try {
    snapshot = localStorage.getItem(storageKey);
  } catch {
    return;
  }
  if (!snapshot) return;
  try {
    const diagnostics = session.importScope(scope, snapshot);
    if (diagnostics.length > 0) {
      console.warn(`Settings import for ${scope} reported diagnostics`, diagnostics);
    }
  } catch (error) {
    console.warn(`Ignoring invalid persisted ${scope} settings`, error);
  }
}

restoreScope("save");
restoreScope("device");

function applyUrlOverrides() {
  const url = new URL(window.location.href);

  const response =
    url.searchParams.get("response") ??
    (url.searchParams.has("character")
      ? url.searchParams.get("character") === "physical"
        ? "physical"
        : "linear"
      : null);
  if (response === "linear" || response === "physical") {
    setChoice("simulation.character_response", response);
  }

  const crateMotion =
    url.searchParams.get("crate-motion") ??
    (url.searchParams.has("crates")
      ? url.searchParams.get("crates") === "upright"
        ? "upright"
        : "free"
      : null);
  if (crateMotion === "free" || crateMotion === "upright") {
    setChoice("simulation.crate_motion", crateMotion);
  }

  const projectileImpact = url.searchParams.get("projectile-impact");
  if (projectileImpact && PROJECTILE_IMPACT_POLICIES.has(projectileImpact)) {
    setChoice("simulation.projectile_impact", projectileImpact);
  }

  if (url.searchParams.has("bake")) {
    setChoice(
      "engine.fixed_geometry",
      url.searchParams.get("bake") === "runtime" ? "runtime" : "load",
    );
  }

  if (url.searchParams.has("collisions")) {
    const enabledPairs = enabledPairsFromQuery(url.searchParams.get("collisions"));
    for (const [pair] of COLLISION_PAIRS) {
      setBoolean(collisionSettingId(pair), enabledPairs.has(pair));
    }
  }

  if (url.searchParams.has("stabilization")) {
    const selected = stabilizationFromQuery(url.searchParams.get("stabilization"));
    for (const pair of STABILIZATION_PAIRS) {
      setChoice(stabilizationSettingId(pair), selected.get(pair) ?? "default");
    }
  }
}

applyUrlOverrides();
persistScope("save");
persistScope("device");

function selectedCollisionPairs() {
  return new Set(
    COLLISION_PAIRS
      .map(([pair]) => pair)
      .filter((pair) => effectiveValue(collisionSettingId(pair))),
  );
}

function selectedStabilization() {
  return new Map(
    STABILIZATION_PAIRS.map((pair) => [pair, effectiveValue(stabilizationSettingId(pair))]),
  );
}

function syncUrl() {
  const url = new URL(window.location.href);
  const characterResponse = effectiveValue("simulation.character_response");
  const crateMotion = effectiveValue("simulation.crate_motion");
  const projectileImpact = effectiveValue("simulation.projectile_impact");
  const fixedGeometry = effectiveValue("engine.fixed_geometry");
  const stabilization = stabilizationToQuery(selectedStabilization());

  url.searchParams.set("response", characterResponse);
  url.searchParams.set("crate-motion", crateMotion);
  url.searchParams.set("projectile-impact", projectileImpact);
  url.searchParams.set("collisions", enabledPairsToQuery(selectedCollisionPairs()));
  url.searchParams.set("character", characterResponse === "physical" ? "physical" : "linear");
  url.searchParams.set("crates", crateMotion === "upright" ? "upright" : "free");
  url.searchParams.set("bake", fixedGeometry);
  if (stabilization) {
    url.searchParams.set("stabilization", stabilization);
  } else {
    url.searchParams.delete("stabilization");
  }
  window.history.replaceState(null, "", url);
}

const characterResponse = document.querySelector("#character-response");
const crateMotion = document.querySelector("#crate-motion");
const projectileImpact = document.querySelector("#projectile-impact-policy");
const fixedGeometrySetting = document.querySelector("#fixed-geometry-setting");
const fullscreenSetting = document.querySelector("#browser-fullscreen");
const fullscreenButton = document.querySelector("#fullscreen");
const resetButton = document.querySelector("#reset");
const resetInteractions = document.querySelector("#reset-interactions");
const settingsPanel = document.querySelector("#settings-panel");
const openSettings = document.querySelector("#open-settings");
const closeSettings = document.querySelector("#close-settings");
const settingsScrim = document.querySelector("#settings-scrim");
const legacyCharacter = document.querySelector("#character-mode");
const legacyInteractionPack = document.querySelector("#upright-crates");
const legacyFixedGeometry = document.querySelector("#fixed-geometry-mode");

const pairControls = new Map(
  [...document.querySelectorAll("[data-collision-pair]")].map((control) => [
    control.dataset.collisionPair,
    control,
  ]),
);
const stabilizationControls = new Map(
  [...document.querySelectorAll("[data-stabilization-pair]")].map((control) => [
    control.dataset.stabilizationPair,
    control,
  ]),
);

for (const [pair, control] of stabilizationControls) {
  for (const choice of STABILIZATION_PASS_CHOICES) {
    const option = document.createElement("option");
    option.value = choice;
    option.textContent =
      choice === "default"
        ? "Engine default (≤ 16)"
        : choice === "0"
          ? "0 passes · disable stage"
          : `${choice} ${choice === "1" ? "pass" : "passes"}`;
    control.append(option);
  }
  if (!STABILIZATION_PAIRS.includes(pair)) {
    throw new Error(`unsupported stabilization control: ${pair}`);
  }
}

function syncControls() {
  characterResponse.value = effectiveValue("simulation.character_response");
  crateMotion.value = effectiveValue("simulation.crate_motion");
  projectileImpact.value = effectiveValue("simulation.projectile_impact");
  fixedGeometrySetting.value = effectiveValue("engine.fixed_geometry");

  for (const [pair] of COLLISION_PAIRS) {
    const control = pairControls.get(pair);
    if (control) control.checked = effectiveValue(collisionSettingId(pair));
  }
  for (const pair of STABILIZATION_PAIRS) {
    const control = stabilizationControls.get(pair);
    if (control) control.value = effectiveValue(stabilizationSettingId(pair));
  }

  const fullscreen = document.fullscreenElement != null;
  fullscreenSetting.checked = fullscreen;
  fullscreenButton.textContent = fullscreen ? "Exit fullscreen" : "Fullscreen";
  fullscreenButton.setAttribute("aria-pressed", String(fullscreen));
}

function enableSettingsControls() {
  characterResponse.disabled = false;
  crateMotion.disabled = false;
  projectileImpact.disabled = false;
  fixedGeometrySetting.disabled = false;
  fullscreenSetting.disabled = !document.fullscreenEnabled;
  resetInteractions.disabled = false;
  for (const control of pairControls.values()) control.disabled = false;
  for (const control of stabilizationControls.values()) control.disabled = false;
}

function applyAndReset(id, value) {
  const definition = definitionById.get(id);
  if (!definition) throw new Error(`unknown setting ${id}`);
  if (definition.kind.type === "bool") {
    setBoolean(id, Boolean(value));
  } else if (definition.kind.type === "choice") {
    setChoice(id, String(value));
  } else {
    throw new Error(`unsupported physics settings kind ${definition.kind.type}`);
  }
  persistSetting(id);
  syncControls();
  syncUrl();
  resetButton.click();
}

characterResponse.addEventListener("change", () => {
  applyAndReset("simulation.character_response", characterResponse.value);
});
crateMotion.addEventListener("change", () => {
  applyAndReset("simulation.crate_motion", crateMotion.value);
});
projectileImpact.addEventListener("change", () => {
  applyAndReset("simulation.projectile_impact", projectileImpact.value);
});
fixedGeometrySetting.addEventListener("change", () => {
  applyAndReset("engine.fixed_geometry", fixedGeometrySetting.value);
});

for (const [pair, control] of pairControls) {
  control.addEventListener("change", () => {
    applyAndReset(collisionSettingId(pair), control.checked);
  });
}

for (const [pair, control] of stabilizationControls) {
  control.addEventListener("change", () => {
    applyAndReset(stabilizationSettingId(pair), control.value);
  });
}

resetInteractions.addEventListener("click", () => {
  for (const [pair] of COLLISION_PAIRS) {
    session.reset(collisionSettingId(pair));
  }
  for (const pair of STABILIZATION_PAIRS) {
    session.reset(stabilizationSettingId(pair));
  }
  persistScope("save");
  syncControls();
  syncUrl();
  resetButton.click();
});

function openSettingsPanel() {
  document.body.classList.add("settings-open");
  openSettings.setAttribute("aria-expanded", "true");
}

function closeSettingsPanel() {
  document.body.classList.remove("settings-open");
  openSettings.setAttribute("aria-expanded", "false");
}

openSettings.addEventListener("click", openSettingsPanel);
closeSettings.addEventListener("click", closeSettingsPanel);
settingsScrim.addEventListener("click", closeSettingsPanel);
settingsPanel.addEventListener("keydown", (event) => event.stopPropagation());

async function setBrowserFullscreen(enabled) {
  try {
    if (enabled && document.fullscreenElement == null) {
      await document.documentElement.requestFullscreen();
    } else if (!enabled && document.fullscreenElement != null) {
      await document.exitFullscreen();
    }
  } catch (error) {
    status.textContent = `Fullscreen request was not accepted: ${error.message}`;
  } finally {
    const fullscreen = document.fullscreenElement != null;
    setBoolean("display.fullscreen", fullscreen);
    syncControls();
  }
}

fullscreenButton.addEventListener("click", () => {
  void setBrowserFullscreen(document.fullscreenElement == null);
});
fullscreenSetting.addEventListener("change", () => {
  void setBrowserFullscreen(fullscreenSetting.checked);
});
document.addEventListener("fullscreenchange", () => {
  setBoolean("display.fullscreen", document.fullscreenElement != null);
  syncControls();
});

function encodedScenarioRules() {
  return encodeScenarioRules({
    characterResponse: effectiveValue("simulation.character_response"),
    crateMotion: effectiveValue("simulation.crate_motion"),
    enabledPairs: selectedCollisionPairs(),
    projectileImpactPolicy: effectiveValue("simulation.projectile_impact"),
  });
}

Object.defineProperty(legacyCharacter, "value", {
  configurable: true,
  get() {
    return String(encodedScenarioRules());
  },
  set() {},
});

Object.defineProperty(legacyInteractionPack, "checked", {
  configurable: true,
  get() {
    return encodeInteractionPolicyPack(selectedStabilization());
  },
  set() {},
});

Object.defineProperty(legacyFixedGeometry, "value", {
  configurable: true,
  get() {
    return effectiveValue("engine.fixed_geometry") === "runtime" ? "0" : "1";
  },
  set() {},
});

syncControls();
syncUrl();
enableSettingsControls();
