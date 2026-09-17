import {
  COLLISION_PAIRS,
  encodeScenarioRules,
  enabledPairsFromQuery,
  enabledPairsToQuery,
  projectileImpactPolicyFromQuery,
} from "./simulation-rules-config.mjs";

const legacyCharacter = document.querySelector("#character-mode");
const legacyUprightCrates = document.querySelector("#upright-crates");
const characterResponse = document.querySelector("#character-response");
const crateMotion = document.querySelector("#crate-motion");
const projectileImpactPolicy = document.querySelector("#projectile-impact-policy");
const fixedGeometry = document.querySelector("#fixed-geometry-mode");
const reset = document.querySelector("#reset");
const pairControls = new Map(
  [...document.querySelectorAll("[data-collision-pair]")].map((control) => [
    control.dataset.collisionPair,
    control,
  ]),
);

const initial = new URL(window.location.href);
characterResponse.value =
  initial.searchParams.get("response") ??
  (initial.searchParams.get("character") === "physical" ? "physical" : "linear");
crateMotion.value =
  initial.searchParams.get("crate-motion") ??
  (initial.searchParams.has("crates")
    ? initial.searchParams.get("crates") === "upright"
      ? "upright"
      : "free"
    : "free");
projectileImpactPolicy.value = projectileImpactPolicyFromQuery(
  initial.searchParams.get("projectile-impact"),
);
const enabledPairs = enabledPairsFromQuery(initial.searchParams.get("collisions"));
for (const [key] of COLLISION_PAIRS) {
  const control = pairControls.get(key);
  if (control) control.checked = enabledPairs.has(key);
}
legacyUprightCrates.checked = crateMotion.value === "upright";

function selectedPairs() {
  return new Set(
    COLLISION_PAIRS.map(([key]) => key).filter((key) => pairControls.get(key)?.checked),
  );
}

function encodedRules() {
  return encodeScenarioRules({
    characterResponse: characterResponse.value,
    crateMotion: crateMotion.value,
    projectileImpactPolicy: projectileImpactPolicy.value,
    enabledPairs: selectedPairs(),
  });
}

Object.defineProperty(legacyCharacter, "value", {
  configurable: true,
  get() {
    return String(encodedRules());
  },
  set(value) {
    if (String(value) === "0") characterResponse.value = "physical";
    if (String(value) === "1") characterResponse.value = "linear";
  },
});

function syncUrl() {
  legacyUprightCrates.checked = crateMotion.value === "upright";
  const url = new URL(window.location.href);
  url.searchParams.set("response", characterResponse.value);
  url.searchParams.set("crate-motion", crateMotion.value);
  url.searchParams.set("projectile-impact", projectileImpactPolicy.value);
  url.searchParams.set("collisions", enabledPairsToQuery(selectedPairs()));
  url.searchParams.set("character", characterResponse.value === "physical" ? "physical" : "linear");
  url.searchParams.set("crates", crateMotion.value === "upright" ? "upright" : "free");
  window.history.replaceState(null, "", url);
}

syncUrl();

const scenarioControls = [
  characterResponse,
  crateMotion,
  projectileImpactPolicy,
  ...pairControls.values(),
];
for (const control of scenarioControls) {
  control.addEventListener("keydown", (event) => event.stopPropagation());
  control.addEventListener("change", () => {
    syncUrl();
    reset.click();
  });
}

function syncDisabledState() {
  for (const control of scenarioControls) control.disabled = legacyCharacter.disabled;
}
const disabledObserver = new MutationObserver(syncDisabledState);
disabledObserver.observe(legacyCharacter, { attributes: true, attributeFilter: ["disabled"] });
syncDisabledState();

fixedGeometry.addEventListener("change", () => queueMicrotask(syncUrl));