import assert from "node:assert/strict";
import test from "node:test";

import {
  CHARACTER_LINEAR_BIT,
  COLLISION_PAIRS,
  CRATE_UPRIGHT_BIT,
  EXPLICIT_RULES_BIT,
  encodeScenarioRules,
  enabledPairsFromQuery,
  enabledPairsToQuery,
} from "./simulation-rules-config.mjs";

test("default puzzle-friendly rules keep all collision pairs and free crate rotation", () => {
  const enabledPairs = enabledPairsFromQuery("all");
  const encoded = encodeScenarioRules({
    characterResponse: "linear",
    crateMotion: "free",
    enabledPairs,
  });
  assert.notEqual(encoded & EXPLICIT_RULES_BIT, 0);
  assert.notEqual(encoded & CHARACTER_LINEAR_BIT, 0);
  assert.equal(encoded & CRATE_UPRIGHT_BIT, 0);
  assert.equal(enabledPairsToQuery(enabledPairs), "all");
  assert.equal(enabledPairs.size, COLLISION_PAIRS.length);
});

test("collision query round-trips deterministic subsets", () => {
  const enabled = enabledPairsFromQuery("character-crate,crate-projectile");
  assert.deepEqual([...enabled], ["character-crate", "crate-projectile"]);
  assert.equal(enabledPairsToQuery(enabled), "character-crate,crate-projectile");
});

test("response axes remain independent", () => {
  const encoded = encodeScenarioRules({
    characterResponse: "physical",
    crateMotion: "upright",
    enabledPairs: enabledPairsFromQuery("none"),
  });
  assert.equal(encoded & CHARACTER_LINEAR_BIT, 0);
  assert.notEqual(encoded & CRATE_UPRIGHT_BIT, 0);
  assert.equal(enabledPairsToQuery(new Set()), "none");
});
