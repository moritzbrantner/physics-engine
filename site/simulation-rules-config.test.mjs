import assert from "node:assert/strict";
import test from "node:test";

import {
  CHARACTER_LINEAR_BIT,
  COLLISION_PAIRS,
  CRATE_UPRIGHT_BIT,
  EXPLICIT_RULES_BIT,
  PROJECTILE_POLICY_EXPLICIT_BIT,
  PROJECTILE_POLICY_SHIFT,
  encodeScenarioRules,
  enabledPairsFromQuery,
  enabledPairsToQuery,
  projectileImpactPolicyFromQuery,
} from "./simulation-rules-config.mjs";

test("default puzzle-friendly rules keep all collision pairs, free crates, and physical projectiles", () => {
  const enabledPairs = enabledPairsFromQuery("all");
  const encoded = encodeScenarioRules({
    characterResponse: "linear",
    crateMotion: "free",
    enabledPairs,
    projectileImpactPolicy: "physical",
  });
  assert.notEqual(encoded & EXPLICIT_RULES_BIT, 0);
  assert.notEqual(encoded & CHARACTER_LINEAR_BIT, 0);
  assert.equal(encoded & CRATE_UPRIGHT_BIT, 0);
  assert.notEqual(encoded & PROJECTILE_POLICY_EXPLICIT_BIT, 0);
  assert.equal((encoded >> PROJECTILE_POLICY_SHIFT) & 0b11, 0);
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
    projectileImpactPolicy: "impact-retire",
  });
  assert.equal(encoded & CHARACTER_LINEAR_BIT, 0);
  assert.notEqual(encoded & CRATE_UPRIGHT_BIT, 0);
  assert.equal((encoded >> PROJECTILE_POLICY_SHIFT) & 0b11, 2);
  assert.equal(enabledPairsToQuery(new Set()), "none");
});

test("projectile impact query values are canonical and invalid values fall back to physical", () => {
  assert.equal(projectileImpactPolicyFromQuery("inelastic"), "inelastic");
  assert.equal(projectileImpactPolicyFromQuery("impact-retire"), "impact-retire");
  assert.equal(projectileImpactPolicyFromQuery("unknown"), "physical");
  assert.equal(projectileImpactPolicyFromQuery(null), "physical");
});

test("unknown projectile policies are rejected by the encoder", () => {
  assert.throws(
    () =>
      encodeScenarioRules({
        characterResponse: "linear",
        crateMotion: "free",
        enabledPairs: enabledPairsFromQuery("all"),
        projectileImpactPolicy: "rubber",
      }),
    /unknown projectile impact policy/,
  );
});