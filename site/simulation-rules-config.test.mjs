import assert from "node:assert/strict";
import test from "node:test";

import {
  COLLISION_PAIRS,
  INTERACTION_POLICY_PACK_BIT,
  STABILIZATION_PAIRS,
  decodeInteractionPolicyPack,
  encodeInteractionPolicyPack,
  encodeScenarioRules,
  enabledPairsFromQuery,
  enabledPairsToQuery,
  projectileImpactPolicyFromQuery,
  stabilizationFromQuery,
  stabilizationToQuery,
} from "./simulation-rules-config.mjs";

test("collision-pair query round-trips deterministic pair order", () => {
  const selected = new Set(["world-character", "crate-crate", "world-world"]);
  const query = enabledPairsToQuery(selected);
  assert.equal(query, "world-world,world-character,crate-crate");
  assert.deepEqual(enabledPairsFromQuery(query), selected);
});

test("collision pair all and none use compact stable spellings", () => {
  const all = new Set(COLLISION_PAIRS.map(([key]) => key));
  assert.equal(enabledPairsToQuery(all), "all");
  assert.equal(enabledPairsToQuery(new Set()), "none");
  assert.deepEqual(enabledPairsFromQuery("all"), all);
  assert.deepEqual(enabledPairsFromQuery("none"), new Set());
});

test("scenario rules encode collision eligibility and projectile response separately", () => {
  const encoded = encodeScenarioRules({
    characterResponse: "linear",
    crateMotion: "upright",
    enabledPairs: new Set(["world-character", "world-projectile"]),
    projectileImpactPolicy: "impact-retire",
  });

  assert.equal(encoded & 1, 1);
  assert.equal(encoded & (1 << 11), 1 << 11);
  assert.equal(encoded & (1 << 2), 1 << 2);
  assert.equal(encoded & (1 << 4), 1 << 4);
  assert.equal(encoded & (1 << 3), 0);
  assert.equal((encoded >> 12) & 0b11, 2);
  assert.equal(encoded & (1 << 14), 1 << 14);
});

test("unknown projectile policy query falls back to single-impact behavior", () => {
  assert.equal(projectileImpactPolicyFromQuery("inelastic"), "inelastic");
  assert.equal(projectileImpactPolicyFromQuery("nonsense"), "impact-retire");
});

test("scenario rules default to impact-and-retire projectiles", () => {
  const encoded = encodeScenarioRules({
    characterResponse: "linear",
    crateMotion: "free",
    enabledPairs: new Set(COLLISION_PAIRS.map(([key]) => key)),
  });

  assert.equal((encoded >> 12) & 0b11, 2);
  assert.equal(encoded & (1 << 14), 1 << 14);
});

test("stabilization query keeps only supported pairs and pass choices", () => {
  const parsed = stabilizationFromQuery(
    "world-crate:2,world-projectile:0,crate-crate:4,world-character:99",
  );
  assert.equal(parsed.get("world-character"), "default");
  assert.equal(parsed.get("world-crate"), "2");
  assert.equal(parsed.get("world-projectile"), "0");
  assert.equal(stabilizationToQuery(parsed), "world-crate:2,world-projectile:0");
});

test("interaction policy pack round-trips all active fixed-world pairs", () => {
  const selected = new Map([
    ["world-character", "4"],
    ["world-crate", "0"],
    ["world-projectile", "16"],
  ]);
  const encoded = encodeInteractionPolicyPack(selected);
  assert.equal(encoded, INTERACTION_POLICY_PACK_BIT | 5 | (1 << 5) | (17 << 10));
  assert.deepEqual([...decodeInteractionPolicyPack(encoded)], [...selected]);
});

test("default interaction policy pack carries an explicit compatibility marker", () => {
  const defaults = new Map(STABILIZATION_PAIRS.map((key) => [key, "default"]));
  const encoded = encodeInteractionPolicyPack(defaults);
  assert.equal(encoded, INTERACTION_POLICY_PACK_BIT);
  assert.deepEqual([...decodeInteractionPolicyPack(encoded)], [...defaults]);
});

test("invalid interaction policy codes fail closed", () => {
  assert.throws(() => decodeInteractionPolicyPack(INTERACTION_POLICY_PACK_BIT | (18 << 10)), /invalid stabilization code/);
});
