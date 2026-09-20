export const EXPLICIT_RULES_BIT = 1 << 29;
export const CHARACTER_LINEAR_BIT = 1;
export const CRATE_UPRIGHT_BIT = 1 << 11;
export const PROJECTILE_POLICY_SHIFT = 12;
export const PROJECTILE_POLICY_EXPLICIT_BIT = 1 << 14;

export const PROJECTILE_IMPACT_POLICIES = new Map([
  ["physical", 0],
  ["inelastic", 1],
  ["impact-retire", 2],
]);

export const COLLISION_PAIRS = [
  ["world-world", 1 << 1],
  ["world-character", 1 << 2],
  ["world-crate", 1 << 3],
  ["world-projectile", 1 << 4],
  ["character-character", 1 << 5],
  ["character-crate", 1 << 6],
  ["character-projectile", 1 << 7],
  ["crate-crate", 1 << 8],
  ["crate-projectile", 1 << 9],
  ["projectile-projectile", 1 << 10],
];

export const STABILIZATION_PAIRS = [
  "world-character",
  "world-crate",
  "world-projectile",
];

export const STABILIZATION_PASS_CHOICES = [
  "default",
  ...Array.from({ length: 17 }, (_, index) => String(index)),
];

const PAIR_KEYS = new Set(COLLISION_PAIRS.map(([key]) => key));
const STABILIZATION_PAIR_KEYS = new Set(STABILIZATION_PAIRS);
export const INTERACTION_POLICY_PACK_BIT = 1 << 30;
const STABILIZATION_BITS_PER_PAIR = 5;
const STABILIZATION_CODE_MASK = (1 << STABILIZATION_BITS_PER_PAIR) - 1;

export function enabledPairsFromQuery(value) {
  if (value == null || value === "all") return new Set(PAIR_KEYS);
  if (value === "none") return new Set();
  return new Set(value.split(",").filter((key) => PAIR_KEYS.has(key)));
}

export function enabledPairsToQuery(enabledPairs) {
  const enabled = COLLISION_PAIRS.map(([key]) => key).filter((key) => enabledPairs.has(key));
  if (enabled.length === COLLISION_PAIRS.length) return "all";
  if (enabled.length === 0) return "none";
  return enabled.join(",");
}

export function projectileImpactPolicyFromQuery(value) {
  return PROJECTILE_IMPACT_POLICIES.has(value) ? value : "impact-retire";
}

export function stabilizationFromQuery(value) {
  const selected = new Map(STABILIZATION_PAIRS.map((key) => [key, "default"]));
  if (!value) return selected;

  for (const entry of value.split(",")) {
    const [key, choice] = entry.split(":");
    if (!STABILIZATION_PAIR_KEYS.has(key) || !STABILIZATION_PASS_CHOICES.includes(choice)) {
      continue;
    }
    selected.set(key, choice);
  }
  return selected;
}

export function stabilizationToQuery(selected) {
  return STABILIZATION_PAIRS
    .map((key) => [key, selected.get(key) ?? "default"])
    .filter(([, choice]) => choice !== "default")
    .map(([key, choice]) => `${key}:${choice}`)
    .join(",");
}

export function encodeInteractionPolicyPack(selected) {
  let encoded = 0;

  for (const [index, key] of STABILIZATION_PAIRS.entries()) {
    const choice = selected.get(key) ?? "default";
    if (!STABILIZATION_PASS_CHOICES.includes(choice)) {
      throw new Error(`unknown stabilization pass choice for ${key}: ${choice}`);
    }
    const code = choice === "default" ? 0 : Number(choice) + 1;
    encoded |= (code & STABILIZATION_CODE_MASK) << (index * STABILIZATION_BITS_PER_PAIR);
  }

  return INTERACTION_POLICY_PACK_BIT | encoded;
}

export function decodeInteractionPolicyPack(encoded) {
  if (
    !Number.isInteger(encoded) ||
    encoded < 0 ||
    (encoded & INTERACTION_POLICY_PACK_BIT) === 0 ||
    (encoded & ~(INTERACTION_POLICY_PACK_BIT | ((1 << 15) - 1))) !== 0
  ) {
    throw new Error(`invalid interaction policy pack: ${encoded}`);
  }

  const payload = encoded & ((1 << 15) - 1);
  const selected = new Map();
  for (const [index, key] of STABILIZATION_PAIRS.entries()) {
    const code =
      (payload >> (index * STABILIZATION_BITS_PER_PAIR)) & STABILIZATION_CODE_MASK;
    if (code > 17) throw new Error(`invalid stabilization code ${code} for ${key}`);
    selected.set(key, code === 0 ? "default" : String(code - 1));
  }
  return selected;
}

export function encodeScenarioRules({
  characterResponse,
  crateMotion,
  enabledPairs,
  projectileImpactPolicy = "impact-retire",
}) {
  let encoded = EXPLICIT_RULES_BIT | PROJECTILE_POLICY_EXPLICIT_BIT;
  if (characterResponse === "linear") encoded |= CHARACTER_LINEAR_BIT;
  if (crateMotion === "upright") encoded |= CRATE_UPRIGHT_BIT;
  const projectilePolicy = PROJECTILE_IMPACT_POLICIES.get(projectileImpactPolicy);
  if (projectilePolicy === undefined) {
    throw new Error(`unknown projectile impact policy: ${projectileImpactPolicy}`);
  }
  encoded |= projectilePolicy << PROJECTILE_POLICY_SHIFT;
  for (const [key, bit] of COLLISION_PAIRS) {
    if (enabledPairs.has(key)) encoded |= bit;
  }
  return encoded;
}
