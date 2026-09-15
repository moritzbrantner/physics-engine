export const EXPLICIT_RULES_BIT = 1 << 29;
export const CHARACTER_LINEAR_BIT = 1;
export const CRATE_UPRIGHT_BIT = 1 << 11;

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

const PAIR_KEYS = new Set(COLLISION_PAIRS.map(([key]) => key));

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

export function encodeScenarioRules({ characterResponse, crateMotion, enabledPairs }) {
  let encoded = EXPLICIT_RULES_BIT;
  if (characterResponse === "linear") encoded |= CHARACTER_LINEAR_BIT;
  if (crateMotion === "upright") encoded |= CRATE_UPRIGHT_BIT;
  for (const [key, bit] of COLLISION_PAIRS) {
    if (enabledPairs.has(key)) encoded |= bit;
  }
  return encoded;
}
