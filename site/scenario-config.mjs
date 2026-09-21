export const SCENARIOS = Object.freeze({
  sandbox: Object.freeze({
    id: 0,
    title: "General sandbox",
    level: 3,
    levelLabel: "Systems",
    summary: "A mixed interactive world combining character motion, crates, projectiles, pair policies, fixed geometry, and performance evidence.",
    help: "WASD move · Space jump · Mouse / arrows look · Click / F shoot · 1/2/3 projectile · R reset · P pause · N step",
  }),
  "ccd-gauntlet": Object.freeze({
    id: 1,
    title: "CCD gauntlet",
    level: 1,
    levelLabel: "Focused",
    summary: "Four projectile lanes use progressively thinner fixed targets to make continuous collision detection and tunnelling failures immediately visible.",
    help: "Strafe between lanes with A/D · Aim at a thin target · Click / F shoot · 1/2/3 compares projectile paths",
  }),
  "collision-query-lab": Object.freeze({
    id: 2,
    title: "Collision query lab",
    level: 1,
    levelLabel: "Focused",
    summary: "An engine-owned overlap-only probe moves through fixed targets. Highlighted bodies are results of the physics query, not browser-side geometry tests.",
    help: "Watch the highlighted overlap probe and targets · WASD moves the viewer · R resets the deterministic pass",
  }),
  "off-centre-impact": Object.freeze({
    id: 3,
    title: "Off-centre impact",
    level: 1,
    levelLabel: "Focused",
    summary: "Shoot the tall target at different points to compare central linear impulse with torque-producing off-centre impacts.",
    help: "Aim at the centre, edge, or upper corner · Click / F shoot · Reset with R to compare exact starting state",
  }),
  "rotating-box-lab": Object.freeze({
    id: 4,
    title: "Rotating box lab",
    level: 2,
    levelLabel: "Coupled",
    summary: "Long cuboids begin with different angular velocities and collide with the floor, pillars, and each other through the rotating OBB path.",
    help: "Observe the spinning bodies · Walk around with WASD · P pauses · N advances one deterministic step",
  }),
  "tower-stability": Object.freeze({
    id: 5,
    title: "Tower stability",
    level: 2,
    levelLabel: "Coupled",
    summary: "A compact rigid-body tower settles under gravity and friction, then can be disturbed with projectiles to expose contact and stabilization behavior.",
    help: "Let the tower settle · Shoot a chosen level to disturb it · P pauses · N inspects contact progression",
  }),
  "sleeping-world": Object.freeze({
    id: 6,
    title: "Sleeping world",
    level: 3,
    levelLabel: "Systems",
    summary: "A local active sphere runs beside a settled population, demonstrating that dormant bodies remain part of the world without all becoming active work.",
    help: "Watch active motion against the settled grid · Use the debug strip to compare total and sleeping body counts",
  }),
});

export function scenarioForKey(key) {
  return SCENARIOS[key] ?? null;
}
