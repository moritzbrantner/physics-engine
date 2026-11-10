// Rapier oracle on WASM: invariants and a 1.10× perf gate against both Rapier figures (consumer
// pattern and bare) per scenario for both engine storage modes (Ordered, Indexed). Each engine's
// lone build is copied to target/size/<side>.wasm, which `coding-tooling performance size`
// judges against .performance/size.json. `--size-only` stops after those two builds (the base-branch
// size measurement), which size-budget.sh builds in its own CARGO_TARGET_DIR.
import { execFileSync } from "node:child_process";
import { copyFile, mkdir, readFile } from "node:fs/promises";

const target = `${process.env.CARGO_TARGET_DIR ?? "target"}/wasm32-unknown-unknown/release/physics_oracle.wasm`;
const build = (features) => {
  execFileSync("cargo", ["build", "-p", "physics-oracle", "--release", "--locked",
    "--target", "wasm32-unknown-unknown", "--lib", "--no-default-features", "--features", features],
    { stdio: "inherit" });
};

const onlyIndex = process.argv.indexOf("--only");
const only = onlyIndex < 0 ? null : process.argv[onlyIndex + 1];
if (onlyIndex >= 0 && (!only || only.startsWith("--") || process.argv.includes("--size-only"))) {
  throw new Error("usage: node scripts/oracle-wasm.mjs [--size-only | --only SCENARIO]");
}

// The focused mode is only a correctness/performance comparison; size budget
// tooling needs both standalone WASM builds and always runs the full suite.
if (only === null) {
  await mkdir("target/size", { recursive: true });
  for (const side of ["engine", "rapier"]) {
    build(side);
    await copyFile(target, `target/size/${side}.wasm`);
  }
}
if (process.argv.includes("--size-only")) process.exit(0);
build("engine,rapier");
const env = { oracle_now: () => performance.now() };
const { instance } = await WebAssembly.instantiate(await readFile(target), { env });
const e = instance.exports;
const name = (i) => new TextDecoder().decode(new Uint8Array(e.memory.buffer, e.oracle_name_ptr(i), e.oracle_name_len(i)));
const median = (v) => v.sort((a, b) => a - b)[v.length >> 1];
// One sample is the mean µs/tick of back-to-back runs (each on a freshly built world, build
// untimed) spanning at least this long, so a scenario of a few dozen microseconds is not
// timed from a single run.
const SAMPLE_MS = 5;
const WARM_UP_MS = 100;
const sample = (i, side) => {
  const start = performance.now();
  let sum = 0;
  let runs = 0;
  do {
    sum += e.oracle_time(i, side);
    runs++;
  } while (performance.now() - start < SAMPLE_MS);
  return sum / runs;
};

// Fail when the engine's WASM median is more than 10% slower than either Rapier figure.
const MARGIN = 1.1;
let failed = false;
let selected = 0;
console.log("scenario             engine µs/tick indexed µs/tick rapier µs/tick   ratio  rapier raw  invariants");
for (let i = 0; i < e.oracle_scenarios(); i++) {
  if (only !== null && name(i) !== only) continue;
  selected++;
  const ok = e.oracle_check(i) === 0;
  failed ||= !ok;
  const hasRaw = e.oracle_has_raw(i) === 1;
  const engine = [], indexed = [], rapier = [], bare = [];
  const sides = hasRaw ? [0, 3, 1, 2] : [0, 3, 1];
  // Untimed warm-up long enough for V8 to tier the hot functions up from its Liftoff baseline
  // compiler, so samples time optimized code: short scenarios were otherwise mostly timed
  // unoptimized (#549: brook-mesh-edge 9.3 µs/tick that way, about 2 µs/tick warmed up).
  // Disabling Liftoff instead crashes V8's speculative inlining on current Node LTS.
  for (const side of sides) {
    const start = performance.now();
    do e.oracle_time(i, side);
    while (performance.now() - start < WARM_UP_MS);
  }
  for (let s = 0; s < 11; s++) {
    engine.push(sample(i, 0));
    indexed.push(sample(i, 3));
    rapier.push(sample(i, 1));
    if (hasRaw) bare.push(sample(i, 2));
  }
  const [pe, px, ra] = [median(engine), median(indexed), median(rapier)];
  const raw = hasRaw ? median(bare) : null;
  // Bare Rapier gates too unless the scenario reports it only (`Scenario::bare_gated`).
  const gatedRaw = raw !== null && e.oracle_bare_gated(i) === 1 ? raw : null;
  const slow = [pe, px].some((t) => t > ra * MARGIN || (gatedRaw !== null && t > gatedRaw * MARGIN));
  failed ||= slow;
  console.log(`${name(i).padEnd(20)} ${pe.toFixed(2).padStart(14)} ${px.toFixed(2).padStart(15)} ${ra.toFixed(2).padStart(14)} ${(pe / ra).toFixed(2).padStart(7)} ${(raw === null ? "-" : raw.toFixed(2)).padStart(11)}  ${ok ? "ok" : "FAILED"}${slow ? " SLOWER THAN RAPIER" : ""}`);
}
if (only !== null && selected === 0) throw new Error(`unknown WASM oracle scenario: ${only}`);
process.exit(failed ? 1 : 0);
