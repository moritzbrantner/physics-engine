// Rapier oracle on WASM: invariants and a 1.10× perf gate against both Rapier figures (consumer
// pattern and bare) per scenario for both engine storage modes (Ordered, Indexed). Each engine's
// lone build is copied to target/size/<side>.wasm, which `coding-tooling performance size`
// judges against .performance/size.json. `--size-only` stops after those two builds (the base-branch
// size measurement); CARGO_TARGET_DIR lets that build share the PR checkout's dependency cache.
import { execFileSync } from "node:child_process";
import { copyFile, mkdir, readFile } from "node:fs/promises";

const target = `${process.env.CARGO_TARGET_DIR ?? "target"}/wasm32-unknown-unknown/release/physics_oracle.wasm`;
const build = (features) => {
  execFileSync("cargo", ["build", "-p", "physics-oracle", "--release", "--locked",
    "--target", "wasm32-unknown-unknown", "--lib", "--no-default-features", "--features", features],
    { stdio: "inherit" });
};

await mkdir("target/size", { recursive: true });
for (const side of ["engine", "rapier"]) {
  build(side);
  await copyFile(target, `target/size/${side}.wasm`);
}
if (process.argv.includes("--size-only")) process.exit(0);
build("engine,rapier");
const env = { oracle_now: () => performance.now() };
const { instance } = await WebAssembly.instantiate(await readFile(target), { env });
const e = instance.exports;
const name = (i) => new TextDecoder().decode(new Uint8Array(e.memory.buffer, e.oracle_name_ptr(i), e.oracle_name_len(i)));
const median = (v) => v.sort((a, b) => a - b)[v.length >> 1];

// Fail when the engine's WASM median is more than 10% slower than either Rapier figure.
const MARGIN = 1.1;
let failed = false;
console.log("scenario             engine µs/tick indexed µs/tick rapier µs/tick   ratio  rapier raw  invariants");
for (let i = 0; i < e.oracle_scenarios(); i++) {
  const ok = e.oracle_check(i) === 0;
  failed ||= !ok;
  const hasRaw = e.oracle_has_raw(i) === 1;
  const engine = [], indexed = [], rapier = [], bare = [];
  for (let s = 0; s < 11; s++) {
    engine.push(e.oracle_time(i, 0));
    indexed.push(e.oracle_time(i, 3));
    rapier.push(e.oracle_time(i, 1));
    if (hasRaw) bare.push(e.oracle_time(i, 2));
  }
  const [pe, px, ra] = [median(engine), median(indexed), median(rapier)];
  const raw = hasRaw ? median(bare) : null;
  const slow = [pe, px].some((t) => t > ra * MARGIN || (raw !== null && t > raw * MARGIN));
  failed ||= slow;
  console.log(`${name(i).padEnd(20)} ${pe.toFixed(2).padStart(14)} ${px.toFixed(2).padStart(15)} ${ra.toFixed(2).padStart(14)} ${(pe / ra).toFixed(2).padStart(7)} ${(raw === null ? "-" : raw.toFixed(2)).padStart(11)}  ${ok ? "ok" : "FAILED"}${slow ? " SLOWER THAN RAPIER" : ""}`);
}
process.exit(failed ? 1 : 0);
