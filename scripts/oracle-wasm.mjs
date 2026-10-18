// Measures the Rapier oracle scenarios on WASM (never gated): per-scenario µs/tick for
// physics-engine and Rapier, plus the WASM size of each engine's build.
import { execFileSync } from "node:child_process";
import { readFile, stat } from "node:fs/promises";

const manifest = "oracle/Cargo.toml";
const target = "oracle/target/wasm32-unknown-unknown/release/physics_oracle.wasm";
const build = (features) => {
  execFileSync("cargo", ["build", "--manifest-path", manifest, "--release", "--locked",
    "--target", "wasm32-unknown-unknown", "--lib", "--no-default-features", "--features", features],
    { stdio: "inherit" });
  return stat(target).then((s) => s.size);
};

const sizes = { engine: await build("engine"), rapier: await build("rapier") };
await build("engine,rapier");
const env = { oracle_now: () => performance.now() };
const { instance } = await WebAssembly.instantiate(await readFile(target), { env });
const e = instance.exports;
const name = (i) => new TextDecoder().decode(new Uint8Array(e.memory.buffer, e.oracle_name_ptr(i), e.oracle_name_len(i)));
const median = (v) => v.sort((a, b) => a - b)[v.length >> 1];

let failed = false;
console.log("scenario             engine µs/tick rapier µs/tick   ratio  rapier raw  invariants");
for (let i = 0; i < e.oracle_scenarios(); i++) {
  const ok = e.oracle_check(i) === 0;
  failed ||= !ok;
  const engine = [], rapier = [];
  for (let s = 0; s < 11; s++) { engine.push(e.oracle_time(i, 0)); rapier.push(e.oracle_time(i, 1)); }
  const [pe, ra] = [median(engine), median(rapier)];
  const raw = e.oracle_has_raw(i) ? median(Array.from({ length: 11 }, () => e.oracle_time(i, 2))).toFixed(2) : "-";
  console.log(`${name(i).padEnd(20)} ${pe.toFixed(2).padStart(14)} ${ra.toFixed(2).padStart(14)} ${(pe / ra).toFixed(2).padStart(7)} ${raw.padStart(11)}  ${ok ? "ok" : "FAILED"}`);
}
console.log(`WASM size: engine ${(sizes.engine / 1e6).toFixed(2)} MB, rapier ${(sizes.rapier / 1e6).toFixed(2)} MB (scenario workloads included)`);
// WASM timings are reported, not gated; invariants must still hold on WASM.
process.exit(failed ? 1 : 0);
