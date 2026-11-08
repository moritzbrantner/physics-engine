// #545 solver comparison driver. `build` compiles every variant, `check` runs the invariants and
// trace identity, `measure` appends samples to target/solver-lab/samples.jsonl and `report` prints
// paired statistics. Variants run in a fresh random order every round, each in its own process,
// so slow load drift lands on all of them; each ratio is taken within one round.
import { execFileSync, spawnSync } from "node:child_process";
import { appendFileSync, copyFileSync, existsSync, mkdirSync, readFileSync, readdirSync, rmSync } from "node:fs";

const out = "target/solver-lab";
const VARIANTS = ["scalar", "hot", "manifold", "colored"];
const featureOf = (v) => (v === "scalar" ? [] : [`solver-${v}`]);
const SMALL = ["rubble-wake", "debris", "debris-town-tumble", "mmorpg-tick", "crowd", "pile-16", "pile-128", "sparse-4"];
const LARGE = ["pile-1024", "pile-4096"];
const CPUS = "4-7";
mkdirSync(out, { recursive: true });

const cargo = (args, env = {}) =>
  execFileSync("cargo", args, { stdio: ["ignore", "ignore", "inherit"], env: { ...process.env, ...env } });

function buildNative() {
  for (const v of VARIANTS) {
    for (const phase of [false, true]) {
      const features = ["engine", "rapier", ...featureOf(v), ...(phase ? ["phase-timing"] : [])].join(",");
      cargo(["build", "--release", "-p", "physics-oracle", "--no-default-features", "--features", features, "--bins"]);
      copyFileSync("target/release/solver-lab", `${out}/native-${v}${phase ? "-phase" : ""}`);
      if (!phase) copyFileSync("target/release/oracle", `${out}/oracle-${v}`);
    }
  }
}

function buildWasm() {
  for (const simd of [false, true]) {
    const env = { CARGO_TARGET_DIR: `target/solver-lab/wasm-${simd ? "simd" : "nosimd"}` };
    if (simd) env.RUSTFLAGS = "-C target-feature=+simd128";
    for (const v of VARIANTS) {
      for (const [suffix, extra] of [["", []], ["-phase", ["phase-timing"]], ["-engine", null]]) {
        const features = extra === null
          ? ["engine", ...featureOf(v)]
          : ["engine", "rapier", ...featureOf(v), ...extra];
        cargo(["build", "-p", "physics-oracle", "--release", "--target", "wasm32-unknown-unknown", "--lib",
          "--no-default-features", "--features", features.join(",")], env);
        copyFileSync(`${env.CARGO_TARGET_DIR}/wasm32-unknown-unknown/release/physics_oracle.wasm`,
          `${out}/wasm-${simd ? "simd" : "nosimd"}-${v}${suffix}.wasm`);
      }
    }
  }
}

async function wasmInstance(file) {
  const env = { oracle_now: () => performance.now() };
  const { instance } = await WebAssembly.instantiate(readFileSync(file), { env });
  return instance.exports;
}
const labName = (e, i) => {
  const len = e.lab_name_len(i);
  return new TextDecoder().decode(new Uint8Array(e.memory.buffer, e.lab_name_ptr(), len));
};

function check() {
  const dumps = {};
  for (const v of VARIANTS) {
    const r = spawnSync(`${out}/oracle-${v}`, ["--no-timing", "--keep-going"], { encoding: "utf8" });
    const failed = r.stdout.split("\n").filter((l) => l.includes("invariant failed"));
    console.log(`native ${v}: ${r.status === 0 ? "all invariants pass" : `${failed.length} failing`}`);
    for (const line of failed) console.log(`  ${line.slice(0, 200)}`);
    const dir = `${out}/dump-${v}`;
    rmSync(dir, { recursive: true, force: true });
    spawnSync(`${out}/oracle-${v}`, ["--no-timing", "--keep-going", "--dump", dir], { encoding: "utf8" });
    dumps[v] = dir;
  }
  for (const v of VARIANTS.slice(1)) {
    const files = readdirSync(dumps.scalar).filter((f) => f.endsWith(".engine.txt"));
    const different = files.filter((f) => !existsSync(`${dumps[v]}/${f}`)
      || !readFileSync(`${dumps.scalar}/${f}`).equals(readFileSync(`${dumps[v]}/${f}`)));
    console.log(`${v}: ${files.length - different.length}/${files.length} engine traces bit-identical to scalar`);
  }
}

async function checkWasm() {
  for (const simd of ["nosimd", "simd"]) {
    for (const v of VARIANTS) {
      const e = await wasmInstance(`${out}/wasm-${simd}-${v}.wasm`);
      const failed = [];
      for (let i = 0; i < e.oracle_scenarios(); i++) {
        if (e.oracle_check(i) !== 0) {
          failed.push(new TextDecoder().decode(new Uint8Array(e.memory.buffer, e.oracle_name_ptr(i), e.oracle_name_len(i))));
        }
      }
      console.log(`wasm ${simd} ${v}: ${failed.length === 0 ? "all invariants pass" : `failing: ${failed.join(", ")}`}`);
    }
  }
}

const shuffled = (items) => {
  const a = [...items];
  for (let i = a.length - 1; i > 0; i--) {
    const j = Math.floor(Math.random() * (i + 1));
    [a[i], a[j]] = [a[j], a[i]];
  }
  return a;
};
const record = (row) => appendFileSync(`${out}/samples.jsonl`, `${JSON.stringify(row)}\n`);

function measureNative(config, round) {
  const run = (samples, names) => {
    const r = spawnSync("taskset", ["-c", CPUS, `${out}/${config}`, String(samples), ...names], { encoding: "utf8" });
    if (r.status !== 0) throw new Error(`${config}: ${r.stderr}`);
    for (const line of r.stdout.trim().split("\n")) record({ config, round, ...JSON.parse(line) });
  };
  run(5, SMALL);
  run(1, LARGE);
}

function measureWasm(config, round) {
  const r = spawnSync("taskset", ["-c", CPUS, process.execPath, process.argv[1], "wasm-worker", config, String(round)],
    { stdio: ["ignore", "ignore", "inherit"] });
  if (r.status !== 0) throw new Error(`${config} failed`);
}

async function wasmWorker(config, round) {
  const e = await wasmInstance(`${out}/${config}.wasm`);
  for (let i = 0; i < e.lab_workloads(); i++) {
    const workload = labName(e, i);
    const samples = LARGE.includes(workload) ? 1 : 5;
    for (let s = 0; s < samples; s++) {
      const us = e.lab_run(i);
      record({ config, round, workload, us, rows: e.lab_phase(0), solve: e.lab_phase(1), prepare: e.lab_phase(2) });
    }
  }
}

const quantile = (values, q) => {
  const v = [...values].sort((a, b) => a - b);
  const at = (v.length - 1) * q;
  const lo = Math.floor(at);
  return v[lo] + (v[Math.min(lo + 1, v.length - 1)] - v[lo]) * (at - lo);
};

function report() {
  const rows = readFileSync(`${out}/samples.jsonl`, "utf8").trim().split("\n").map(JSON.parse);
  const key = (r) => `${r.config}\0${r.workload}\0${r.round}`;
  const best = new Map();
  for (const r of rows) {
    const k = key(r);
    if (!best.has(k) || r.us < best.get(k).us) best.set(k, r);
  }
  const groups = [
    ["native", "native-scalar", ["native-scalar", "native-hot", "native-manifold", "native-colored"]],
    ["wasm", "wasm-nosimd-scalar", ["wasm-nosimd-scalar", "wasm-simd-scalar", "wasm-nosimd-hot", "wasm-simd-hot",
      "wasm-nosimd-manifold", "wasm-simd-manifold", "wasm-nosimd-colored", "wasm-simd-colored"]],
  ];
  const workloads = [...new Set(rows.map((r) => r.workload))];
  for (const [title, reference, configs] of groups) {
    console.log(`\n## ${title}: µs/tick (best of a round), and paired ratio to ${reference}\n`);
    console.log("| workload | config | rounds | median µs | min µs | spread (p90/p10) | ratio median | ratio p10 | ratio p90 |");
    console.log("| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |");
    for (const workload of workloads) {
      for (const config of configs) {
        const series = [...best.values()].filter((r) => r.workload === workload && r.config === config);
        if (series.length === 0) continue;
        const us = series.map((r) => r.us);
        const ratios = series.flatMap((r) => {
          const b = best.get(`${reference}\0${workload}\0${r.round}`);
          return b ? [r.us / b.us] : [];
        });
        console.log(`| ${workload} | ${config} | ${series.length} | ${quantile(us, 0.5).toFixed(2)} | ${Math.min(...us).toFixed(2)} | ${(quantile(us, 0.9) / quantile(us, 0.1)).toFixed(3)} | ${quantile(ratios, 0.5).toFixed(3)} | ${quantile(ratios, 0.1).toFixed(3)} | ${quantile(ratios, 0.9).toFixed(3)} |`);
      }
    }
  }
  console.log("\n## phases: µs/tick, median over rounds, from the phase-timing builds\n");
  console.log("| workload | config | rounds | rows | solve | of which prepare |");
  console.log("| --- | --- | ---: | ---: | ---: | ---: |");
  for (const workload of workloads) {
    const configs = [...new Set(rows.filter((r) => r.config.endsWith("-phase")).map((r) => r.config))].sort();
    for (const config of configs) {
      const series = [...best.values()].filter((r) => r.workload === workload && r.config === config);
      if (series.length === 0) continue;
      console.log(`| ${workload} | ${config} | ${series.length} | ${quantile(series.map((r) => r.rows), 0.5).toFixed(2)} | ${quantile(series.map((r) => r.solve), 0.5).toFixed(2)} | ${quantile(series.map((r) => r.prepare), 0.5).toFixed(2)} |`);
    }
  }
}

function sizes() {
  console.log("| wasm build (engine only) | bytes |\n| --- | ---: |");
  for (const simd of ["nosimd", "simd"]) {
    for (const v of VARIANTS) {
      const file = `${out}/wasm-${simd}-${v}-engine.wasm`;
      console.log(`| ${simd} ${v} | ${readFileSync(file).length} |`);
    }
  }
}

const [command, a, b] = process.argv.slice(2);
if (command === "build") {
  buildNative();
  buildWasm();
} else if (command === "check") {
  check();
  await checkWasm();
} else if (command === "wasm-worker") {
  await wasmWorker(a, Number(b));
} else if (command === "measure") {
  const nativeRounds = Number(a);
  const wasmRounds = Number(b);
  const native = VARIANTS.flatMap((v) => [`native-${v}`, `native-${v}-phase`]);
  const wasm = ["nosimd", "simd"].flatMap((s) => VARIANTS.flatMap((v) => [`wasm-${s}-${v}`, `wasm-${s}-${v}-phase`]));
  const firstRound = existsSync(`${out}/samples.jsonl`)
    ? 1 + Math.max(...readFileSync(`${out}/samples.jsonl`, "utf8").trim().split("\n").map((l) => JSON.parse(l).round))
    : 0;
  for (let round = firstRound; round < firstRound + Math.max(nativeRounds, wasmRounds); round++) {
    const configs = [
      ...(round - firstRound < nativeRounds ? native.map((c) => ["native", c]) : []),
      ...(round - firstRound < wasmRounds ? wasm.map((c) => ["wasm", c]) : []),
    ];
    for (const [kind, config] of shuffled(configs)) {
      (kind === "native" ? measureNative : measureWasm)(config, round);
    }
    console.log(`round ${round} done`);
  }
} else if (command === "report") {
  report();
  sizes();
} else {
  console.error("usage: node scripts/solver-lab.mjs build | check | measure NATIVE_ROUNDS WASM_ROUNDS | report");
  process.exit(1);
}
