#!/usr/bin/env node
import assert from "node:assert/strict";
import { spawnSync } from "node:child_process";
import { randomUUID } from "node:crypto";
import { existsSync, mkdirSync, readFileSync, readdirSync, writeFileSync } from "node:fs";
import { arch, cpus, platform, release } from "node:os";
import { dirname, join, resolve } from "node:path";
import { fileURLToPath } from "node:url";
import { assertHistoryPrefix, compareRows, hash, parseNative, sandboxRows, validateHistory } from "./performance-ratchet-lib.mjs";

const root = resolve(dirname(fileURLToPath(import.meta.url)), "..");
const historyRoot = join(root, ".performance/ratchet/history");
const fixturePath = join(root, ".performance/ratchet-fixtures.json");
const fixtureBytes = readFileSync(fixturePath);
const fixture = JSON.parse(fixtureBytes);
const fixtureSha256 = hash(fixtureBytes);
const [command, ...args] = process.argv.slice(2);
assert(["check", "record", "history"].includes(command),
  "usage: node scripts/performance-ratchet.mjs check [--base SHA] | record --id NAME --reason TEXT [--new-epoch] | history");
const options = {};
while (args.length) {
  const key = args.shift();
  assert(!Object.hasOwn(options, key), `duplicate option ${key}`);
  assert(["--base", "--id", "--reason", "--new-epoch"].includes(key), `unknown option ${key}`);
  if (key === "--new-epoch") options[key] = true;
  else {
    const value = args.shift();
    assert(value && !value.startsWith("--"), `missing value for ${key}`);
    options[key] = value;
  }
}
if (command !== "record") {
  assert(!options["--id"] && !options["--reason"] && !options["--new-epoch"], "read-only commands cannot update baselines");
} else {
  assert(/^[a-z0-9]+(?:-[a-z0-9]+)*$/.test(options["--id"] ?? ""), "record requires --id with lowercase words/dashes");
  assert((options["--reason"] ?? "").trim().length >= 20, "record requires an explanatory --reason (at least 20 characters)");
}

function git(...args) {
  const result = spawnSync("git", args, { cwd: root, encoding: "utf8", maxBuffer: 32 * 1024 * 1024 });
  assert.equal(result.status, 0, result.stderr || String(result.error));
  return result.stdout;
}

function history() {
  if (!existsSync(historyRoot)) return [];
  return readdirSync(historyRoot).sort().map((name) => {
    assert(/^\d{4}-[a-z0-9-]+\.json$/.test(name), `unexpected history file ${name}`);
    return { name, contents: readFileSync(join(historyRoot, name), "utf8") };
  });
}

function sourceSnapshot() {
  const paths = git("ls-files", "-z", "--cached", "--others", "--exclude-standard").split("\0").filter(Boolean);
  const files = [...new Set(paths)].filter((path) =>
    /^(src\/|tests\/|scripts\/|demo-wasm\/|Cargo\.|rust-toolchain|\.coding-tooling\.json$|\.performance\/[^/]+\.json$)/.test(path)
  ).sort().map((path) => ({ path, sha256: existsSync(join(root, path)) ? hash(readFileSync(join(root, path))) : null }));
  return { files, snapshotSha256: hash(JSON.stringify(files)) };
}

const initialHistory = history();
const baseline = validateHistory(initialHistory);
if (command === "history") {
  let previous = null;
  for (const file of initialHistory) {
    const entry = JSON.parse(file.contents);
    console.log(`${file.name} | epoch ${entry.epoch} | ${entry.recordedAt}\n  ${entry.reason}\n  source ${entry.source.revision}${entry.source.dirty ? " (dirty snapshot)" : ""} ${entry.source.snapshotSha256}`);
    if (previous?.epoch === entry.epoch) {
      for (const change of compareRows(previous.rows, entry.rows)) {
        console.log(`  ${change.scenario} ${change.metric}: ${change.before} -> ${change.after}`);
      }
    } else console.log("  Establishes a new correctness/workload baseline.");
    previous = entry;
  }
  process.exit(0);
}
if (command === "record") assert(!initialHistory.some((file) => JSON.parse(file.contents).id === options["--id"]), "history id already recorded; choose a new milestone id");
if (command === "check") assert(baseline, "no ratchet baseline; use the explicit record command to initialize it");
if (baseline && !options["--new-epoch"]) assert.equal(fixtureSha256, baseline.fixtureSha256, "fixture changed; record an explicit new epoch");
assert.equal(fixture.schemaVersion, 1, "unsupported fixture schema");
assert.equal(fixture.sandbox.trials, 2, "this workload requires two replay trials");
const base = options["--base"] ?? process.env.RATCHET_BASE_SHA;
if (base) {
  assert(/^[a-f0-9]{40}$/.test(base), "base must be a full commit SHA");
  const prefix = ".performance/ratchet/history/";
  const paths = git("ls-tree", "-r", "--name-only", base, "--", prefix).trim().split("\n").filter(Boolean).sort();
  const files = paths.map((path) => ({ name: path.slice(prefix.length), contents: git("show", `${base}:${path}`) }));
  validateHistory(files);
  assertHistoryPrefix(files, initialHistory);
}

const recordedAt = new Date().toISOString();
const output = join(root, "performance-evidence/ratchet", `${recordedAt.replaceAll(":", "-")}-${randomUUID().slice(0, 8)}`);
mkdirSync(output, { recursive: true });
const initialSource = sourceSnapshot();
const source = {
  revision: git("rev-parse", "HEAD").trim(),
  dirty: git("status", "--porcelain", "--untracked-files=normal").length > 0,
  snapshotSha256: initialSource.snapshotSha256,
  conventionsSourceRevision: process.env.CONVENTIONS_SOURCE_REVISION ?? null,
};
writeFileSync(join(output, "source-files.json"), `${JSON.stringify(initialSource.files, null, 2)}\n`);
for (const file of initialSource.files) {
  if (file.sha256 === null) continue;
  const destination = join(output, "source", file.path);
  mkdirSync(dirname(destination), { recursive: true });
  writeFileSync(destination, readFileSync(join(root, file.path)));
}
writeFileSync(join(output, "fixtures.json"), fixtureBytes);
writeFileSync(join(output, "source.patch"), git("diff", "HEAD", "--", ".", ":(exclude).performance/ratchet/history"));

function run(label, executable, argv, overrides = {}) {
  console.log(`ratchet: ${label}`);
  const env = { ...process.env, LC_ALL: "C", TZ: "UTC", ...overrides };
  for (const key of ["CASE", "BASELINE_WASM", "CARGO_BUILD_TARGET"]) delete env[key];
  const result = spawnSync(executable, argv, { cwd: root, env, encoding: "utf8", maxBuffer: 64 * 1024 * 1024 });
  writeFileSync(join(output, `${label}.log`), `${result.stdout ?? ""}\n${result.stderr ?? ""}`);
  assert.equal(result.status, 0, `${label} failed: ${result.error ?? result.stderr ?? ""}\nSee ${output}`);
  return result.stdout;
}

const report = { schemaVersion: 1, recordedAt, source, fixtureSha256, status: "failed" };
try {
  report.environment = {
    platform: platform(), arch: arch(), osRelease: release(), cpu: cpus()[0]?.model ?? "unknown",
    node: process.version, v8: process.versions.v8,
    rustc: run("rustc", "rustc", ["-Vv"]).trim(), cargo: run("cargo", "cargo", ["--version"]).trim(),
    profile: "release", nativeTestThreads: 1, rustflags: process.env.RUSTFLAGS ?? "",
  };
  const rows = {};
  for (const [index, native] of fixture.native.entries()) {
    const stdout = run(`native-${index + 1}`, "cargo", ["test", "--release", "--locked", "--lib", native.test,
      "--", "--exact", "--include-ignored", "--nocapture", "--test-threads=1"]);
    const collected = parseNative(stdout, native.scenarios);
    for (const id of Object.keys(collected)) assert(!Object.hasOwn(rows, id), `duplicate fixture ${id}`);
    Object.assign(rows, collected);
  }
  run("wasm-build", "cargo", ["build", "--manifest-path", "demo-wasm/Cargo.toml", "--locked", "--release", "--target", "wasm32-unknown-unknown"]);
  const wasm = join(root, "demo-wasm/target/wasm32-unknown-unknown/release/physics_engine_demo.wasm");
  report.wasmSha256 = hash(readFileSync(wasm));
  run("sandbox", process.execPath, [join(root, "scripts/benchmark-sandbox.mjs"), wasm, join(output, "sandbox.json")],
    { TRIALS: String(fixture.sandbox.trials), HEAD_SHA: source.revision });
  Object.assign(rows, sandboxRows(JSON.parse(readFileSync(join(output, "sandbox.json"))), fixture.sandbox));
  report.rows = Object.fromEntries(Object.entries(rows).sort(([a], [b]) => a.localeCompare(b)));
  assert.equal(sourceSnapshot().snapshotSha256, source.snapshotSha256, "source changed during measurement; discard this run");
  assert.deepEqual(history(), initialHistory, "history changed during measurement; retry against the new baseline");
  if (baseline && !options["--new-epoch"]) report.improvements = compareRows(baseline.rows, report.rows);
  if (command === "record") {
    const entry = {
      schemaVersion: 1, ordinal: initialHistory.length + 1, id: options["--id"],
      epoch: baseline ? baseline.epoch + (options["--new-epoch"] ? 1 : 0) : 1,
      reason: options["--reason"].trim(), recordedAt,
      previousSha256: initialHistory.length ? hash(initialHistory.at(-1).contents) : null,
      fixtureSha256, source, environment: report.environment, wasmSha256: report.wasmSha256, rows: report.rows,
    };
    const name = `${String(entry.ordinal).padStart(4, "0")}-${entry.id}.json`;
    const contents = `${JSON.stringify(entry, null, 2)}\n`;
    validateHistory([...initialHistory, { name, contents }]);
    mkdirSync(historyRoot, { recursive: true });
    writeFileSync(join(historyRoot, name), contents, { flag: "wx" });
    report.recordedEntry = name;
  }
  report.status = "passed";
  console.log(`ratchet passed: ${Object.keys(rows).length} scenarios; ${report.improvements?.length ?? 0} lower work counts`);
} catch (error) {
  report.error = String(error);
  process.exitCode = 1;
  console.error(report.error);
} finally {
  writeFileSync(join(output, "result.json"), `${JSON.stringify(report, null, 2)}\n`);
  console.log(`ratchet evidence: ${output}`);
}
