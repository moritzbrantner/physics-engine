import { readFile, writeFile } from "node:fs/promises";
import { basename, resolve } from "node:path";
import { pathToFileURL } from "node:url";

function displayUrl(url) {
  if (!url) return null;
  if (url.startsWith("wasm://")) return url;
  if (url.startsWith("file://")) return basename(new URL(url).pathname);
  return url;
}

export function summarizeCpuProfile(profile) {
  if (!Array.isArray(profile.nodes) || !Array.isArray(profile.samples)) {
    throw new Error("CPU profile must contain nodes and samples");
  }
  if (!Array.isArray(profile.timeDeltas) || profile.timeDeltas.length !== profile.samples.length) {
    throw new Error("CPU profile samples and timeDeltas must have equal length");
  }

  const nodes = new Map(profile.nodes.map((node) => [node.id, node]));
  const totals = new Map();
  let totalMicroseconds = 0;
  for (let index = 0; index < profile.samples.length; index += 1) {
    const node = nodes.get(profile.samples[index]);
    if (!node) throw new Error(`CPU profile references missing node ${profile.samples[index]}`);
    const microseconds = profile.timeDeltas[index];
    if (!Number.isFinite(microseconds) || microseconds < 0) {
      throw new Error("CPU profile contains an invalid time delta");
    }
    totalMicroseconds += microseconds;
    const frame = node.callFrame ?? {};
    const key = JSON.stringify([frame.functionName ?? "(anonymous)", frame.url ?? "", frame.lineNumber ?? -1]);
    const existing = totals.get(key) ?? {
      function: frame.functionName || "(anonymous)",
      url: displayUrl(frame.url),
      line: Number.isInteger(frame.lineNumber) ? frame.lineNumber + 1 : null,
      self_microseconds: 0,
      samples: 0,
    };
    existing.self_microseconds += microseconds;
    existing.samples += 1;
    totals.set(key, existing);
  }

  const hotFunctions = [...totals.values()]
    .sort((left, right) =>
      right.self_microseconds - left.self_microseconds || left.function.localeCompare(right.function),
    )
    .slice(0, 50)
    .map((entry) => ({
      ...entry,
      self_ms: entry.self_microseconds / 1_000,
      self_percent: totalMicroseconds === 0 ? 0 : (entry.self_microseconds * 100) / totalMicroseconds,
    }));

  return {
    schema_version: 1,
    kind: "physics-engine-cpu-profile-summary",
    note: "V8 statistical CPU profile of one fixed head-only WASM workload. Sampling and profiling overhead make timings advisory.",
    total_sampled_ms: totalMicroseconds / 1_000,
    sample_count: profile.samples.length,
    hot_functions: hotFunctions,
  };
}

if (process.argv[1] && pathToFileURL(resolve(process.argv[1])).href === import.meta.url) {
  const [profilePath, outputPath] = process.argv.slice(2);
  if (!profilePath || !outputPath) {
    throw new Error("usage: node scripts/summarize-cpu-profile.mjs <profile.cpuprofile> <summary.json>");
  }
  const summary = summarizeCpuProfile(JSON.parse(await readFile(profilePath, "utf8")));
  await writeFile(outputPath, `${JSON.stringify(summary, null, 2)}\n`);
}
