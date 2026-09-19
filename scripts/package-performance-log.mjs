import { createHash } from "node:crypto";
import { execFileSync } from "node:child_process";
import { cp, mkdir, readFile, readdir, stat, writeFile } from "node:fs/promises";
import { basename, dirname, join, relative, resolve } from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

import { buildCanonicalPerformanceEvidence } from "./adapt-performance-evidence.mjs";

const GENERATED_FILES = new Set(["README.md", "manifest.json", "provenance.json", "SHA256SUMS"]);
const CONTRACT = {
  repository: "https://github.com/moritzbrantner/performance-evidence",
  revision: "068d880a1b76f41e06548c557af0a7bf8c8057eb",
  schema_version: "1.0.0",
};
const SCRIPT_ROOT = dirname(fileURLToPath(import.meta.url));
const SANDBOX_PROFILE = join(SCRIPT_ROOT, "..", ".performance", "sandbox-evidence-profile.json");

async function filesBelow(root, directory = root) {
  const entries = await readdir(directory, { withFileTypes: true });
  const files = [];
  for (const entry of entries.sort((left, right) => left.name.localeCompare(right.name))) {
    const path = join(directory, entry.name);
    if (entry.isDirectory()) files.push(...(await filesBelow(root, path)));
    else if (entry.isFile()) files.push(relative(root, path).replaceAll("\\", "/"));
  }
  return files;
}

async function sha256(path) {
  return createHash("sha256").update(await readFile(path)).digest("hex");
}

async function readJsonIfPossible(path) {
  try {
    return JSON.parse(await readFile(path, "utf8"));
  } catch {
    return null;
  }
}

function evidenceKind(path, parsed) {
  if (parsed?.schema_version === "1.0.0" && parsed?.measurements) return "canonical performance evidence";
  if (parsed?.kind === "physics-engine-browser-session") return "interactive session";
  if (parsed?.kind === "physics-engine-cpu-profile-summary") return "CPU profile summary";
  if (path.startsWith("profiles/") && path.endsWith(".json")) return "measurement profile";
  if (parsed?.workload) return `deterministic workload: ${parsed.workload}`;
  if (path.endsWith(".wasm")) return "measured WASM artifact";
  if (path.endsWith(".cpuprofile")) return "CPU profile";
  if (path.endsWith(".log")) return "raw command log";
  return "supporting evidence";
}

function detectSourceDirty() {
  if (process.env.SOURCE_DIRTY === "true") return true;
  if (process.env.SOURCE_DIRTY === "false") return false;
  try {
    return execFileSync("git", ["status", "--porcelain"], { encoding: "utf8" }).trim().length > 0;
  } catch {
    return true;
  }
}

export async function buildPerformanceLogBundle({ evidenceDirectory, archivePath, sessionPaths = [] }) {
  const root = resolve(evidenceDirectory);
  await mkdir(root, { recursive: true });
  if (sessionPaths.length > 0) await mkdir(join(root, "sessions"), { recursive: true });

  for (const source of sessionPaths) {
    const parsed = await readJsonIfPossible(source);
    if (parsed?.schema_version !== 2 || parsed?.kind !== "physics-engine-browser-session") {
      throw new Error(`${source} is not a current physics-engine browser performance session`);
    }
    await cp(source, join(root, "sessions", basename(source)));
  }

  await mkdir(join(root, "profiles"), { recursive: true });
  await cp(SANDBOX_PROFILE, join(root, "profiles", "physics-engine-sandbox-v1.json"));
  const canonicalPaths = await buildCanonicalPerformanceEvidence({
    evidenceDirectory: root,
    sourceDirty: detectSourceDirty(),
  });

  for (const generated of GENERATED_FILES) {
    try {
      const path = join(root, generated);
      if ((await stat(path)).isFile()) await writeFile(path, "");
    } catch {
      // The first packaging run has no generated metadata yet.
    }
  }

  const payloadFiles = (await filesBelow(root)).filter(
    (path) => !GENERATED_FILES.has(path) && !path.endsWith(".tar.gz"),
  );
  const entries = [];
  const failures = [];
  for (const path of payloadFiles) {
    const fullPath = join(root, path);
    const parsed = path.endsWith(".json") ? await readJsonIfPossible(fullPath) : null;
    if (parsed?.failure) failures.push({ path, failure: parsed.failure });
    entries.push({
      path,
      bytes: (await stat(fullPath)).size,
      sha256: await sha256(fullPath),
      kind: evidenceKind(path, parsed),
    });
  }

  const provenance = {
    schema_version: 2,
    kind: "physics-engine-performance-log-provenance",
    generated_at: new Date().toISOString(),
    head_revision: process.env.HEAD_SHA ?? null,
    base_revision: process.env.BASE_SHA ?? null,
    workflow_run_id: process.env.GITHUB_RUN_ID ?? null,
    workflow_run_attempt: process.env.GITHUB_RUN_ATTEMPT ?? null,
    performance_evidence_contract: CONTRACT,
    limitations: [
      "Wall-clock timings are advisory and must not be used as brittle CI pass/fail thresholds.",
      "Node/V8 WASM timings isolate physics calls; they are not browser FPS or GPU measurements.",
      "Interactive sessions include browser scheduling, rendering, device, and user-input variability.",
      "Compare identical workload hashes and inspect replay fingerprints before attributing a timing difference to code.",
    ],
  };
  await writeFile(join(root, "provenance.json"), `${JSON.stringify(provenance, null, 2)}\n`);

  const manifest = {
    schema_version: 2,
    kind: "physics-engine-performance-log-manifest",
    performance_evidence_contract: CONTRACT,
    measurement_profiles: ["profiles/physics-engine-sandbox-v1.json"],
    canonical_evidence: canonicalPaths,
    files: entries,
    detected_failures: failures,
  };
  await writeFile(join(root, "manifest.json"), `${JSON.stringify(manifest, null, 2)}\n`);
  const rows = entries.map(
    (entry) => `| \`${entry.path}\` | ${entry.kind} | ${entry.bytes} | \`${entry.sha256.slice(0, 12)}…\` |`,
  );
  const summary = `# Physics-engine performance evidence bundle\n\n` +
    `The primary records are the canonical Performance Evidence documents under \`canonical/\`. ` +
    `They classify useful work, induced work, and outcomes while preserving exact workload, source, environment, artifact, and baseline provenance. ` +
    `Raw benchmark logs, browser sessions, WASM binaries, and CPU profiles remain attached as hashed supporting evidence.\n\n` +
    `## Contract\n\n- Performance Evidence: \`${CONTRACT.revision}\`\n` +
    `- Schema: \`${CONTRACT.schema_version}\`\n` +
    `- Canonical documents: ${canonicalPaths.length}\n` +
    `- Measurement profile: \`profiles/physics-engine-sandbox-v1.json\`\n\n` +
    `## Revisions\n\n- Head: \`${provenance.head_revision ?? "not recorded"}\`\n` +
    `- Base: \`${provenance.base_revision ?? "not recorded"}\`\n` +
    `- Detected workload failures: ${failures.length}\n\n` +
    `## Evidence\n\n| File | Purpose | Bytes | SHA-256 |\n| --- | --- | ---: | --- |\n` +
    `${rows.join("\n")}\n\n` +
    `## Analysis guidance\n\n` +
    `Start with canonical evidence and compare identical workload hashes. Use induced-work counters to explain changes before ` +
    `treating wall-clock movement as causal. Replay fingerprints remain correctness evidence. Inspect the referenced raw artifacts ` +
    `when a counter or timing distribution needs deeper diagnosis; browser sessions are diagnostic and do not replace deterministic workloads.\n`;
  await writeFile(join(root, "README.md"), summary);

  const hashableFiles = (await filesBelow(root)).filter(
    (path) => path !== "SHA256SUMS" && !path.endsWith(".tar.gz"),
  );
  const checksums = [];
  for (const path of hashableFiles) checksums.push(`${await sha256(join(root, path))}  ${path}`);
  await writeFile(join(root, "SHA256SUMS"), `${checksums.join("\n")}\n`);

  const target = resolve(archivePath ?? `${root}.tar.gz`);
  execFileSync("tar", [
    "--sort=name",
    "--mtime=@0",
    "--owner=0",
    "--group=0",
    "--numeric-owner",
    "-czf",
    target,
    "-C",
    root,
    ".",
  ]);
  return { archivePath: target, manifest, provenance };
}

if (process.argv[1] && pathToFileURL(resolve(process.argv[1])).href === import.meta.url) {
  const [evidenceDirectory, archivePath, ...sessionPaths] = process.argv.slice(2);
  if (!evidenceDirectory) {
    throw new Error(
      "usage: node scripts/package-performance-log.mjs <evidence-directory> [archive.tar.gz] [browser-session.json ...]",
    );
  }
  const result = await buildPerformanceLogBundle({ evidenceDirectory, archivePath, sessionPaths });
  console.log(result.archivePath);
}
