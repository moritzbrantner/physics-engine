import { createHash } from "node:crypto";
import { execFileSync } from "node:child_process";
import { cp, mkdir, readFile, readdir, stat, writeFile } from "node:fs/promises";
import { basename, join, relative, resolve } from "node:path";
import { pathToFileURL } from "node:url";

const GENERATED_FILES = new Set(["README.md", "manifest.json", "provenance.json", "SHA256SUMS"]);

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
  if (parsed?.kind === "physics-engine-browser-session") return "interactive session";
  if (parsed?.kind === "physics-engine-cpu-profile-summary") return "CPU profile summary";
  if (parsed?.workload) return `deterministic workload: ${parsed.workload}`;
  if (path.endsWith(".wasm")) return "measured WASM artifact";
  if (path.endsWith(".cpuprofile")) return "CPU profile";
  if (path.endsWith(".log")) return "raw command log";
  return "supporting evidence";
}

export async function buildPerformanceLogBundle({ evidenceDirectory, archivePath, sessionPaths = [] }) {
  const root = resolve(evidenceDirectory);
  await mkdir(root, { recursive: true });
  if (sessionPaths.length > 0) await mkdir(join(root, "sessions"), { recursive: true });

  for (const source of sessionPaths) {
    const parsed = await readJsonIfPossible(source);
    if (parsed?.schema_version !== 1 || parsed?.kind !== "physics-engine-browser-session") {
      throw new Error(`${source} is not a physics-engine browser performance session`);
    }
    await cp(source, join(root, "sessions", basename(source)));
  }

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
    schema_version: 1,
    kind: "physics-engine-performance-log-provenance",
    generated_at: new Date().toISOString(),
    head_revision: process.env.HEAD_SHA ?? null,
    base_revision: process.env.BASE_SHA ?? null,
    workflow_run_id: process.env.GITHUB_RUN_ID ?? null,
    workflow_run_attempt: process.env.GITHUB_RUN_ATTEMPT ?? null,
    limitations: [
      "Wall-clock timings are advisory and must not be used as brittle CI pass/fail thresholds.",
      "Node/V8 WASM timings isolate physics calls; they are not browser FPS or GPU measurements.",
      "Interactive sessions include browser scheduling, rendering, device, and user-input variability.",
      "Compare unchanged workload versions and inspect replay hashes before attributing a timing difference to code.",
    ],
  };
  await writeFile(join(root, "provenance.json"), `${JSON.stringify(provenance, null, 2)}\n`);

  const manifest = {
    schema_version: 1,
    kind: "physics-engine-performance-log-manifest",
    files: entries,
    detected_failures: failures,
  };
  await writeFile(join(root, "manifest.json"), `${JSON.stringify(manifest, null, 2)}\n`);
  const rows = entries.map(
    (entry) => `| \`${entry.path}\` | ${entry.kind} | ${entry.bytes} | \`${entry.sha256.slice(0, 12)}…\` |`,
  );
  const summary = `# Physics-engine performance log\n\n` +
    `This bundle is designed to be attached to a follow-up conversation for performance analysis. ` +
    `Start with \`manifest.json\` and \`provenance.json\`, then inspect raw arrays and profiles before drawing conclusions.\n\n` +
    `## Revisions\n\n- Head: \`${provenance.head_revision ?? "not recorded"}\`\n` +
    `- Base: \`${provenance.base_revision ?? "not recorded"}\`\n` +
    `- Detected workload failures: ${failures.length}\n\n` +
    `## Evidence\n\n| File | Purpose | Bytes | SHA-256 |\n| --- | --- | ---: | --- |\n` +
    `${rows.join("\n")}\n\n` +
    `## Analysis guidance\n\n` +
    `Treat deterministic workload inputs and replay hashes as correctness boundaries. Compare timing distributions ` +
    `only across equivalent workload versions and similar environments. Browser sessions are useful for finding ` +
    `frame spikes and correlations, but they do not replace the deterministic benchmark evidence.\n`;
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
