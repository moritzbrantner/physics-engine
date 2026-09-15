import assert from "node:assert/strict";
import { mkdtemp, readFile, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import test from "node:test";

import { buildPerformanceLogBundle } from "./package-performance-log.mjs";

test("bundle records payload hashes and accepts a browser session", async () => {
  const temporary = await mkdtemp(join(tmpdir(), "physics-performance-log-"));
  const evidence = join(temporary, "evidence");
  const session = join(temporary, "session.json");
  const archive = join(temporary, "bundle.tar.gz");
  await writeFile(session, JSON.stringify({ schema_version: 1, kind: "physics-engine-browser-session" }));
  const { manifest } = await buildPerformanceLogBundle({
    evidenceDirectory: evidence,
    archivePath: archive,
    sessionPaths: [session],
  });

  assert.equal(manifest.files.length, 1);
  assert.equal(manifest.files[0].kind, "interactive session");
  assert.match(await readFile(join(evidence, "SHA256SUMS"), "utf8"), /sessions\/session\.json/);
  assert.ok((await readFile(archive)).length > 0);
});

test("bundle rejects unrelated JSON passed as a browser session", async () => {
  const temporary = await mkdtemp(join(tmpdir(), "physics-performance-log-"));
  const session = join(temporary, "session.json");
  await writeFile(session, JSON.stringify({ schema_version: 1, kind: "other" }));

  await assert.rejects(
    buildPerformanceLogBundle({ evidenceDirectory: join(temporary, "evidence"), sessionPaths: [session] }),
    /not a physics-engine browser performance session/,
  );
});
