// Read capability names from their canonical documentation; the Pages UI owns no second list.
import { readFile, writeFile } from "node:fs/promises";

const source = await readFile(new URL("../docs/consumer-capabilities.md", import.meta.url), "utf8");
const capabilities = source.split(/\r?\n/).flatMap((line) => {
  const match = /^\|\s*(C\d+)\s*\|\s*([^|]+?)\s*\|/.exec(line);
  if (!match) return [];
  const detail = match[2].replace(/\*\*/g, "").replace(/`/g, "").trim();
  const clause = detail.split(";")[0];
  const parenthesis = clause.indexOf(" (");
  const title = parenthesis !== -1 && clause.length > 50 ? clause.slice(0, parenthesis) : clause;
  return [{
    id: match[1],
    title,
    detail,
  }];
});

if (capabilities.length !== 19 || capabilities.some((item, i) => item.id !== `C${i + 1}`)) {
  throw new Error("docs/consumer-capabilities.md must define C1–C19 once, in order");
}

const destination = process.argv[2];
if (!destination) throw new Error("usage: node scripts/generate-pages-capabilities.mjs <output.json>");
await writeFile(destination, `${JSON.stringify(capabilities, null, 2)}\n`);
