import assert from "node:assert/strict";
import test from "node:test";

import { SCENARIOS, scenarioForKey } from "./scenario-config.mjs";

test("scenario catalog has stable numeric ids and progressive sophistication", () => {
  assert.deepEqual(
    Object.values(SCENARIOS).map(({ id }) => id).sort((a, b) => a - b),
    [0, 1, 2, 3, 4, 5, 6],
  );
  assert.equal(scenarioForKey("ccd-gauntlet").level, 1);
  assert.equal(scenarioForKey("rotating-box-lab").level, 2);
  assert.equal(scenarioForKey("sleeping-world").level, 3);
  assert.equal(scenarioForKey("missing"), null);
});
