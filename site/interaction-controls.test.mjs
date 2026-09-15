import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import { runInNewContext } from "node:vm";

test("comparison controls keep native keyboard input away from game handlers", () => {
  const listeners = [];
  const controls = [0, 1, 2].map(() => ({
    addEventListener(type, handler) {
      assert.equal(type, "keydown");
      listeners.push(handler);
    },
  }));
  runInNewContext(readFileSync(new URL("./interaction-controls.mjs", import.meta.url), "utf8"), {
    document: {
      querySelectorAll(selector) {
        assert.equal(selector, "#character-mode, #upright-crates, #fixed-geometry-mode");
        return controls;
      },
    },
  });
  assert.equal(listeners.length, 3);
  for (const handler of listeners) {
    let stopped = false;
    handler({
      stopPropagation() {
        stopped = true;
      },
      preventDefault() {
        assert.fail("native control behavior must remain available");
      },
    });
    assert.ok(stopped);
  }
});
