import assert from "node:assert/strict";
import test from "node:test";

import { physicsFailureMessage } from "./physics-error.mjs";

test("error 6 exposes the deterministic engine detail", () => {
  assert.equal(
    physicsFailureMessage(6, 611),
    "Physics stopped fail-closed with sandbox error 6 / detail 611 (repeated-event limit). Reset to start from the deterministic fixture again.",
  );
});

test("unknown error 6 detail remains actionable", () => {
  assert.equal(
    physicsFailureMessage(6, 777),
    "Physics stopped fail-closed with sandbox error 6 / detail 777 (unknown rotating-world failure). Reset to start from the deterministic fixture again.",
  );
});

test("legacy errors retain their stable public message", () => {
  assert.equal(
    physicsFailureMessage(3, 0),
    "Physics stopped fail-closed with sandbox error 3. Reset to start from the deterministic fixture again.",
  );
});

// The error number is a stable compatibility surface; the label must not imply exact math.
test("numeric range errors describe the production floating-point path", () => {
  assert.match(physicsFailureMessage(6, 612), /remaining-time numerical range/);
  assert.doesNotMatch(physicsFailureMessage(6, 612), /exact/);
});
