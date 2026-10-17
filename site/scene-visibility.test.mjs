import assert from "node:assert/strict";
import test from "node:test";
import { bodyMayBeVisible, cameraSpace, createCamera } from "./scene-visibility.mjs";

const camera = createCamera([0, 10, 0], 0, 0, 16 / 9, Math.PI / 3, 0.8, 500);
const body = (position, half = [10, 10, 10]) => ({ position, half });

test("camera transform faces negative Z with cached trigonometry", () => {
  assert.deepEqual(cameraSpace([0, 10, -40], camera), [0, 0, 40]);
  assert.deepEqual(cameraSpace([10, 10, 0], camera), [10, 0, 0]);
});

test("conservative body culling keeps edge intersections and nearby bodies", () => {
  assert.equal(bodyMayBeVisible(body([0, 10, -40]), camera), true);
  assert.equal(bodyMayBeVisible(body([0, 10, 8]), camera), true);
  assert.equal(bodyMayBeVisible(body([45, 10, -40]), camera), true);
  assert.equal(bodyMayBeVisible(body([200, 10, -40]), camera), false);
  assert.equal(bodyMayBeVisible(body([0, 10, 100]), camera), false);
  assert.equal(bodyMayBeVisible(body([0, 10, -700]), camera), false);
});

test("rotated view retains a body straight ahead", () => {
  const turned = createCamera([0, 0, 0], Math.PI / 2, 0, 1, Math.PI / 2, 0.8, 500);
  assert.equal(bodyMayBeVisible(body([100, 0, 0]), turned), true);
  assert.equal(bodyMayBeVisible(body([0, 0, -100]), turned), false);
});
