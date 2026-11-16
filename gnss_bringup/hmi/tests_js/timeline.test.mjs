import test from "node:test";
import assert from "node:assert/strict";
import { sigmaSeriesCm } from "../web/js/timeline.js";
import { skyXY } from "../web/js/skyplot.js";

test("sigma meters to cm, nulls preserved", () => {
  assert.deepEqual(sigmaSeriesCm([0.011, null, 0.02]), [1.1, null, 2]);
});

test("sky projection: el=90 center, el=0 rim, az=90 east", () => {
  const c = skyXY(0, 90, 100);      // az irrelevant at zenith
  assert.ok(Math.abs(c.x - 100) < 1e-9 && Math.abs(c.y - 100) < 1e-9);
  const e = skyXY(90, 0, 100);      // due east on the rim
  assert.ok(Math.abs(e.x - 200) < 1e-9 && Math.abs(e.y - 100) < 1e-9);
  const n = skyXY(0, 0, 100);       // due north on the rim (up)
  assert.ok(Math.abs(n.x - 100) < 1e-9 && Math.abs(n.y - 0) < 1e-9);
});
