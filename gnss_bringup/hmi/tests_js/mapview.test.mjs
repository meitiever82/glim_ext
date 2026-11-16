import test from "node:test";
import assert from "node:assert/strict";
import { trailColor } from "../web/js/mapview.js";

test("trail colors map fix classes", () => {
  assert.equal(trailColor("fixed"), "#3fb96c");
  assert.equal(trailColor("float"), "#e0b23c");
  assert.equal(trailColor("bad"), "#e05c4f");
  assert.equal(trailColor("none"), "#5a6472");
});
