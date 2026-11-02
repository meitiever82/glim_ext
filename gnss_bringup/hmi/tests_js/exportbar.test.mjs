import test from "node:test";
import assert from "node:assert/strict";
import { exportRange, reportUrl, posUrl } from "../web/js/exportbar.js";

test("exportRange converts datetime-local strings to epoch seconds", () => {
  const a = new Date("2026-11-03T10:00").getTime(), b = new Date("2026-11-03T11:00").getTime();
  assert.deepEqual(exportRange("2026-11-03T10:00", "2026-11-03T11:00"), [a / 1000, b / 1000]);
});

test("exportRange rejects empty, reversed and unparsable windows", () => {
  assert.equal(exportRange("", ""), null);
  assert.equal(exportRange("2026-11-03T11:00", "2026-11-03T10:00"), null);
  assert.equal(exportRange("2026-11-03T10:00", "2026-11-03T10:00"), null);
  assert.equal(exportRange("nope", "2026-11-03T10:00"), null);
});

test("reportUrl carries the window", () => {
  const a = new Date("2026-11-03T10:00").getTime() / 1000;
  const b = new Date("2026-11-03T11:00").getTime() / 1000;
  assert.equal(reportUrl("2026-11-03T10:00", "2026-11-03T11:00"), `/report?t0=${a}&t1=${b}`);
  assert.equal(reportUrl("2026-11-03T11:00", "2026-11-03T10:00"), null);
});

test("posUrl selects source and encodes it", () => {
  const a = new Date("2026-11-03T10:00").getTime() / 1000;
  const b = new Date("2026-11-03T11:00").getTime() / 1000;
  assert.equal(posUrl("2026-11-03T10:00", "2026-11-03T11:00", "app"),
               `/api/pos?src=app&t0=${a}&t1=${b}`);
  assert.equal(posUrl("2026-11-03T10:00", "2026-11-03T11:00", "fused"),
               `/api/pos?src=fused&t0=${a}&t1=${b}`);
  assert.equal(posUrl("2026-11-03T10:00", "2026-11-03T11:00", "a b"),
               `/api/pos?src=a%20b&t0=${a}&t1=${b}`);
});
