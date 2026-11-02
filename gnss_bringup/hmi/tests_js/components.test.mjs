import test from "node:test";
import assert from "node:assert/strict";
import { eventReplayWindow } from "../web/js/eventlist.js";

test("event replay window: closed event", () => {
  const w = eventReplayWindow({ t: 100, t_close: 160 });
  assert.deepEqual(w, { t0: 70, t1: 190 });
});

test("event replay window: open event uses t+60 fallback", () => {
  const w = eventReplayWindow({ t: 100, t_close: null });
  assert.deepEqual(w, { t0: 70, t1: 190 });
});
