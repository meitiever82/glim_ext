import test from "node:test";
import assert from "node:assert/strict";
import { eventReplayWindow } from "../web/js/eventlist.js";
import { linkRows } from "../web/js/linkcard.js";

test("event replay window: closed event", () => {
  const w = eventReplayWindow({ t: 100, t_close: 160 });
  assert.deepEqual(w, { t0: 70, t1: 190 });
});

test("event replay window: open event uses t+60 fallback", () => {
  const w = eventReplayWindow({ t: 100, t_close: null });
  assert.deepEqual(w, { t0: 70, t1: 190 });
});

test("link card flags interrupted corrections and rtkrcv health", () => {
  const rows = linkRows({
    corr_rate: { rtcm_bps: 0, obs_bps: 800, rtcm_stale: true, obs_stale: false },
    diagnostics: { rtkrcv: { level_name: "ERROR", message: "rtkrcv 崩溃" } },
    verdict: { level: "error", message: "差分中断" },
    links: [{ topic: "/gnss/diagnostics", state: "no_data" }],
  });
  const byName = Object.fromEntries(rows.map((r) => [r.name, r]));
  assert.equal(byName["差分"].cls, "bad");
  assert.equal(byName["差分"].note, "中断");
  assert.equal(byName["观测"].cls, "ok");
  assert.equal(byName["rtkrcv"].cls, "bad");
  assert.equal(byName["诊断"].text, "error");
  assert.equal(byName["/gnss/diagnostics"].cls, "bad");
});

test("link card stays quiet when everything is healthy", () => {
  const rows = linkRows({
    corr_rate: { rtcm_bps: 1200, obs_bps: 900, rtcm_stale: false, obs_stale: false },
    diagnostics: { rtkrcv: { level_name: "OK", message: "" } },
    verdict: { level: "ok", message: "" },
    links: [{ topic: "/gnss/diagnostics", state: "ok" }],
  });
  assert.equal(rows.length, 4);
  assert.ok(rows.every((r) => r.cls === "ok"));
});
