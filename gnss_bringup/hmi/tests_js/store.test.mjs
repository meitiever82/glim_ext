import test from "node:test";
import assert from "node:assert/strict";
import { createStore } from "../web/js/store.js";

// minimal Vue shim: reactive() just returns the object (no proxying needed
// for these logic tests)
const Vue = { reactive: (o) => o };

test("live mode drops in-flight replay messages", () => {
  const s = createStore(Vue);
  s.replaying = false;
  // a replay position still in flight after 回到实时 must not land on the trail
  s.applyMessage({ type: "position", src: "can", lat: 44.5, lon: 90.3, q: 4, replay: true });
  assert.equal(s.trails.can.length, 0);
  // a genuine live message (no replay flag) is applied
  s.applyMessage({ type: "position", src: "can", lat: 44.5, lon: 90.3, q: 4 });
  assert.equal(s.trails.can.length, 1);
});

test("replay mode accepts replay-flagged messages", () => {
  const s = createStore(Vue);
  s.replaying = true;
  s.applyMessage({ type: "position", src: "rtkrcv", lat: 49.1, lon: 12.8, q: 1, replay: true });
  assert.equal(s.trails.rtkrcv.length, 1);
});

test("replay_end still processed while replaying (clears mode)", () => {
  const s = createStore(Vue);
  s.replaying = true;
  s.applyMessage({ type: "replay_end", t: 100, replay: true });
  assert.equal(s.replaying, false);   // not dropped by the live-guard
});

const status = (outage) => ({
  type: "status", t: 100, replay: false,
  link: { corr: { rate_bps: 120, gap_s: outage ? 5 : 0.2, last_t: 100, outage },
          obs: null, rtkrcv: null, topics: {} },
  verdict: { level: "ok", code: "-", message: "" },
});

test("link outage transition lands in the log once", () => {
  const s = createStore(Vue);
  s.applyMessage(status(false));
  assert.equal(s.logs.length, 0);
  s.applyMessage(status(true));
  assert.equal(s.logs.length, 1);
  assert.equal(s.logs[0].kind, "link");
  assert.equal(s.logs[0].text, "差分中断 5.0 s");
  s.applyMessage(status(true));       // unchanged outage must not repeat
  assert.equal(s.logs.length, 1);
  s.applyMessage(status(false));
  assert.equal(s.logs[0].text, "差分恢复");
  assert.equal(s.logs.length, 2);
});

test("events also land in the unified log", () => {
  const s = createStore(Vue);
  s.applyMessage({ type: "event", action: "open", replay: false,
                   event: { t: 50, code: "no_data", level: "warning", message: "无数据" } });
  assert.equal(s.logs.length, 1);
  assert.equal(s.logs[0].kind, "event");
  assert.equal(s.logs[0].text, "无数据");
});
