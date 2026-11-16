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
  s.applyMessage({ type: "position", src: "app", lat: 44.5, lon: 90.3, q: 4, replay: true });
  assert.equal(s.trails.app.length, 0);
  // a genuine live message (no replay flag) is applied
  s.applyMessage({ type: "position", src: "app", lat: 44.5, lon: 90.3, q: 4 });
  assert.equal(s.trails.app.length, 1);
});

test("replay mode accepts replay-flagged messages", () => {
  const s = createStore(Vue);
  s.replaying = true;
  s.applyMessage({ type: "position", src: "fused", lat: 49.1, lon: 12.8, q: 3, replay: true });
  assert.equal(s.trails.fused.length, 1);
});

test("replay_end still processed while replaying (clears mode)", () => {
  const s = createStore(Vue);
  s.replaying = true;
  s.applyMessage({ type: "replay_end", t: 100, replay: true });
  assert.equal(s.replaying, false);   // not dropped by the live-guard
});

test("status keeps dual trails independent and appends series", () => {
  const s = createStore(Vue);
  s.applyMessage({
    type: "status", t: 1000,
    sources: {
      app: { available: true, quality: 4, lat: 44.5, lon: 90.3, sats: 22, age_s: 1.2,
             sigma_h: 0.016, ratio: 9.5, latency_s: 0.04 },
      fused: { available: false, quality: 4, lat: 44.6, lon: 90.4, reason: "数据中断" },
    },
    corr_rate: { rtcm_bps: 1200 },
  });
  assert.equal(s.trails.app.length, 1);
  // 无数据的一路不追加轨迹（保留地图上的最后位置）
  assert.equal(s.trails.fused.length, 0);
  assert.deepEqual(s.series.sats, [22]);
  assert.deepEqual(s.series.sigma, [0.016]);
  assert.deepEqual(s.series.latency, [0.04]);
  assert.deepEqual(s.series.corr, [1200]);
});

test("duplicate position points are dropped per trail", () => {
  const s = createStore(Vue);
  const m = { type: "position", src: "app", lat: 44.5, lon: 90.3, q: 4 };
  s.applyMessage(m);
  s.applyMessage(m);
  assert.equal(s.trails.app.length, 1);
});
