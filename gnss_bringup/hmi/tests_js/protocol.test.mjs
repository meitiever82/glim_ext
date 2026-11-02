import test from "node:test";
import assert from "node:assert/strict";
import { fixClass, badge, sourceBadge, fmtAge, fmtSigma, fmtT, fmtNum, segmentTrail,
         SPEED_OPTIONS, eventKey }
  from "../web/js/protocol.js";

test("fixClass maps RtkFix quality 0-4 regardless of source", () => {
  assert.equal(fixClass("app", 0), "none");
  assert.equal(fixClass("app", 1), "bad");
  assert.equal(fixClass("fused", 2), "bad");
  assert.equal(fixClass("fused", 3), "float");
  assert.equal(fixClass("app", 4), "fixed");
  assert.equal(fixClass("app", null), "none");
  assert.equal(fixClass("app", undefined), "none");
});

test("badge prefers app, falls back to fused, honors verdict", () => {
  const sol = { q: 4 }, can = { q: 3 };
  assert.deepEqual(badge({ sol, can, verdict: { level: "ok" } }),
                   { cls: "fixed", text: "RTK 固定" });
  assert.deepEqual(badge({ sol: null, can, verdict: { level: "ok" } }),
                   { cls: "float", text: "浮点解" });
  const b = badge({ sol: null, can: null,
                    verdict: { level: "warning", code: "no_data", message: "无数据——检查采集链路" } });
  assert.equal(b.cls, "none");
  assert.equal(b.text, "无数据——检查采集链路");
  const s = badge({ sol, can, verdict: { level: "serious", message: "差分中断 5s" } });
  assert.equal(s.cls, "fixed");
  assert.equal(s.text, "差分中断 5s");
});

test("sourceBadge shows five quality levels and greys missing data", () => {
  const base = { q: 4, sats: 20, age: 1, sigma_n: 0.01, sigma_e: 0.02,
                 ratio: 3.5, latency: 0.05, lat: 44.5 };
  assert.equal(sourceBadge("APP 自解", base).cls, "fixed");
  assert.equal(sourceBadge("APP 自解", { ...base, q: 3 }).cls, "float");
  assert.equal(sourceBadge("APP 自解", { ...base, q: 2 }).cls, "bad");
  assert.equal(sourceBadge("APP 自解", { ...base, q: 1 }).cls, "bad");
  assert.equal(sourceBadge("APP 自解", { ...base, q: 0 }).cls, "none");
  const none = sourceBadge("610 融合", null);
  assert.equal(none.cls, "none");
  assert.equal(none.text, "无数据");
  const stale = sourceBadge("610 融合", { ...base, stale: true });
  assert.equal(stale.text, "无数据");
  const b = sourceBadge("APP 自解", base);
  assert.equal(b.sats, 20);
  assert.equal(b.sigma, fmtSigma(0.01, 0.02));
  assert.equal(b.ratio, "3.5");
  assert.equal(b.latency, "50 ms");
});

test("formatters render null as em-dash", () => {
  assert.equal(fmtAge(null), "—");
  assert.equal(fmtAge(0.8), "0.8 s");
  assert.equal(fmtSigma(0.011, 0.012), "1.6 cm");
  assert.equal(fmtSigma(null, 0.012), "—");
  assert.equal(fmtNum(undefined, 1, " m/s"), "—");
  assert.match(fmtT(0), /^\d{2}:\d{2}:\d{2}$/);
});

test("segmentTrail splits on quality class change with continuity", () => {
  const pts = [
    { lat: 1, lon: 1, src: "fused", q: 4 },
    { lat: 2, lon: 2, src: "fused", q: 4 },
    { lat: 3, lon: 3, src: "fused", q: 3 },
    { lat: 4, lon: 4, src: "fused", q: 3 },
  ];
  const segs = segmentTrail(pts);
  assert.equal(segs.length, 2);
  assert.equal(segs[0].cls, "fixed");
  assert.equal(segs[1].cls, "float");
  assert.deepEqual(segs[0].latlngs.at(-1), [2, 2]);
  assert.deepEqual(segs[1].latlngs[0], [2, 2]);   // continuity point
  assert.equal(segs[1].latlngs.length, 3);
});

test("speed options", () => assert.deepEqual(SPEED_OPTIONS, [1, 10, 60]));

test("eventKey identifies same open/close occurrence, distinguishes others", () => {
  const a = { t: 100, action: "open", code: "no_data" };
  const b = { t: 100, action: "open", code: "no_data" };
  const c = { t: 100, action: "close", code: "no_data" };
  const d = { t: 100, action: "open", code: "sat_low" };
  assert.equal(eventKey(a), eventKey(b));
  assert.notEqual(eventKey(a), eventKey(c));
  assert.notEqual(eventKey(a), eventKey(d));
});
