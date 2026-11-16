import test from "node:test";
import assert from "node:assert/strict";
import { fixClass, badge, fmtAge, fmtSigma, fmtLatency, fmtBps, fmtT, fmtNum,
         segmentTrail, SPEED_OPTIONS, eventKey } from "../web/js/protocol.js";

test("fixClass maps RtkFix.quality 0-4", () => {
  assert.equal(fixClass(0), "none");
  assert.equal(fixClass(1), "bad");       // SINGLE
  assert.equal(fixClass(2), "bad");       // DGPS
  assert.equal(fixClass(3), "float");
  assert.equal(fixClass(4), "fixed");
  assert.equal(fixClass(null), "none");
  assert.equal(fixClass(undefined), "none");
  assert.equal(fixClass(7), "bad");       // 未知档位按非固定处理
});

test("badge is per-source and grays out when unavailable", () => {
  assert.deepEqual(badge({ available: true, quality: 4 }), { cls: "fixed", text: "RTK 固定" });
  assert.deepEqual(badge({ available: true, quality: 3 }), { cls: "float", text: "浮点解" });
  assert.deepEqual(badge({ available: false, quality: 4, reason: "数据中断" }),
                   { cls: "none", text: "数据中断" });
  assert.deepEqual(badge({ available: false }), { cls: "none", text: "无数据" });
  assert.deepEqual(badge(null), { cls: "none", text: "无数据" });
});

test("formatters render null as em-dash", () => {
  assert.equal(fmtAge(null), "—");
  assert.equal(fmtAge(0.8), "0.8 s");
  // sigma_h 已是米；只做 cm 显示
  assert.equal(fmtSigma(0.016), "1.6 cm");
  assert.equal(fmtSigma(null), "—");
  assert.equal(fmtLatency(0.042), "42 ms");
  assert.equal(fmtLatency(null), "—");
  assert.equal(fmtBps(1500), "1.5 kB/s");
  assert.equal(fmtBps(null), "—");
  assert.equal(fmtNum(undefined, 1, " m/s"), "—");
  assert.match(fmtT(0), /^\d{2}:\d{2}:\d{2}$/);
});

test("segmentTrail splits on class change with continuity", () => {
  const pts = [
    { lat: 1, lon: 1, src: "app", q: 4 },
    { lat: 2, lon: 2, src: "app", q: 4 },
    { lat: 3, lon: 3, src: "app", q: 3 },
    { lat: 4, lon: 4, src: "app", q: 3 },
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
