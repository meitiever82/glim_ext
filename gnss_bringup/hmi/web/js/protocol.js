// web/js/protocol.js — pure logic, no DOM; tested with node --test.
export const SPEED_OPTIONS = [1, 10, 60];

// gnss_msgs/RtkFix.quality is source-independent:
// 0 NONE, 1 SINGLE, 2 DGPS, 3 FLOAT, 4 FIXED.
const _QUALITY_CLASS = { 0: "none", 1: "bad", 2: "bad", 3: "float", 4: "fixed" };

export function fixClass(src, q) {
  if (q === null || q === undefined) return "none";
  return _QUALITY_CLASS[q] ?? "bad";
}

const _BADGE_TEXT = { fixed: "RTK 固定", float: "浮点解", bad: "非固定", none: "无数据" };

// Legacy single badge (kept for compatibility with older clients/tests).
export function badge(status) {
  const sol = status.sol, can = status.can, v = status.verdict || {};
  let cls;
  if (sol && sol.q !== null && sol.q !== undefined) cls = fixClass("app", sol.q);
  else if (can && can.q !== null && can.q !== undefined) cls = fixClass("fused", can.q);
  else cls = "none";
  let text = _BADGE_TEXT[cls];
  if (["warning", "serious", "critical"].includes(v.level) && v.message) text = v.message;
  return { cls, text };
}

// One of the two always-visible source badges (APP 自解 / 610 融合).
// Missing or stale data greys the badge instead of hiding it.
export function sourceBadge(name, st) {
  const empty = { name, cls: "none", text: "无数据", sats: "—", age: "—",
                  sigma: "—", ratio: "—", latency: "—" };
  if (!st || st.stale || st.lat === null || st.lat === undefined) return empty;
  const cls = fixClass(name, st.q);
  return {
    name,
    cls,
    text: _BADGE_TEXT[cls],
    sats: st.sats ?? "—",
    age: fmtAge(st.age),
    sigma: fmtSigma(st.sigma_n, st.sigma_e),
    ratio: st.ratio === null || st.ratio === undefined ? "—" : st.ratio.toFixed(1),
    latency: fmtLatency(st.latency),
  };
}

export function fmtNum(v, digits, suffix) {
  if (v === null || v === undefined || Number.isNaN(v)) return "—";
  return v.toFixed(digits) + (suffix || "");
}

export const fmtAge = (v) => fmtNum(v, 1, " s");

export function fmtSigma(sdn, sde) {
  if (sdn === null || sdn === undefined || sde === null || sde === undefined) return "—";
  return (Math.hypot(sdn, sde) * 100).toFixed(1) + " cm";
}

export function fmtLatency(v) {
  if (v === null || v === undefined || Number.isNaN(v)) return "—";
  return (v * 1000).toFixed(0) + " ms";
}

export function fmtBytesPerSec(v) {
  if (v === null || v === undefined || Number.isNaN(v)) return "—";
  if (v >= 1024) return (v / 1024).toFixed(1) + " KiB/s";
  return v.toFixed(0) + " B/s";
}

export function fmtT(t) {
  if (t === null || t === undefined) return "—";
  return new Date(t * 1000).toTimeString().slice(0, 8);
}

// Dedup key for an event message merged as { action, ...event } (see store.js
// applyMessage's "event" branch): identifies the same open/close occurrence
// so a duplicate push (e.g. re-delivered on reconnect) is skipped.
export function eventKey(e) {
  return `${e.t}|${e.action}|${e.code}`;
}

export function segmentTrail(points) {
  const segs = [];
  let cur = null;
  for (const p of points) {
    const cls = fixClass(p.src, p.q);
    if (!cur || cur.cls !== cls) {
      const start = cur ? [cur.latlngs.at(-1)] : [];
      cur = { cls, latlngs: [...start] };
      segs.push(cur);
    }
    cur.latlngs.push([p.lat, p.lon]);
  }
  return segs.filter((s) => s.latlngs.length >= 2);
}
