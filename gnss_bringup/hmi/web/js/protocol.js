// web/js/protocol.js — pure logic, no DOM; tested with node --test.
export const SPEED_OPTIONS = [1, 10, 60];

// gnss_msgs/RtkFix.quality（spec 数据映射）：
// 0 NONE → none；1 SINGLE / 2 DGPS → bad；3 FLOAT → float；4 FIXED → fixed。
const _QUALITY_CLASS = ["none", "bad", "bad", "float", "fixed"];

export function fixClass(q) {
  if (q === null || q === undefined || Number.isNaN(Number(q))) return "none";
  return _QUALITY_CLASS[Number(q)] || "bad";
}

const _BADGE_TEXT = { fixed: "RTK 固定", float: "浮点解", bad: "非固定", none: "无数据" };

// 每路来源（status.sources.app / .fused）独立取徽标，无数据不隐藏。
export function badge(source) {
  if (!source || source.available === false) {
    return { cls: "none", text: (source && source.reason) || "无数据" };
  }
  const cls = fixClass(source.quality);
  return { cls, text: _BADGE_TEXT[cls] };
}

export function qualityText(q) {
  return _BADGE_TEXT[fixClass(q)];
}

export function fmtNum(v, digits, suffix) {
  if (v === null || v === undefined || Number.isNaN(Number(v))) return "—";
  return Number(v).toFixed(digits) + (suffix || "");
}

export const fmtAge = (v) => fmtNum(v, 1, " s");

// 水平 σ：后端已按 E/N 合成 sigma_h（单位 m），这里只做 cm 显示。
export function fmtSigma(sigmaH) {
  if (sigmaH === null || sigmaH === undefined) return "—";
  return (Number(sigmaH) * 100).toFixed(1) + " cm";
}

export function fmtLatency(v) {
  if (v === null || v === undefined) return "—";
  return (Number(v) * 1000).toFixed(0) + " ms";
}

export function fmtBps(v) {
  if (v === null || v === undefined) return "—";
  const n = Number(v);
  return n >= 1000 ? (n / 1000).toFixed(1) + " kB/s" : n.toFixed(0) + " B/s";
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

// 诊断事件来自 /gnss/diagnostics：action open/close + code + level + message。
export function segmentTrail(points) {
  const segs = [];
  let cur = null;
  for (const p of points) {
    const cls = fixClass(p.q);
    if (!cur || cur.cls !== cls) {
      const start = cur ? [cur.latlngs.at(-1)] : [];
      cur = { cls, latlngs: [...start] };
      segs.push(cur);
    }
    cur.latlngs.push([p.lat, p.lon]);
  }
  return segs.filter((s) => s.latlngs.length >= 2);
}
