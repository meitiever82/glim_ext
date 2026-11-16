// web/js/store.js
// NOTE: intra-web/js imports use relative paths so `node --test` can resolve
// them without a bundler or import map; web/app.js (never node-tested) uses
// absolute /js/... paths instead, matching how the FastAPI StaticFiles mount
// serves web/ at "/".
import { eventKey } from "./protocol.js";

const TRAIL_MAX = 3600, SERIES_MAX = 1800, EVENTS_MAX = 200;
const SOURCES = ["app", "fused"];

export function pickPrimary(sources) {
  const app = sources && sources.app, fused = sources && sources.fused;
  if (app && app.available && app.lat != null) return app;
  if (fused && fused.available && fused.lat != null) return fused;
  return app || fused || {};
}

export function createStore(Vue) {
  const s = Vue.reactive({
    connected: false, replaying: false, status: null, lastError: null,
    trails: { app: [], fused: [] },
    events: [],
    series: { t: [], sats: [], age: [], sigma: [], ratio: [], latency: [], corr: [] },
  });

  function pushTrail(src, lat, lon, q) {
    const arr = s.trails[src];
    if (!arr) return;
    arr.push({ lat, lon, src, q });
    if (arr.length > TRAIL_MAX) arr.splice(0, arr.length - TRAIL_MAX);
  }

  // Skip a push identical to the last point on this trail (same lat/lon/q):
  // a stale-epoch source would otherwise flood the trail with duplicates.
  function pushTrailDedup(src, lat, lon, q) {
    if (lat === null || lat === undefined) return;
    const arr = s.trails[src], last = arr[arr.length - 1];
    if (last && last.lat === lat && last.lon === lon && last.q === q) return;
    pushTrail(src, lat, lon, q);
  }

  // Shared by clearForReplay/resumeLive/replay_end: resets trails+series so
  // the map/timelines don't carry stale replay data into the next mode.
  function resetTrails() {
    s.trails = { app: [], fused: [] };
    s.series = { t: [], sats: [], age: [], sigma: [], ratio: [], latency: [], corr: [] };
  }

  s.applyMessage = (m) => {
    // Discard replay messages that arrive after the user has returned to live.
    if (m.replay && !s.replaying) return;
    if (m.type === "status") {
      s.status = m;
      const srcs = m.sources || {};
      for (const key of SOURCES) {
        const src = srcs[key];
        // 无数据时保留最后位置（地图降透明度），但不追加重复点。
        if (src && src.lat != null && src.available !== false) {
          pushTrailDedup(key, src.lat, src.lon, src.quality);
        }
      }
      const p = pickPrimary(srcs);
      const corr = m.corr_rate || {};
      for (const [k, v] of Object.entries({
        t: m.t,
        sats: p.sats ?? null,
        age: p.age_s ?? null,
        sigma: p.sigma_h ?? null,
        ratio: p.ratio ?? null,
        latency: p.latency_s ?? null,
        corr: corr.rtcm_bps ?? null,
      })) {
        s.series[k].push(v);
        if (s.series[k].length > SERIES_MAX) s.series[k].shift();
      }
    } else if (m.type === "position") {
      if (m.src !== "app" && m.src !== "fused") return;
      pushTrailDedup(m.src, m.lat, m.lon, m.q);
    } else if (m.type === "event") {
      const ev = { action: m.action, ...m.event };
      const key = eventKey(ev);
      if (!s.events.some((e) => eventKey(e) === key)) {
        s.events.unshift(ev);
        if (s.events.length > EVENTS_MAX) s.events.pop();
      }
    } else if (m.type === "replay_end") {
      resetTrails();
      s.replaying = false;
    } else if (m.type === "error") {
      s.lastError = m.detail; s.replaying = false;
    }
  };

  s.clearForReplay = () => {
    resetTrails();
    s.replaying = true;
  };

  // Used by replaybar's "回到实时" button to leave replay mode explicitly
  // (as opposed to replay_end, which fires when a replay window finishes).
  s.resumeLive = () => {
    resetTrails();
    s.replaying = false;
  };

  return s;
}
