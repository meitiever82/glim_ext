// web/js/store.js
// NOTE: intra-web/js imports use relative paths so `node --test` can resolve
// them without a bundler or import map; web/app.js (never node-tested) uses
// absolute /js/... paths instead, matching how the FastAPI StaticFiles mount
// serves web/ at "/".
import { eventKey } from "./protocol.js";
import { linkTransitions } from "./linkcard.js";

const TRAIL_MAX = 3600, SERIES_MAX = 3600, EVENTS_MAX = 200, LOGS_MAX = 300;
const _EMPTY_TRAILS = { app: [], fused: [], can: [], rtkrcv: [], gpchc: [] };
const _EMPTY_SERIES = { t: [], sats: [], age: [], sigma: [], ratio: [], latency: [], corr_bps: [] };

export function createStore(Vue) {
  const s = Vue.reactive({
    connected: false, replaying: false, status: null, lastError: null,
    trails: { ..._EMPTY_TRAILS },
    events: [],
    logs: [],
    series: { ..._EMPTY_SERIES },
  });

  function pushTrail(src, lat, lon, q) {
    if (lat === null || lat === undefined) return;
    const arr = s.trails[src];
    if (!arr) return;
    arr.push({ lat, lon, src, q });
    if (arr.length > TRAIL_MAX) arr.splice(0, arr.length - TRAIL_MAX);
  }

  // Skip a push identical to the last point on this trail (same lat/lon/q):
  // a stale-epoch source (e.g. gpchc re-delivering the same fix while other
  // routes advance) would otherwise flood the trail with duplicate points.
  function pushTrailDedup(src, lat, lon, q) {
    const arr = s.trails[src], last = arr && arr[arr.length - 1];
    if (last && last.lat === lat && last.lon === lon && last.q === q) return;
    pushTrail(src, lat, lon, q);
  }

  // Shared by clearForReplay/resumeLive/replay_end: resets trails+series so
  // the map/timelines don't carry stale replay data (e.g. a straight line
  // from the last replayed point to the live position) into the next mode.
  function resetTrails() {
    s.trails = { ..._EMPTY_TRAILS };
    s.series = { ..._EMPTY_SERIES };
  }

  function pushSeries(key, v) {
    s.series[key].push(v);
    if (s.series[key].length > SERIES_MAX) s.series[key].shift();
  }

  // Unified log window: events plus link/diagnosis transitions, newest first.
  function pushLog(e) {
    s.logs.unshift(e);
    if (s.logs.length > LOGS_MAX) s.logs.pop();
  }

  s.applyMessage = (m) => {
    // Discard replay messages that arrive after the user has returned to live
    // (the server streams replay one message at a time, so a few are still in
    // flight when "回到实时" fires). Without this they append to the freshly
    // cleared trail and draw a straight line to the live position. Replay-mode
    // messages are accepted normally because s.replaying is still true then.
    if (m.replay && !s.replaying) return;
    if (m.type === "status") {
      const prev = s.status;
      s.status = m;
      for (const tr of linkTransitions(prev, m)) pushLog({ ...tr, t: m.t });
      // Two always-on sources: app (rtkrcv self solution), fused (610).
      // Replay exposes the same data under the older sol/can keys.
      const hasSources = !!(m.sources && (m.sources.app || m.sources.fused));
      const app = hasSources ? m.sources.app : m.sol;
      const fused = hasSources ? m.sources.fused : m.can;
      if (app) pushTrail("app", app.lat, app.lon, app.q);
      if (fused) pushTrailDedup("fused", fused.lat, fused.lon, fused.q);

      const sigmaOf = (x) => {
        if (!x) return null;
        if (x.sigma_n != null && x.sigma_e != null) return Math.hypot(x.sigma_n, x.sigma_e);
        if (x.sdn != null && x.sde != null) return Math.hypot(x.sdn, x.sde);
        return null;
      };
      pushSeries("t", m.t);
      pushSeries("sats", app?.sats ?? fused?.sats ?? null);
      pushSeries("age", app?.age ?? fused?.age ?? null);
      pushSeries("sigma", sigmaOf(app) ?? sigmaOf(fused));
      pushSeries("ratio", app?.ratio ?? fused?.ratio ?? null);
      pushSeries("latency", app?.latency ?? null);
      pushSeries("corr_bps", m.corr_rate ? m.corr_rate.rate_bps : null);
    } else if (m.type === "position") {
      pushTrail(m.src, m.lat, m.lon, m.q);
    } else if (m.type === "event") {
      const ev = { action: m.action, ...m.event };
      const key = eventKey(ev);
      if (!s.events.some((e) => eventKey(e) === key)) {
        s.events.unshift(ev);
        if (s.events.length > EVENTS_MAX) s.events.pop();
        pushLog({ kind: "event", action: ev.action, level: ev.level || "info",
                  text: ev.message || "", t: ev.t, code: ev.code });
      }
    } else if (m.type === "replay_end") {
      // Also clear trails/series: otherwise the map draws a straight line
      // from the last replayed point to wherever live resumes.
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
