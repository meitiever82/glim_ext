// web/js/mapview.js — Leaflet map view: tiles with grid fallback, state-colored
// trails for both sources (APP / 610 fused), heading arrow, sigma circle.
//
// NOTE: intra-web/js imports use relative paths so `node --test` can resolve
// them without a bundler (see web/js/store.js); web/app.js (never node-tested)
// imports this module via the absolute /js/mapview.js path.
import { segmentTrail } from "./protocol.js";

const COLORS = { fixed: "#3fb96c", float: "#e0b23c", bad: "#e05c4f", none: "#5a6472" };
export const trailColor = (cls) => COLORS[cls];

// Grid fallback tile layer: draws a bordered canvas tile labeled z/x/y so an
// offline deployment without tiles still shows a readable grid instead of a
// blank pane. Guarded by `typeof L` so this module stays importable under
// `node --test` (no global Leaflet there) while trailColor is exercised in
// isolation — see tests_js/mapview.test.mjs.
const GridFallback = typeof L !== "undefined" ? L.GridLayer.extend({
  createTile(coords) {
    const c = document.createElement("canvas");
    c.width = c.height = 256;
    const g = c.getContext("2d");
    g.strokeStyle = "#2a3340";
    g.strokeRect(0, 0, 256, 256);
    g.fillStyle = "#5a6472";
    g.font = "12px sans-serif";
    g.fillText(`${coords.z}/${coords.x}/${coords.y}`, 8, 16);
    return c;
  },
}) : null;

const TRAIL_SRCS = ["app", "fused", "can", "rtkrcv", "gpchc"];

export class MapView {
  constructor(elId, store) {
    this.store = store;
    // preferCanvas: rendering trails (frequently redrawn polylines) as canvas
    // rather than SVG DOM elements avoids per-point DOM churn on long trails.
    this.map = L.map(elId, { zoomControl: true, preferCanvas: true }).setView([0, 0], 3);
    this.map.on("dragstart", () => { this._userMoved = true; });

    // Local cached tiles first; otherwise Esri World Imagery online (WGS-84 /
    // EPSG:3857 — aligns with the RTK track, unlike GCJ-02/BD-09 sources), with
    // the local coordinate grid underneath so a no-internet vehicle still gets
    // a usable backdrop instead of blank tiles.
    const useOnlineFallback = () => {
      new GridFallback().addTo(this.map);
      L.tileLayer(
        "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
        { maxZoom: 19, attribution: "Esri World Imagery" }).addTo(this.map);
    };
    fetch("/api/tiles_info").then((r) => r.json()).then((info) => {
      if (info && info.available) L.tileLayer("/tiles/{z}/{x}/{y}.png",
        { maxZoom: 22,
          attribution: "矿区影像" + (info.version ? " · " + info.version : "") }).addTo(this.map);
      else useOnlineFallback();
    }).catch(useOnlineFallback);

    this.groups = {
      app: L.layerGroup().addTo(this.map),
      fused: L.layerGroup().addTo(this.map),
      can: L.layerGroup().addTo(this.map),
      rtkrcv: L.layerGroup().addTo(this.map),
      gpchc: L.layerGroup().addTo(this.map),
    };
    // Independent show/hide per trajectory (§5): both sources default on.
    L.control.layers(null, {
      "APP 自解": this.groups.app,
      "610 融合": this.groups.fused,
    }, { collapsed: false }).addTo(this.map);
    this.marker = null;
    this.sigma = null;
    this._centered = false;
    this._userMoved = false;
    // Per-src trail length as last rendered: lets render() skip
    // clearLayers()+redraw for a trail that hasn't grown/reset since the
    // last frame (replay resets an array to [] -> length changes -> redraw).
    this._trailLen = Object.fromEntries(TRAIL_SRCS.map((s) => [s, -1]));
  }

  _style(src, cls) {
    const base = { color: trailColor(cls) };
    if (src === "app") return { ...base, weight: 3 };
    if (src === "fused") return { ...base, weight: 2, opacity: 0.9 };
    return { ...base, weight: 2, opacity: 0.7 };
  }

  render() {
    for (const src of TRAIL_SRCS) {
      const trail = this.store.trails[src];
      if (!trail || trail.length === this._trailLen[src]) continue;   // unchanged
      this._trailLen[src] = trail.length;
      const g = this.groups[src];
      if (!g) continue;
      g.clearLayers();
      for (const seg of segmentTrail(trail))
        L.polyline(seg.latlngs, this._style(src, seg.cls)).addTo(g);
    }

    const st = this.store.status || {};
    const srcs = st.sources || {};
    // Replay carries the same data under sol/can.
    const app = srcs.app || (st.sol ? { lat: st.sol.lat, lon: st.sol.lon, q: st.sol.q,
                                       sigma_n: st.sol.sdn, sigma_e: st.sol.sde } : null);
    const fused = srcs.fused || (st.can ? { lat: st.can.lat, lon: st.can.lon, q: st.can.q,
                                            heading: st.can.heading, sigma_n: st.can.sdn,
                                            sigma_e: st.can.sde } : null);
    const pos = (fused && fused.lat != null) ? fused : (app && app.lat != null ? app : null);
    if (!pos) return;
    const appPos = (app && app.lat != null) ? app : pos;

    if (this.sigma && (!appPos || appPos.sigma_n == null)) {
      this.sigma.remove();
      this.sigma = null;
    }

    const ll = [pos.lat, pos.lon];
    const heading = pos.heading;
    // .veh-arrow (style.css) already styles an inner <svg> arrow sized to the
    // divIcon. The divIcon itself is created once and reused: rotation
    // is applied by mutating the existing DOM element's transform instead of
    // rebuilding the icon HTML (and its DOM) on every render. A missing /
    // invalid heading hides the arrow (never draws it pointing north).
    if (!this.marker) {
      const icon = L.divIcon({
        className: "veh",
        html: `<div class="veh-arrow"><svg viewBox="0 0 24 24"><polygon points="12,2 20,22 12,17 4,22"/></svg></div>`,
        iconSize: [18, 18],
        iconAnchor: [9, 9],
      });
      this.marker = L.marker(ll, { icon }).addTo(this.map);
    } else {
      this.marker.setLatLng(ll);
    }
    const arrowEl = this.marker.getElement()?.querySelector(".veh-arrow");
    if (arrowEl) {
      const valid = heading !== null && heading !== undefined && !Number.isNaN(heading);
      arrowEl.style.display = valid ? "" : "none";
      if (valid) arrowEl.style.transform = `rotate(${heading}deg)`;
    }

    if (appPos && appPos.sigma_n != null && appPos.sigma_e != null) {
      const r = Math.hypot(appPos.sigma_n, appPos.sigma_e);
      if (!this.sigma) this.sigma = L.circle(ll, { radius: r, color: "#4a90d9", weight: 1 }).addTo(this.map);
      else { this.sigma.setLatLng(ll); this.sigma.setRadius(r); }
    }

    if (!this._centered) { this.map.setView(ll, 17); this._centered = true; }
    else if (!this._userMoved) this.map.panTo(ll, { animate: false });
  }
}
