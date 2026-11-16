// web/js/mapview.js — Leaflet map view: offline tiles with grid fallback,
// state-colored trails, vehicle marker with heading, sigma circle.
//
// NOTE: intra-web/js imports use relative paths so `node --test` can resolve
// them without a bundler (see web/js/store.js); web/app.js (never
// node-tested) imports this module via the absolute /js/mapview.js path.
import { segmentTrail } from "./protocol.js";

const COLORS = { fixed: "#3fb96c", float: "#e0b23c", bad: "#e05c4f", none: "#5a6472" };
export const trailColor = (cls) => COLORS[cls];

// 底图版本标签：Leaflet attribution 常显在右下角。内容为
// “归属 · 缓存文件名（含日期/缩放范围） · SHA-256 前 8 位”，
// 现场一眼能确认当前用的是哪份缓存（spec §5 验收：页面显示底图版本）。
export function basemapLabel(info) {
  if (!info || !info.available) return "";
  const parts = [info.attribution || info.source || "本地瓦片"];
  if (info.file) parts.push(String(info.file).replace(/\.mbtiles$/, ""));
  if (info.sha256) parts.push(String(info.sha256).slice(0, 8));
  return parts.filter(Boolean).join(" · ");
}

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

export class MapView {
  constructor(elId, store) {
    this.store = store;
    // preferCanvas: rendering trails (frequently redrawn polylines) as canvas
    // rather than SVG DOM elements avoids per-point DOM churn on long trails.
    this.map = L.map(elId, { zoomControl: true, preferCanvas: true }).setView([0, 0], 3);
    this.map.on("dragstart", () => { this._userMoved = true; });

    // Probe /api/tiles_info rather than a fixed tile coordinate: with a
    // mine-only tileset, any single hardcoded tile (e.g. z12/3000/1500) can
    // legitimately be absent while the tile store itself is populated, which
    // would false-negative a fixed-tile probe into the grid fallback.
    // Basemap selection: prefer the offline MBTiles store when configured.
    // Otherwise fall back to Esri World Imagery online (WGS-84 / EPSG:3857 —
    // aligns with the RTK track, unlike GCJ-02/BD-09 sources), with the local
    // coordinate grid layered underneath so a no-internet vehicle still gets
    // a usable backdrop instead of blank tiles.
    const useOnlineFallback = () => {
      new GridFallback().addTo(this.map);
      L.tileLayer(
        "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
        { maxZoom: 19, attribution: "Esri World Imagery" }).addTo(this.map);
    };
    fetch("/api/tiles_info").then((r) => r.json()).then((info) => {
      if (info && info.available) {
        L.tileLayer("/tiles/{z}/{x}/{y}.png",
          { maxZoom: 22, attribution: basemapLabel(info) }).addTo(this.map);
      } else {
        useOnlineFallback();
      }
    }).catch(useOnlineFallback);

    this.groups = {
      app: L.layerGroup().addTo(this.map),
      fused: L.layerGroup().addTo(this.map),
    };
    // Independent show/hide per trajectory (§5): a Leaflet overlay control
    // toggles each trail's layerGroup on/off. render() keeps drawing into the
    // groups regardless; hidden groups just aren't on the map.
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
    this._trailLen = { app: -1, fused: -1 };
  }

  _style(src, cls) {
    const base = { color: trailColor(cls) };
    if (src === "app") return { ...base, weight: 3 };
    return { ...base, weight: 2, opacity: 0.8 };
  }

  render() {
    const srcs = (this.store.status || {}).sources || {};
    for (const src of ["app", "fused"]) {
      const trail = this.store.trails[src];
      const available = !srcs[src] || srcs[src].available !== false;
      if (trail.length !== this._trailLen[src] || available !== this._srcAvail?.[src]) {
        this._trailLen[src] = trail.length;
        this._srcAvail = { ...(this._srcAvail || {}), [src]: available };
        const g = this.groups[src];
        g.clearLayers();
        for (const seg of segmentTrail(trail))
          L.polyline(seg.latlngs, this._style(src, seg.cls)).addTo(g);
      }
      // 数据中断时保留最后轨迹，整体降透明度。
      this.groups[src].eachLayer((l) => {
        if (l.setStyle) l.setStyle({ opacity: available ? 1 : 0.35 });
      });
    }

    const st = this.store.status || {};
    const primary = srcs.app && srcs.app.lat != null ? srcs.app
      : (srcs.fused && srcs.fused.lat != null ? srcs.fused : null);
    if (primary && primary.available === false && this.marker) {
      // 保留最后位置但不伪装成新解
      this.marker.setOpacity(0.35);
    } else if (this.marker) {
      this.marker.setOpacity(1);
    }
    // Hide the sigma circle once its inputs go away (e.g. source dropped out,
    // or sigma_h missing) instead of leaving a stale circle at the last position.
    if (this.sigma && (!primary || primary.sigma_h == null)) {
      this.sigma.remove();
      this.sigma = null;
    }
    if (!primary) return;

    const ll = [primary.lat, primary.lon];
    // .veh-arrow (style.css) already styles an inner <svg> arrow sized to
    // the divIcon. The divIcon itself is created once and reused: rotation
    // is applied by mutating the existing DOM element's transform instead of
    // rebuilding the icon HTML (and its DOM) on every render.
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
      // heading 无效（null/undefined）时不显示箭头，避免指向正北的假姿态。
      const heading = primary.heading;
      arrowEl.style.display = heading === null || heading === undefined ? "none" : "";
      if (heading !== null && heading !== undefined)
        arrowEl.style.transform = `rotate(${heading}deg)`;
    }

    if (primary.sigma_h != null) {
      const r = primary.sigma_h;
      if (!this.sigma) this.sigma = L.circle(ll, { radius: r, color: "#4a90d9", weight: 1 }).addTo(this.map);
      else { this.sigma.setLatLng(ll); this.sigma.setRadius(r); }
    }

    if (!this._centered) { this.map.setView(ll, 17); this._centered = true; }
    else if (!this._userMoved) this.map.panTo(ll, { animate: false });
  }
}
