// web/js/timeline.js
export function sigmaSeriesCm(arr) {
  return arr.map((v) => (v === null || v === undefined ? null : Math.round(v * 1000) / 10));
}

export function latencySeriesMs(arr) {
  return arr.map((v) => (v === null || v === undefined ? null : Math.round(v * 1000)));
}

// Trim the trailing window to `maxN` samples (1 Hz -> 1 h = 3600).
export function trimSeries(arr, maxN) {
  return arr.length > maxN ? arr.slice(arr.length - maxN) : arr;
}

const AXIS = { stroke: "#8b95a3", grid: { stroke: "#232b35" }, ticks: { stroke: "#232b35" } };

// Zoom preservation (I5, pragmatic): uPlot has no first-class "did the user
// zoom" flag, so we track it ourselves, per chart, as a plain property on the
// uPlot instance (chart._userZoom). A drag-selection with nonzero width
// (uPlot's "setSelect" hook) means the user is zooming in; a double-click on
// the chart root is uPlot's built-in reset-to-full-data gesture. While
// _userZoom is set, setData(data, false) is used so uPlot keeps the current
// x-scale instead of re-fitting it to the new data on every incoming point.
//
// Cross-chart sync: a drag-zoom on any chart propagates its x-range to the
// other three (and marks them zoomed), and a double-click resets all four, so
// the four timelines always share one x-window. `shared` holds the chart list
// and a re-entrancy guard; it is populated after all charts are constructed.
// (Cursor position is separately synced via the `sync: { key: "rtk" }` group.)
function mkChart(el, label, color, shared) {
  const chart = new uPlot({
    width: el.clientWidth || 600, height: 110,
    cursor: { sync: { key: "rtk" } },
    legend: { show: false },
    scales: { x: { time: true } },
    axes: [ { ...AXIS }, { ...AXIS, label } ],
    series: [ {}, { label, stroke: color, width: 2, spanGaps: false } ],
    hooks: {
      setSelect: [(u) => {
        if (!(u.select && u.select.width > 0)) return;
        u._userZoom = true;
        if (shared.syncing) return;
        // translate the pixel selection to data x-values and apply the same
        // window to every sibling so all four zoom together
        const min = u.posToVal(u.select.left, "x");
        const max = u.posToVal(u.select.left + u.select.width, "x");
        shared.syncing = true;
        for (const other of shared.charts) {
          if (other !== u) { other._userZoom = true; other.setScale("x", { min, max }); }
        }
        shared.syncing = false;
      }],
    },
  }, [[], []], el);
  chart._userZoom = false;
  // double-click resets every chart to full data on the next render
  el.addEventListener("dblclick", () => {
    for (const c of shared.charts) c._userZoom = false;
  });
  return chart;
}

export class Timelines {
  constructor(store) {
    this.store = store;
    const shared = { charts: [], syncing: false };
    this.charts = {
      sats: mkChart(document.getElementById("chart-sats"), "卫星数", "#4a90d9", shared),
      age: mkChart(document.getElementById("chart-age"), "龄期 s", "#3fb96c", shared),
      sigma: mkChart(document.getElementById("chart-sigma"), "σ cm", "#e0b23c", shared),
      ratio: mkChart(document.getElementById("chart-ratio"), "ratio", "#b07ad9", shared),
      latency: mkChart(document.getElementById("chart-latency"), "接收延迟 ms", "#4aa3d9", shared),
      corr: mkChart(document.getElementById("chart-corr"), "差分 B/s", "#d98a4a", shared),
    };
    shared.charts.push(...Object.values(this.charts));
  }
  render() {
    const s = this.store.series;
    const MAX = 3600;   // 1 Hz -> 1 h window
    const t = trimSeries(s.t, MAX);
    const y = (arr) => trimSeries(arr, MAX);
    const set = (chart, data) => chart.setData(data, !chart._userZoom);
    set(this.charts.sats, [t, y(s.sats)]);
    set(this.charts.age, [t, y(s.age)]);
    set(this.charts.sigma, [t, y(sigmaSeriesCm(s.sigma))]);
    set(this.charts.ratio, [t, y(s.ratio)]);
    set(this.charts.latency, [t, y(latencySeriesMs(s.latency))]);
    set(this.charts.corr, [t, y(s.corr_bps)]);
  }
}
