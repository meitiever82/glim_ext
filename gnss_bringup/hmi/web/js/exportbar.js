// web/js/exportbar.js
// Data export: HTML report and RTKLIB `.pos` download for an explicit window.
// Both endpoints read stored epochs, so they are empty until storage.enabled.

export function exportRange(t0, t1) {
  const a = new Date(t0).getTime(), b = new Date(t1).getTime();
  if (!Number.isFinite(a) || !Number.isFinite(b) || !(b > a)) return null;
  return [a / 1000, b / 1000];
}

export function reportUrl(t0, t1) {
  const r = exportRange(t0, t1);
  return r && `/report?t0=${r[0]}&t1=${r[1]}`;
}

export function posUrl(t0, t1, src) {
  const r = exportRange(t0, t1);
  return r && `/api/pos?src=${encodeURIComponent(src)}&t0=${r[0]}&t1=${r[1]}`;
}

export function mountExportbar(Vue) {
  Vue.createApp({
    data() {
      const now = new Date(), ago = new Date(Date.now() - 3600e3);
      const fmt = (d) => new Date(d - d.getTimezoneOffset() * 60e3).toISOString().slice(0, 16);
      return {
        t0: fmt(ago), t1: fmt(now), src: "app",
        sources: [{ v: "app", label: "APP 自解" }, { v: "fused", label: "610 融合" }],
      };
    },
    computed: {
      report() { return reportUrl(this.t0, this.t1); },
      pos() { return posUrl(this.t0, this.t1, this.src); },
    },
    template: `
      <div class="exportbar">
        <label>导出 <input type="datetime-local" v-model="t0"></label>
        <label>至 <input type="datetime-local" v-model="t1"></label>
        <select v-model="src"><option v-for="s in sources" :value="s.v">{{ s.label }}</option></select>
        <a v-if="report" :href="report" target="_blank" rel="noopener">HTML 报告</a>
        <a v-if="pos" :href="pos" download>下载 .pos</a>
      </div>`,
  }).mount("#exportbar");
}
