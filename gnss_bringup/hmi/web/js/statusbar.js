// web/js/statusbar.js — two fixed source badges (APP self-solution / 610 fused).
import { sourceBadge, fmtAge, fmtNum, fmtT } from "./protocol.js";

export function mountStatusbar(Vue, store) {
  Vue.createApp({
    computed: {
      st() { return store.status || {}; },
      ab() { return sourceBadge("APP 自解", this.st.sources && this.st.sources.app); },
      fb() { return sourceBadge("610 融合", this.st.sources && this.st.sources.fused); },
      speed() { return fmtNum(this.st.can && this.st.can.speed, 1, " m/s"); },
      latency() {
        const a = this.st.sources && this.st.sources.app;
        return a && a.latency != null ? fmtAge(Math.abs(a.latency)) : "—";
      },
      verdict() { return this.st.verdict || {}; },
      verdictText() {
        return ["warning", "serious", "critical"].includes(this.verdict.level)
          ? this.verdict.message : "";
      },
      clock() { return fmtT(this.st.t); },
      replaying() { return store.replaying; },
    },
    template: `
      <div class="statusbar">
        <span class="badge src" :class="ab.cls">
          <b>APP 自解</b> {{ ab.text }}
          <small>星 {{ ab.sats }} · 龄 {{ ab.age }} · σ {{ ab.sigma }} · R {{ ab.ratio }}</small>
        </span>
        <span class="badge src" :class="fb.cls">
          <b>610 融合</b> {{ fb.text }}
          <small>星 {{ fb.sats }} · 龄 {{ fb.age }} · σ {{ fb.sigma }} · R {{ fb.ratio }}</small>
        </span>
        <span v-if="verdictText" class="kv verdict">{{ verdictText }}</span>
        <span class="kv">速度 <b>{{ speed }}</b></span>
        <span class="kv">延迟 <b>{{ latency }}</b></span>
        <span class="kv time">{{ clock }}</span>
        <span v-if="replaying" class="tag replay">回放</span>
      </div>`,
  }).mount("#statusbar");
}
