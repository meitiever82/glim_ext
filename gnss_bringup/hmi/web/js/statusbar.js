// web/js/statusbar.js
import { badge, fmtAge, fmtSigma, fmtNum, fmtLatency, fmtT } from "./protocol.js";

export function mountStatusbar(Vue, store) {
  Vue.createApp({
    computed: {
      st() { return store.status || {}; },
      appS() { return (this.st.sources || {}).app || {}; },
      fusedS() { return (this.st.sources || {}).fused || {}; },
      appB() { return badge(this.appS); },
      fusedB() { return badge(this.fusedS); },
      appSats() { return this.appS.sats ?? "—"; },
      fusedSats() { return this.fusedS.sats ?? "—"; },
      appAge() { return fmtAge(this.appS.age_s); },
      fusedAge() { return fmtAge(this.fusedS.age_s); },
      appSigma() { return fmtSigma(this.appS.sigma_h); },
      fusedSigma() { return fmtSigma(this.fusedS.sigma_h); },
      appRatio() { return fmtNum(this.appS.ratio, 1); },
      fusedRatio() { return fmtNum(this.fusedS.ratio, 1); },
      speed() { return fmtNum(this.appS.speed ?? this.fusedS.speed, 1, " m/s"); },
      latency() { return fmtLatency(this.appS.latency_s ?? this.fusedS.latency_s); },
      clock() { return fmtT(this.st.t); },
      replaying() { return store.replaying; },
      stale() { return this.st.stale === true; },
      verdictText() { return (this.st.verdict || {}).message || ""; },
      verdictCls() {
        const lv = (this.st.verdict || {}).level || "ok";
        return lv === "ok" ? "" : "v-" + lv;
      },
    },
    template: `
      <div class="statusbar">
        <span class="src">
          <span :class="'badge ' + appB.cls">APP 自解 · {{ appB.text }}</span>
          <span class="kv">卫星 <b>{{ appSats }}</b></span>
          <span class="kv">龄期 <b>{{ appAge }}</b></span>
          <span class="kv">σ <b>{{ appSigma }}</b></span>
          <span class="kv">ratio <b>{{ appRatio }}</b></span>
        </span>
        <span class="src">
          <span :class="'badge ' + fusedB.cls">610 融合 · {{ fusedB.text }}</span>
          <span class="kv">卫星 <b>{{ fusedSats }}</b></span>
          <span class="kv">龄期 <b>{{ fusedAge }}</b></span>
          <span class="kv">σ <b>{{ fusedSigma }}</b></span>
          <span class="kv">ratio <b>{{ fusedRatio }}</b></span>
        </span>
        <span v-if="replaying" class="tag replay">回放</span>
        <span v-if="stale && !replaying" class="tag stale">数据中断</span>
        <span v-if="verdictText && !replaying" :class="'tag ' + verdictCls">{{ verdictText }}</span>
        <span class="kv">速度 <b>{{ speed }}</b></span>
        <span class="kv">延迟 <b>{{ latency }}</b></span>
        <span class="kv time">{{ clock }}</span>
      </div>`,
  }).mount("#statusbar");
}
