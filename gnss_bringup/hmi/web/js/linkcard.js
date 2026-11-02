// web/js/linkcard.js — read-only link health card + status-change log entries.
export function linkState(st) {
  const l = (st && st.link) || {};
  const pick = (r) => ({
    bps: r && r.rate_bps != null ? r.rate_bps : null,
    gap: r && r.gap_s != null ? r.gap_s : null,
    outage: !!(r && r.outage),
    ever: !!(r && r.last_t != null),
  });
  const rt = l.rtkrcv;
  return {
    corr: pick(l.corr),
    obs: pick(l.obs),
    rtkrcv: rt ? { level: rt.level || "ok", message: rt.message || "" } : null,
    diag: (st && st.verdict) || { level: "ok", code: "-", message: "" },
    topics: l.topics || {},
  };
}

export function fmtBps(bps) {
  if (bps == null) return "—";
  if (bps >= 1024) return `${(bps / 1024).toFixed(1)} kB/s`;
  return `${Math.round(bps)} B/s`;
}

export function fmtGap(gap) {
  if (gap == null) return "—";
  return gap < 60 ? `${gap.toFixed(1)} s` : `${Math.floor(gap / 60)} min`;
}

// Compare two statuses and emit only the link/diagnosis transitions worth
// logging. Events themselves are logged separately by the store.
export function linkTransitions(prev, next) {
  const out = [];
  if (!next) return out;
  const a = linkState(prev), b = linkState(next);
  const flow = (name, label, was, now) => {
    if (!now.ever) return;
    if (!was.outage && now.outage) {
      out.push({ kind: "link", level: "warning",
                 text: `${label}中断${now.gap != null ? " " + fmtGap(now.gap) : ""}` });
    } else if (was.outage && !now.outage) {
      out.push({ kind: "link", level: "info", text: `${label}恢复` });
    }
  };
  // Prev may be null on the first status; only report an already-broken link
  // (not a recovery that never happened).
  if (prev == null) {
    if (b.corr.ever && b.corr.outage) out.push({ kind: "link", level: "warning", text: "差分中断" });
    if (b.obs.ever && b.obs.outage) out.push({ kind: "link", level: "warning", text: "观测中断" });
  } else {
    flow("corr", "差分", a.corr, b.corr);
    flow("obs", "观测", a.obs, b.obs);
  }
  const rank = { ok: 0, info: 0, warning: 1, serious: 2, critical: 3, stale: 1, error: 3 };
  const al = a.rtkrcv && a.rtkrcv.level, bl = b.rtkrcv && b.rtkrcv.level;
  if (b.rtkrcv && bl && (rank[bl] || 0) > (rank[al] || 0)) {
    out.push({ kind: "link", level: "warning",
               text: `rtkrcv ${b.rtkrcv.message || bl}` });
  }
  const ad = rank[a.diag.level] || 0, bd = rank[b.diag.level] || 0;
  if (bd > ad && b.diag.message) {
    out.push({ kind: "link", level: b.diag.level, text: b.diag.message });
  }
  return out;
}

export function mountLinkcard(Vue, store) {
  Vue.createApp({
    computed: {
      st() { return linkState(store.status); },
      corrBps() { return fmtBps(this.st.corr.bps); },
      obsBps() { return fmtBps(this.st.obs.bps); },
      corrGap() { return this.st.corr.outage ? fmtGap(this.st.corr.gap) : "正常"; },
      obsGap() { return this.st.obs.outage ? fmtGap(this.st.obs.gap) : "正常"; },
      rtk() { return this.st.rtkrcv; },
      diag() { return this.st.diag; },
      topics() {
        // The backend records the last message wall time per topic, not an
        // age; derive it from the status timestamp.
        const now = (store.status && store.status.t) || 0;
        return Object.entries(this.st.topics).map(([name, ts]) => {
          const age = ts != null && now ? Math.max(0, now - ts) : null;
          const stale = age == null || age > 5;
          return {
            name, state: stale ? "无数据" : fmtGap(age),
            cls: stale ? "none" : "ok",
          };
        });
      },
    },
    template: `
      <div class="linkcard">
        <div class="row" :class="{ warn: st.corr.outage }">
          <span>差分</span><b>{{ corrBps }}</b><span class="sub">{{ corrGap }}</span>
        </div>
        <div class="row" :class="{ warn: st.obs.outage }">
          <span>观测</span><b>{{ obsBps }}</b><span class="sub">{{ obsGap }}</span>
        </div>
        <div class="row" :class="{ warn: rtk && rtk.level !== 'ok' }">
          <span>rtkrcv</span><b>{{ rtk ? rtk.level : '—' }}</b>
          <span class="sub">{{ rtk ? rtk.message : '未配置' }}</span>
        </div>
        <div class="row" :class="'lv-' + (diag.level || 'ok')">
          <span>诊断</span><b>{{ diag.level }}</b>
          <span class="sub">{{ diag.message || diag.code }}</span>
        </div>
        <div class="topics">
          <span v-for="t in topics" :key="t.name" class="topic" :class="t.cls">
            {{ t.name }} {{ t.state }}
          </span>
        </div>
      </div>`,
  }).mount("#linkcard");
}
