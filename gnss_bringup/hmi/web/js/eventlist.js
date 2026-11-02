// web/js/eventlist.js — event + link-change log window with pause/filter/copy.
import { fmtT } from "./protocol.js";

export function eventReplayWindow(ev) {
  const end = (ev.t_close ?? ev.t + 60);
  return { t0: ev.t - 30, t1: end + 30 };
}

export function logLine(entry) {
  const tag = entry.kind === "event"
    ? (entry.action === "open" ? "事件开始" : "事件结束")
    : "链路";
  return `${fmtT(entry.t)} [${entry.level || "info"}] ${tag} ${entry.text || ""}`.trim();
}

function copyText(text) {
  if (navigator.clipboard && navigator.clipboard.writeText) {
    return navigator.clipboard.writeText(text);
  }
  // Fallback for non-secure origins (plain http on the vehicle LAN).
  const ta = document.createElement("textarea");
  ta.value = text;
  document.body.appendChild(ta);
  ta.select();
  try { document.execCommand("copy"); } finally { document.body.removeChild(ta); }
  return Promise.resolve();
}

export function mountEventlist(Vue, store, ws) {
  Vue.createApp({
    data() {
      return { paused: false, frozen: [], filter: "all", copied: false };
    },
    computed: {
      source() { return this.paused ? this.frozen : store.logs; },
      logs() {
        const f = this.filter;
        return f === "all" ? this.source : this.source.filter((e) => e.kind === f);
      },
    },
    methods: {
      fmtT,
      togglePause() {
        if (!this.paused) this.frozen = store.logs.slice();
        this.paused = !this.paused;
      },
      copy() {
        const text = this.logs.map(logLine).join("\n");
        this.copied = true;
        copyText(text).catch(() => {});
        setTimeout(() => { this.copied = false; }, 1200);
      },
      jump(ev) {
        if (ev.kind !== "event" || !ev.action) return;
        const { t0, t1 } = eventReplayWindow(ev);
        store.clearForReplay();
        ws.sendReplay(t0, t1, 10);
      },
    },
    template: `
      <div class="logctl">
        <button @click="togglePause">{{ paused ? '继续' : '暂停' }}</button>
        <select v-model="filter">
          <option value="all">全部</option>
          <option value="event">事件</option>
          <option value="link">链路</option>
        </select>
        <button @click="copy">{{ copied ? '已复制' : '复制' }}</button>
        <span class="logcount">{{ logs.length }} 条</span>
      </div>
      <ul class="events">
        <li v-for="(e, i) in logs" :key="i" :class="'lv-' + (e.level || 'info')"
            :title="e.kind === 'event' ? '点击回放该时段' : ''"
            @click="jump(e)">
          <span class="et">{{ fmtT(e.t) }}</span>
          <span class="ea">{{ e.kind === 'event' ? (e.action === 'open' ? '▲' : '▼') : '·' }}</span>
          <span class="em">{{ e.text }}</span>
        </li>
        <li v-if="!logs.length" class="empty">暂无日志</li>
      </ul>`,
  }).mount("#eventlist");
}
