// web/js/linkcard.js — 链路卡片：差分/观测速率、中断、rtkrcv 健康与最严重诊断。
import { fmtBps } from "./protocol.js";

const LEVEL_CLASS = { ok: "ok", warning: "warn", warn: "warn", error: "bad",
                      serious: "bad", critical: "bad", stale: "warn" };

export function linkRows(status) {
  const st = status || {};
  const corr = st.corr_rate || {};
  const rtk = (st.diagnostics || {}).rtkrcv || {};
  const v = st.verdict || {};
  const idle = (stale) => (stale ? "中断" : "正常");
  const rows = [
    { name: "差分", text: fmtBps(corr.rtcm_bps), note: idle(corr.rtcm_stale),
      cls: corr.rtcm_stale ? "bad" : "ok" },
    { name: "观测", text: fmtBps(corr.obs_bps), note: idle(corr.obs_stale),
      cls: corr.obs_stale ? "bad" : "ok" },
    { name: "rtkrcv", text: rtk.level_name || "—", note: rtk.message || "",
      cls: LEVEL_CLASS[(rtk.level_name || "").toLowerCase()] || "ok" },
    { name: "诊断", text: v.level || "ok", note: v.message || "",
      cls: LEVEL_CLASS[v.level] || "ok" },
  ];
  for (const link of st.links || []) {
    if (link.state === "ok") continue;
    rows.push({
      name: link.topic,
      text: link.state === "waiting" ? "等待" : "无消息",
      note: link.state === "waiting" ? "启动宽限" : "检查 ros2 topic info -v 的 QoS",
      cls: link.state === "waiting" ? "warn" : "bad",
    });
  }
  return rows;
}

export function mountLinkcard(Vue, store) {
  Vue.createApp({
    computed: {
      rows() { return linkRows(store.status); },
    },
    template: `
      <ul class="links">
        <li v-for="r in rows" :key="r.name" :class="'lv-' + r.cls">
          <span class="ln">{{ r.name }}</span>
          <span class="lv">{{ r.text }}</span>
          <span class="lnote" v-if="r.note">{{ r.note }}</span>
        </li>
      </ul>`,
  }).mount("#linkcard");
}
