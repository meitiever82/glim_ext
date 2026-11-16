// web/js/eventlist.js
import { fmtT } from "./protocol.js";

export function eventReplayWindow(ev) {
  const end = (ev.t_close ?? ev.t + 60);
  return { t0: ev.t - 30, t1: end + 30 };
}

export function mountEventlist(Vue, store, ws) {
  Vue.createApp({
    computed: { events() { return store.events; } },
    methods: {
      fmtT,
      jump(ev) {
        const { t0, t1 } = eventReplayWindow(ev);
        store.clearForReplay();
        ws.sendReplay(t0, t1, 10);
      },
    },
    template: `
      <ul class="events">
        <li v-for="e in events" :class="'lv-' + (e.level || 'info')" @click="jump(e)">
          <span class="et">{{ fmtT(e.t) }}</span>
          <span class="ea">{{ e.action === 'open' ? '▲' : '▼' }}</span>
          <span class="ec" v-if="e.code">{{ e.code }}</span>
          <span class="em">{{ e.message }}</span>
        </li>
        <li v-if="!events.length" class="empty">暂无事件</li>
      </ul>`,
  }).mount("#eventlist");
}
