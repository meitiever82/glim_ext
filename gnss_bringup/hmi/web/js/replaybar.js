// web/js/replaybar.js
import { SPEED_OPTIONS } from "./protocol.js";

export function mountReplaybar(Vue, store, ws) {
  Vue.createApp({
    data() {
      const now = new Date(), ago = new Date(Date.now() - 3600e3);
      const fmt = (d) => new Date(d - d.getTimezoneOffset() * 60e3).toISOString().slice(0, 16);
      return { t0: fmt(ago), t1: fmt(now), speed: 10, opts: SPEED_OPTIONS };
    },
    computed: { replaying() { return store.replaying; },
                error() { return store.lastError; } },
    methods: {
      start() {
        const a = new Date(this.t0).getTime() / 1000, b = new Date(this.t1).getTime() / 1000;
        if (!(b > a)) return;
        store.lastError = null;
        store.clearForReplay();
        ws.sendReplay(a, b, Number(this.speed));
      },
      live() { ws.sendLive(); store.resumeLive(); },
    },
    template: `
      <div class="replaybar">
        <label>回放 <input type="datetime-local" v-model="t0"></label>
        <label>至 <input type="datetime-local" v-model="t1"></label>
        <select v-model="speed"><option v-for="s in opts" :value="s">{{ s }}×</option></select>
        <button @click="start" :disabled="replaying">开始回放</button>
        <button @click="live" :disabled="!replaying">回到实时</button>
        <span v-if="error" class="err">{{ error }}</span>
      </div>`,
  }).mount("#replaybar");
}
