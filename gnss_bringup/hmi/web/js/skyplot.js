// web/js/skyplot.js
export function skyXY(azDeg, elDeg, r) {
  const rho = r * (90 - elDeg) / 90;
  const a = (azDeg - 90) * Math.PI / 180;   // az 0 = north = up
  return { x: r + rho * Math.cos(a), y: r + rho * Math.sin(a) };
}

export class Skyplot {
  constructor(canvasId, store) {
    this.el = document.getElementById(canvasId);
    this.store = store;
  }
  render() {
    const g = this.el.getContext("2d"), W = this.el.width, r = W / 2;
    g.clearRect(0, 0, W, W);
    g.strokeStyle = "#2a3340";
    for (const f of [1, 2 / 3, 1 / 3]) {
      g.beginPath(); g.arc(r, r, r * f - 1, 0, 7); g.stroke();
    }
    // Top-level status.sky is the single source for the $SAT stream; fall back
    // to the legacy sol.sats_json only for older status payloads.
    const st = this.store.status || {};
    const raw = st.sky || (st.sol && st.sol.sats_json);
    let sats = null;
    try { sats = raw ? JSON.parse(raw) : null; } catch { /* keep null */ }
    if (!sats || !sats.length) {
      g.fillStyle = "#5a6472"; g.font = "13px sans-serif"; g.textAlign = "center";
      g.fillText("等待 $SAT 数据", r, r - 6);
      g.fillText("（真机接通 stat 流后显示）", r, r + 12);
      return;
    }
    for (const s of sats) {
      const { x, y } = skyXY(s.az, s.el, r);
      g.beginPath(); g.arc(x, y, 5, 0, 7);
      g.fillStyle = s.snr > 40 ? "#3fb96c" : s.snr >= 35 ? "#e0b23c" : "#e05c4f";
      if (s.used) g.fill(); else { g.strokeStyle = g.fillStyle; g.stroke(); }
      g.fillStyle = "#8b95a3"; g.font = "9px sans-serif"; g.fillText(s.sat, x + 6, y + 3);
    }
  }
}
