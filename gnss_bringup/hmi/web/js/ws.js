// web/js/ws.js
export class WsClient {
  constructor(url, onMessage, onState) {
    this.url = url; this.onMessage = onMessage; this.onState = onState;
    this.backoff = 1000; this.ws = null; this.closed = false;
  }
  connect() {
    this.ws = new WebSocket(this.url);
    this.ws.onopen = () => { this.backoff = 1000; this.onState(true); };
    this.ws.onmessage = (ev) => this.onMessage(JSON.parse(ev.data));
    this.ws.onclose = () => {
      this.onState(false);
      if (this.closed) return;
      setTimeout(() => this.connect(), this.backoff);
      this.backoff = Math.min(this.backoff * 2, 5000);
    };
    this.ws.onerror = () => this.ws.close();
  }
  _send(obj) { if (this.ws && this.ws.readyState === 1) this.ws.send(JSON.stringify(obj)); }
  sendReplay(t0, t1, speed) { this._send({ cmd: "replay", t0, t1, speed }); }
  sendLive() { this._send({ cmd: "live" }); }
  close() { this.closed = true; if (this.ws) this.ws.close(); }
}
