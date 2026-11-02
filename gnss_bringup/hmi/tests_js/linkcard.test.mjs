import test from "node:test";
import assert from "node:assert/strict";
import { linkState, linkTransitions, fmtBps, fmtGap }
  from "../web/js/linkcard.js";
import { logLine } from "../web/js/eventlist.js";

const rate = (over = {}) => ({ rate_bps: 120, gap_s: 0.2, last_t: 100,
                               outage: false, ...over });
const st = (corr, obs, verdict = { level: "ok", code: "-", message: "" }, rtk = null) => ({
  t: 200, link: { corr, obs, rtkrcv: rtk, topics: {} }, verdict,
});

test("linkState normalises rates, rtkrcv and verdict", () => {
  const a = linkState(st(rate(), rate()));
  assert.equal(a.corr.bps, 120);
  assert.equal(a.corr.outage, false);
  assert.equal(a.corr.ever, true);
  assert.equal(a.obs.bps, 120);
  assert.equal(a.rtkrcv, null);
  assert.equal(a.diag.level, "ok");
  const b = linkState(st(null, null, { level: "critical", code: "x", message: "差分中断" },
                          { level: "error", message: "solver stopped" }));
  assert.equal(b.corr.bps, null);
  assert.equal(b.corr.ever, false);
  assert.equal(b.rtkrcv.level, "error");
  assert.equal(b.diag.message, "差分中断");
  const c = linkState(null);
  assert.equal(c.corr.outage, false);
  assert.equal(c.diag.level, "ok");
});

test("linkTransitions: outage appears once, recovery closes it", () => {
  const bad = rate({ outage: true, gap_s: 5 });
  const one = linkTransitions(st(rate(), rate()), st(bad, rate()));
  assert.deepEqual(one.map((e) => e.text), ["差分中断 5.0 s"]);
  assert.equal(one[0].level, "warning");
  // unchanged outage must not repeat
  assert.deepEqual(linkTransitions(st(bad, rate()), st(bad, rate())), []);
  const back = linkTransitions(st(bad, rate()), st(rate(), rate()));
  assert.deepEqual(back.map((e) => [e.level, e.text]), [["info", "差分恢复"]]);
});

test("linkTransitions: first status only reports an already-broken link", () => {
  const bad = rate({ outage: true, gap_s: 8 });
  const first = linkTransitions(null, st(bad, bad));
  assert.deepEqual(first.map((e) => e.text), ["差分中断", "观测中断"]);
  assert.deepEqual(linkTransitions(null, st(rate(), rate())), []);
});

test("linkTransitions: rtkrcv and diagnosis escalation", () => {
  const esc = linkTransitions(
    st(rate(), rate(), { level: "ok", code: "-", message: "" }, { level: "ok", message: "" }),
    st(rate(), rate(), { level: "critical", code: "no_data", message: "无数据——检查采集链路" },
       { level: "error", message: "rtkrcv exited" }));
  assert.equal(esc.length, 2);
  assert.ok(esc.some((e) => e.text.includes("rtkrcv exited")));
  assert.ok(esc.some((e) => e.text.includes("无数据")));
  // de-escalation is not logged
  assert.deepEqual(linkTransitions(
    st(rate(), rate(), { level: "critical", message: "x" }), st(rate(), rate())), []);
});

test("link card formatters", () => {
  assert.equal(fmtBps(null), "—");
  assert.equal(fmtBps(100), "100 B/s");
  assert.equal(fmtBps(2048), "2.0 kB/s");
  assert.equal(fmtGap(null), "—");
  assert.equal(fmtGap(5), "5.0 s");
  assert.equal(fmtGap(120), "2 min");
});

test("logLine renders event and link rows", () => {
  const t = 1000;
  assert.match(logLine({ kind: "event", action: "open", level: "warning",
                         text: "差分中断", t }), /\[warning\] 事件开始 差分中断$/);
  assert.match(logLine({ kind: "link", level: "info", text: "差分恢复", t }),
               /\[info\] 链路 差分恢复$/);
});
