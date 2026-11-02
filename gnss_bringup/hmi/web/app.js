// web/app.js — SPA bootstrap. Served by the browser (never node-tested), so it
// uses absolute /js/... and /vendor/... paths matching the StaticFiles mount.
import * as Vue from "/vendor/vue.esm-browser.prod.js";
import { createStore } from "/js/store.js";
import { WsClient } from "/js/ws.js";
import { mountStatusbar } from "/js/statusbar.js";
import { mountEventlist } from "/js/eventlist.js";
import { mountLinkcard } from "/js/linkcard.js";
import { mountReplaybar } from "/js/replaybar.js";
import { mountExportbar } from "/js/exportbar.js";
import { MapView } from "/js/mapview.js";
import { Timelines } from "/js/timeline.js";
import { Skyplot } from "/js/skyplot.js";

const store = createStore(Vue);
const proto = location.protocol === "https:" ? "wss" : "ws";
const ws = new WsClient(`${proto}://${location.host}/ws`, store.applyMessage,
                        (up) => { store.connected = up;
                                  document.getElementById("banner").classList.toggle("hidden", up); });
ws.connect();

mountStatusbar(Vue, store);
mountLinkcard(Vue, store);
mountEventlist(Vue, store, ws);
mountReplaybar(Vue, store, ws);
mountExportbar(Vue);
const map = new MapView("map", store);
const tl = new Timelines(store);
const sky = new Skyplot("skyplot", store);
setInterval(() => { map.render(); tl.render(); sky.render(); }, 1000);
