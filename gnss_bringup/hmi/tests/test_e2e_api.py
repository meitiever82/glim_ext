# tests/test_e2e_api.py — full App with web enabled; raw websockets client
import asyncio
import json
import sys
import textwrap
from pathlib import Path

import websockets

from gnss_hmi.config import load_config
from gnss_hmi.main import build_app

FAKE = Path(__file__).parent / "fake_rtkrcv.py"


async def test_api_and_ws_live(tmp_path):
    cfg_file = tmp_path / "config.yaml"
    cfg_file.write_text(textwrap.dedent(f"""\
        data_root: {tmp_path}/log
        db_path: {tmp_path}/rtk.db
        corrections: {{host: 127.0.0.1, port: 1}}
        raw_obs: {{host: 127.0.0.1, port: 1}}
        gnss_solution: {{host: 127.0.0.1, port: 0, listen: true}}
        can_channel: virtual:apie2e
        reserve: {{corrections_port: 0, raw_obs_port: 0}}
        retention_days: 14
        disk_watermark_pct: 85.0
        rtkrcv: {{binary: "{sys.executable}", sol_port: 0, extra_args: ["{FAKE}"]}}
        web: {{port: 0}}
        diagnosis: {{corr_gap_s: 0.5}}
        """))
    app = build_app(load_config(cfg_file))
    task = asyncio.create_task(app.run_forever())
    for _ in range(100):
        await asyncio.sleep(0.1)
        # Wait not just for a diagnosis tick to have run, but for one where
        # the fake rtkrcv's solution has already been picked up -- the first
        # tick (at t=~1s) can fire before the subprocess has spawned and
        # connected, in which case last_status is truthy but st["sol"] is
        # still None.
        if app.web_port() and app.last_status and app.last_status.get("sol"):
            break
    port = app.web_port()
    assert port

    import httpx
    async with httpx.AsyncClient() as c:
        st = (await c.get(f"http://127.0.0.1:{port}/api/status")).json()
        assert st["type"] == "status" and st["sol"]["q"] == 1
        eps = (await c.get(f"http://127.0.0.1:{port}/api/epochs",
                           params={"src": "rtkrcv", "t0": 0, "t1": 9e9})).json()
        assert eps and eps[-1]["sats"] == 38

    async with websockets.connect(f"ws://127.0.0.1:{port}/ws") as ws:
        for _ in range(20):
            m = json.loads(await asyncio.wait_for(ws.recv(), 3.0))
            if m["type"] == "status":
                assert m["verdict"]["code"] in ("corr_outage", "no_solution", "rtk_fixed",
                                                "not_fixed", "no_data")
                break
        else:
            raise AssertionError("no status message on live WS")
        await ws.send(json.dumps({"cmd": "replay", "t0": 0, "t1": 1, "speed": 1000}))
        while True:
            m = json.loads(await asyncio.wait_for(ws.recv(), 3.0))
            if m["type"] == "replay_end":
                break
    task.cancel()
    await app.shutdown()
