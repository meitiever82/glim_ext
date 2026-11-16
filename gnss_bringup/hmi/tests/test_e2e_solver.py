# tests/test_e2e_solver.py
import asyncio
import sys
import textwrap
from pathlib import Path

from gnss_hmi.config import load_config
from gnss_hmi.main import build_app

FAKE = Path(__file__).parent / "fake_rtkrcv.py"


async def test_solver_chain_writes_rtkrcv_epochs(tmp_path):
    cfg_file = tmp_path / "config.yaml"
    cfg_file.write_text(textwrap.dedent(f"""\
        data_root: {tmp_path}/log
        db_path: {tmp_path}/rtk.db
        corrections: {{host: 127.0.0.1, port: 1}}
        raw_obs: {{host: 127.0.0.1, port: 1}}
        gnss_solution: {{host: 127.0.0.1, port: 0, listen: true}}
        can_channel: virtual:solvertest
        reserve: {{corrections_port: 0, raw_obs_port: 0}}
        retention_days: 14
        disk_watermark_pct: 85.0
        rtkrcv: {{binary: "{sys.executable}", sol_port: 0, extra_args: ["{FAKE}"]}}
        """))
    app = build_app(load_config(cfg_file))
    task = asyncio.create_task(app.run_forever())
    for _ in range(100):
        await asyncio.sleep(0.1)
        if app.epochs.latest("rtkrcv") is not None:
            break
    task.cancel()
    await app.shutdown()
    e = app.epochs.latest("rtkrcv")
    assert e is not None and e.q == 1 and e.sats == 38
    assert abs(e.lat - 44.501234567) < 1e-9 and abs(e.ratio - 25.0) < 1e-6
