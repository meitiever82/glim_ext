import asyncio
import textwrap

import can

from gnss_hmi.config import load_config
from gnss_hmi.main import build_app
from gnss_hmi.parsers.rtcm import crc24q


def _rtcm_frame(msg_type: int) -> bytes:
    payload = (msg_type << 4).to_bytes(2, "big") + b"\x00" * 6
    head = bytes([0xD3, 0x00, len(payload)])
    return head + payload + crc24q(head + payload).to_bytes(3, "big")


async def _stream_server(payload: bytes, interval: float):
    async def handler(reader, writer):
        try:
            while True:
                writer.write(payload)
                await writer.drain()
                await asyncio.sleep(interval)
        except (ConnectionResetError, BrokenPipeError):
            pass
    server = await asyncio.start_server(handler, "127.0.0.1", 0)
    return server, server.sockets[0].getsockname()[1]


async def test_end_to_end_collect_and_log(tmp_path):
    corr_srv, corr_port = await _stream_server(_rtcm_frame(1074), 0.02)
    obs_srv, obs_port = await _stream_server(_rtcm_frame(1077), 0.02)
    cfg_file = tmp_path / "config.yaml"
    cfg_file.write_text(textwrap.dedent(f"""\
        data_root: {tmp_path}/log
        db_path: {tmp_path}/rtk.db
        corrections: {{host: 127.0.0.1, port: {corr_port}}}
        raw_obs: {{host: 127.0.0.1, port: {obs_port}}}
        gnss_solution: {{host: 127.0.0.1, port: 0, listen: true}}
        can_channel: virtual:e2e
        reserve: {{corrections_port: 0, raw_obs_port: 0}}
        retention_days: 14
        disk_watermark_pct: 85.0
        """))
    app = build_app(load_config(cfg_file))
    task = asyncio.create_task(app.run_forever())
    await asyncio.sleep(0.05)
    with can.Bus(interface="virtual", channel="e2e") as tx:
        tx.send(can.Message(arbitration_id=0x320,
                            data=bytes.fromhex("4409a03c3c060000"), is_extended_id=False))
        await asyncio.sleep(0.5)
    task.cancel()
    await app.shutdown()

    day = next((tmp_path / "log").iterdir())
    assert next(day.glob("corr_*.rtcm3")).stat().st_size > 0
    assert next(day.glob("obs_*.rtcm3")).stat().st_size > 0
    assert "320#" in next(day.glob("can*_*.log")).read_text()
    idx_line = next(day.glob("corr_*.idx.jsonl")).read_text().splitlines()[0]
    assert '"type": 1074' in idx_line
    states = [(e.etype, e.state) for e in app.events.query()]
    assert ("corrections_link", "connected") in states
    corr_srv.close(); obs_srv.close()
