import asyncio
import json
import socket

from gnss_hmi.diagnosis.rules import Verdict
from gnss_hmi.parsers.rtksol import RtkSolution
from gnss_hmi.publisher import UdpPublisher


async def test_fix_and_event_lines():
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0)); rx.setblocking(False)
    port = rx.getsockname()[1]
    p = UdpPublisher("127.0.0.1", port)
    await p.start()
    sol = RtkSolution(t=1000.0, lat=44.5, lon=90.28, alt=617.0, q=1, ns=38,
                      sdn=0.011, sde=0.012, sdu=0.032, age=0.8, ratio=25.0)
    p.publish_fix(sol, heading=174.2)
    p.publish_event("open", Verdict("serious", "corr_outage", "差分中断 5s"), 1000.5)
    await asyncio.sleep(0.05)
    loop = asyncio.get_running_loop()
    msgs = []
    for _ in range(2):
        data = await asyncio.wait_for(loop.sock_recv(rx, 4096), 1.0)
        msgs.append(json.loads(data.decode().strip()))
    fix = next(m for m in msgs if m["type"] == "gnss_fix")
    ev = next(m for m in msgs if m["type"] == "gnss_event")
    assert fix["ver"] == 1 and fix["q"] == 1 and fix["lat"] == 44.5
    assert fix["sigma_e"] == 0.012 and fix["heading"] == 174.2
    assert fix["source"] == "rtkrcv"
    assert ev["event"] == "corr_outage" and ev["state"] == "open"
    await p.stop(); rx.close()


async def test_publish_without_start_is_noop():
    p = UdpPublisher("127.0.0.1", 9)      # never started
    p.publish_event("open", Verdict("warning", "x", "y"), 1.0)  # must not raise


async def test_host_time_present_on_both_message_types():
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0)); rx.setblocking(False)
    p = UdpPublisher("127.0.0.1", rx.getsockname()[1])
    await p.start()
    sol = RtkSolution(t=1000.0, lat=1, lon=2, alt=3, q=1, ns=30,
                      sdn=.01, sde=.01, sdu=.03, age=.8, ratio=25.0)
    p.publish_fix(sol, heading=None, host_time=1234.5)
    p.publish_event("open", Verdict("serious", "corr_outage", "x"), 1000.5)
    await asyncio.sleep(0.05)
    loop = asyncio.get_running_loop()
    msgs = [json.loads((await asyncio.wait_for(loop.sock_recv(rx, 4096), 1.0)).decode())
            for _ in range(2)]
    fix = next(m for m in msgs if m["type"] == "gnss_fix")
    ev = next(m for m in msgs if m["type"] == "gnss_event")
    assert fix["host_time"] == 1234.5 and fix["gps_time"] == 1000.0
    assert ev["host_time"] == ev["gps_time"] == 1000.5
    await p.stop(); rx.close()
