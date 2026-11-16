import asyncio
from gnss_hmi.broadcast import Broadcaster


async def test_fanout_and_slow_subscriber_drops_oldest():
    b = Broadcaster(maxsize=2)
    q1, q2 = b.subscribe(), b.subscribe()
    for i in range(4):
        b.publish({"n": i})
    assert q1.qsize() == 2 and (await q1.get())["n"] == 2   # oldest dropped
    assert (await q2.get())["n"] == 2
    b.unsubscribe(q1)
    b.publish({"n": 9})
    assert q1.qsize() == 1                                   # no longer fed
    assert (await q2.get())["n"] == 3 and (await q2.get())["n"] == 9


async def test_publish_with_no_subscribers_is_noop():
    Broadcaster().publish({"x": 1})
