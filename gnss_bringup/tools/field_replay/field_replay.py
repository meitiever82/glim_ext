#!/usr/bin/env python3
"""field_replay.py —— 现场数据回放编排器(Task 3)。

把一段实车记录按数据自带时间、1× 实时喂进 GNSS 链路:
- 差分流(`base.rtcm3`)与观测流(`cgi610.dat`)各开一个 127.0.0.1 TCP 监听端口,
  等 `rtcm_bridge`(listen=false)连上后按时间发送**文件原始字节**(按帧边界切块,
  见 `replay_plan.py`);
- CAN 日志到时刻用 `canplayer -I <log> <iface>=<log-iface>` 放到 vcan;
- 以 50 Hz 发布 `/clock = t0 + (wall − W0)·speed`,供 `use_sim_time` 的诊断节点使用;
- 可选 `--nav-rtcm`:在差分流第一块之前注入一遍星历电文,之后每 30 数据秒重发(方案 B)。

时间基准:t0 = min(差分首块, 观测首块, CAN 首帧) 的 unix UTC 秒;两个端口都连上(给了
--wait-fresh-ports 时再等下游 rtkrcv 新连上)时记 W0 = monotonic(),数据时刻 t 的块在
W0 + (t − t0)/speed 发出。

退出码:
  0   正常结束(全部发完、canplayer 退出、/clock 再走 --tail-clock-s 秒)
  2   参数/输入文件错误(文件读不了、没有可切分的帧、找不到 canplayer)
  3   rtcm_bridge 在 --connect-timeout 秒内没有把两个端口都连上;或 --wait-fresh-ports 的
      本机端口在 --downstream-timeout 秒内没有出现新连接
  4   连接断开后 --reconnect-timeout 秒内没有重连
  5   canplayer 启动失败或非正常退出(不是被本程序停掉的)
  130 收到 SIGINT/SIGTERM

注意:canplayer 没有倍速选项,`--speed` ≠ 1 时 CAN 仍按 1× 播放(只适合不带 CAN 的冒烟测试)。
"""
import argparse
import heapq
import os
import select
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from typing import List, Optional

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import candump_log  # noqa: E402
import replay_plan as rp  # noqa: E402
import stream_timing as st  # noqa: E402

EXIT_OK, EXIT_INPUT, EXIT_NO_CONNECT, EXIT_LOST, EXIT_CANPLAYER, EXIT_SIGNAL = 0, 2, 3, 4, 5, 130
NAV_PERIOD_S = 30.0
CLOCK_HZ = 50.0
PROGRESS_PERIOD_S = 10.0


def log(msg: str) -> None:
    print("[field_replay %s] %s" % (time.strftime("%H:%M:%S"), msg), flush=True)


def fmt_t(t: float) -> str:
    return "%s.%03d" % (time.strftime("%H:%M:%S", time.gmtime(t)), int(round((t % 1) * 1000)) % 1000)


class Stopped(Exception):
    """收到 SIGINT/SIGTERM。"""


class StreamPort:
    """一路 TCP:本机监听,rtcm_bridge 作为客户端连进来。"""

    def __init__(self, name: str, port: int, file_bytes: int):
        self.name = name
        self.port = port
        self.file_bytes = file_bytes
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listener.bind(("127.0.0.1", port))
        self.listener.listen(1)
        self.conn: Optional[socket.socket] = None
        self.sent_bytes = 0
        self.sent_data_bytes = 0
        self.sent_nav_bytes = 0
        self.sent_chunks = 0
        self.sent_frames = 0
        self.reconnects = 0

    def state(self) -> str:
        return "up" if self.conn is not None else "DOWN"

    def accept(self, deadline: float, stop: threading.Event) -> bool:
        """等到 deadline(monotonic)为止;连上返回 True。"""
        while not stop.is_set():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            r, _, _ = select.select([self.listener], [], [], min(remaining, 0.2))
            if r:
                conn, addr = self.listener.accept()
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                conn.settimeout(10.0)
                self.conn = conn
                log("%s 端口 %d 已连接(对端 %s:%d)" % (self.name, self.port, addr[0], addr[1]))
                return True
        raise Stopped()

    def peer_closed(self) -> bool:
        """不阻塞地检查对端是否已关闭(可读且读到 0 字节 / 出错)。"""
        if self.conn is None:
            return True
        try:
            r, _, _ = select.select([self.conn], [], [], 0)
            if not r:
                return False
            data = self.conn.recv(4096, socket.MSG_DONTWAIT)
            return data == b""   # 对端发来的字节(rtcm_bridge 不会发)丢弃
        except (BlockingIOError, InterruptedError):
            return False
        except OSError:
            return True

    def drop(self, why: str) -> None:
        log("%s 端口 %d 连接断开: %s" % (self.name, self.port, why))
        if self.conn is not None:
            try:
                self.conn.close()
            except OSError:
                pass
        self.conn = None

    def close(self) -> None:
        for s in (self.conn, self.listener):
            if s is not None:
                try:
                    s.close()
                except OSError:
                    pass
        self.conn = None


def read_established_ports() -> dict:
    counts: dict = {}
    for path in ("/proc/net/tcp", "/proc/net/tcp6"):
        try:
            with open(path) as f:
                text = f.read()
        except OSError:
            continue
        for port, n in rp.established_local_ports(text).items():
            counts[port] = counts.get(port, 0) + n
    return counts


def wait_fresh_ports(ports: List[int], timeout_s: float, stop: threading.Event) -> bool:
    """等 ports 上各出现一次新建立的连接(见 replay_plan.FreshConnectionWaiter)。"""
    waiter = rp.FreshConnectionWaiter(ports)
    start = time.monotonic()
    log("等待本机端口 %s 上出现新建立的连接(rtkrcv 的 tcpcli),最多 %.0f s" % (ports, timeout_s))
    while time.monotonic() - start < timeout_s:
        if stop.is_set():
            raise Stopped()
        if waiter.observe(read_established_ports()):
            log("端口 %s 已新连上,等待 %.1f s" % (ports, time.monotonic() - start))
            return True
        time.sleep(0.05)
    return False


class ClockPublisher:
    """rclpy 节点 field_replay:独立线程 50 Hz 发布 /clock(best effort, depth 1 = ClockQoS)。"""

    def __init__(self, t0: float, speed: float):
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from rclpy.signals import SignalHandlerOptions
        from rosgraph_msgs.msg import Clock
        self._rclpy = rclpy
        self._Clock = Clock
        # 信号由本程序自己处理(要先停 canplayer、关套接字),不让 rclpy 装处理器
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node("field_replay")
        self.pub = self.node.create_publisher(
            Clock, "/clock", QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.t0 = t0
        self.speed = speed
        self.w0: Optional[float] = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="clock", daemon=True)

    def start(self, w0: float) -> None:
        self.w0 = w0
        self._thread.start()

    def now_data(self) -> float:
        return rp.data_time(time.monotonic(), self.t0, self.w0, self.speed)

    def _run(self) -> None:
        period = 1.0 / CLOCK_HZ
        next_tick = time.monotonic()
        while not self._stop.is_set():
            t = self.now_data()
            msg = self._Clock()
            sec = int(t)
            nsec = int(round((t - sec) * 1e9))
            if nsec >= 1000000000:   # 舍入进位:不能写成取模,否则时钟会瞬间倒退 1 s
                sec, nsec = sec + 1, nsec - 1000000000
            msg.clock.sec = sec
            msg.clock.nanosec = nsec
            try:
                self.pub.publish(msg)
            except Exception as e:  # noqa: BLE001  上下文关闭时的竞态,线程随后退出
                if not self._stop.is_set():
                    log("/clock 发布失败: %s" % e)
                return
            next_tick += period
            delay = next_tick - time.monotonic()
            if delay < -1.0:          # 机器卡顿过久:不追帧,重新对齐
                next_tick = time.monotonic()
            elif delay > 0:
                self._stop.wait(delay)

    def shutdown(self) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        try:
            self.node.destroy_node()
            self._rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


class CanPlayer:
    def __init__(self, log_path: str, iface: str, log_iface: str):
        self.cmd = ["canplayer", "-I", log_path, "%s=%s" % (iface, log_iface)]
        self.proc: Optional[subprocess.Popen] = None
        self.stopped_by_us = False
        self.start_wall: Optional[float] = None

    def start(self) -> None:
        log("启动: %s" % " ".join(self.cmd))
        self.start_wall = time.monotonic()
        self.proc = subprocess.Popen(self.cmd, stdin=subprocess.DEVNULL)

    def state(self) -> str:
        if self.proc is None:
            return "pending"
        rc = self.proc.poll()
        return "running" if rc is None else "exit=%d" % rc

    def stop(self) -> None:
        if self.proc is None or self.proc.poll() is not None:
            return
        self.stopped_by_us = True
        self.proc.send_signal(signal.SIGINT)
        try:
            self.proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.wait()


def parse_args(argv: List[str]) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rtcm", required=True, help="差分流文件(RTCM3,如 <seg>/gnss/base.rtcm3)")
    ap.add_argument("--obs", required=True, help="观测流文件(NovAtel 二进制,如 <seg>/raw/cgi610.dat)")
    ap.add_argument("--can-log", help="candump 日志(补过 σ 帧的副本);不给则不放 CAN(仅冒烟测试)")
    ap.add_argument("--nav-rtcm", help="要注入差分流的 RTCM3 星历电文文件(方案 B)")
    ap.add_argument("--corr-port", type=int, default=15031)
    ap.add_argument("--obs-port", type=int, default=15032)
    ap.add_argument("--can-iface", default="vcan0")
    ap.add_argument("--can-log-iface", default="can7")
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--connect-timeout", type=float, default=60.0)
    ap.add_argument("--reconnect-timeout", type=float, default=30.0)
    ap.add_argument("--leap", type=int, default=18, help="GPST−UTC 闰秒")
    ap.add_argument("--wait-fresh-ports", default="",
                    help="逗号分隔的本机端口(如 rtkrcv_node 的 corr_port/obs_port 15041,15042):"
                         "两个上游端口连上后,再等这些端口上各出现一次新建立的连接才开始(W0),"
                         "避开 rtkrcv tcpcli 无数据 10 s 断开、10 s 后重连的空窗,见 replay_plan.py")
    # 25 s < rtcm_bridge 的 idle_timeout_s(30 s):两个上游端口连上后到 W0 之间一个字节都不发,
    # 等太久 bridge 会因空闲主动断开,之后第一次 sendall 可能写进一条对端已关闭的连接
    ap.add_argument("--downstream-timeout", type=float, default=25.0,
                    help="--wait-fresh-ports 最多等多少秒(默认 25,须小于 rtcm_bridge idle_timeout_s=30)")
    ap.add_argument("--duration-s", type=float, default=None, help="只放前 N 数据秒(试跑)")
    ap.add_argument("--tail-clock-s", type=float, default=10.0, help="发完后 /clock 再走的墙钟秒数")
    a = ap.parse_args(argv)
    if not a.speed > 0:
        ap.error("--speed 必须 > 0")
    if a.duration_s is not None and not a.duration_s > 0:
        ap.error("--duration-s 必须 > 0")
    try:
        a.wait_fresh_ports = [int(x) for x in a.wait_fresh_ports.split(",") if x.strip()]
    except ValueError:
        ap.error("--wait-fresh-ports 必须是逗号分隔的端口号")
    return a


def build_plan(a: argparse.Namespace):
    """读文件、切块;返回 (t0, corr_chunks, obs_chunks, can_t0, sizes)。出错抛 ValueError/OSError。"""
    with open(a.obs, "rb") as f:
        obs_bytes = f.read()
    with open(a.rtcm, "rb") as f:
        corr_bytes = f.read()
    obs_spans = rp.novatel_spans(obs_bytes, a.leap)
    if not obs_spans:
        raise ValueError("%s 里没有 CRC 通过的 NovAtel 帧" % a.obs)
    obs_chunks = rp.chunk_stream(obs_bytes, obs_spans)
    corr_chunks = rp.chunk_stream(corr_bytes, rp.rtcm3_spans(corr_bytes, obs_chunks[0].t, a.leap))
    log("观测流 %s: %d 字节, %d 帧/块, 数据 %s – %s UTC"
        % (a.obs, len(obs_bytes), len(obs_chunks), fmt_t(obs_chunks[0].t), fmt_t(obs_chunks[-1].t)))
    log("差分流 %s: %d 字节, %d 帧/块, 数据 %s – %s UTC"
        % (a.rtcm, len(corr_bytes), len(corr_chunks), fmt_t(corr_chunks[0].t), fmt_t(corr_chunks[-1].t)))
    nav_bytes = b""
    if a.nav_rtcm:
        with open(a.nav_rtcm, "rb") as f:
            nav_bytes = f.read()
        nav_frames = list(st.iter_rtcm3_frames(nav_bytes))
        covered = sum(len(fr.raw) for fr in nav_frames)
        if not nav_frames:
            raise ValueError("%s 里没有 RTCM3 帧" % a.nav_rtcm)
        types = {}
        for fr in nav_frames:
            types[fr.msg_type] = types.get(fr.msg_type, 0) + 1
        log("星历 %s: %d 字节, %d 条电文 %s%s" % (
            a.nav_rtcm, len(nav_bytes), len(nav_frames), dict(sorted(types.items())),
            "" if covered == len(nav_bytes) else "(有 %d 字节不属于任何帧,照发)" % (len(nav_bytes) - covered)))
        corr_chunks = rp.inject_periodic(corr_chunks, nav_bytes, NAV_PERIOD_S, len(nav_frames))
    can_t0 = None
    if a.can_log:
        can_t0 = candump_log.first_timestamp(a.can_log)
        log("CAN 日志 %s: 首帧 %s UTC" % (a.can_log, fmt_t(can_t0)))
    t0 = min([corr_chunks[0].t, obs_chunks[0].t] + ([can_t0] if can_t0 is not None else []))
    t_end = None if a.duration_s is None else t0 + a.duration_s
    corr_plan = rp.truncate(corr_chunks, t_end)
    obs_plan = rp.truncate(obs_chunks, t_end)
    sizes = {"corr_file": len(corr_bytes), "obs_file": len(obs_bytes), "nav": len(nav_bytes),
             "nav_injections_planned": sum(1 for c in corr_plan if c.kind == "nav"),
             "corr_plan_bytes": sum(len(c.data) for c in corr_plan),
             "obs_plan_bytes": sum(len(c.data) for c in obs_plan)}
    return t0, t_end, corr_plan, obs_plan, can_t0, sizes


def main(argv: List[str]) -> int:
    code = run(parse_args(argv))
    log("退出码 %d" % code)
    return code


def run(a: argparse.Namespace) -> int:
    stop = threading.Event()

    def on_signal(signum, _frame):
        if not stop.is_set():
            log("收到信号 %d,停止回放" % signum)
        stop.set()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    try:
        t0, t_end, corr_plan, obs_plan, can_t0, sizes = build_plan(a)
    except (OSError, ValueError) as e:
        log("输入错误: %s" % e)
        return EXIT_INPUT
    if a.can_log and shutil.which("canplayer") is None:
        log("找不到 canplayer(apt 包 can-utils)")
        return EXIT_INPUT
    if a.can_log and a.speed != 1.0:
        log("警告: canplayer 不支持倍速,CAN 仍按 1× 播放,与 --speed %.3g 的另两路不同步" % a.speed)
    log("t0 = %.3f (%s UTC)%s, speed = %.3g" % (
        t0, fmt_t(t0), "" if t_end is None else ", 只放 %.1f 数据秒(到 %s)" % (a.duration_s, fmt_t(t_end)),
        a.speed))
    log("计划发送: 差分 %d 块 %d 字节(文件 %d,星历注入 %d 次 × %d 字节), 观测 %d 块 %d 字节(文件 %d)" % (
        len(corr_plan), sizes["corr_plan_bytes"], sizes["corr_file"], sizes["nav_injections_planned"],
        sizes["nav"], len(obs_plan), sizes["obs_plan_bytes"], sizes["obs_file"]))

    ports: List[StreamPort] = []
    canplayer: Optional[CanPlayer] = None
    clock: Optional[ClockPublisher] = None
    code = EXIT_OK
    try:
        try:
            ports = [StreamPort("差分", a.corr_port, sizes["corr_file"]),
                     StreamPort("观测", a.obs_port, sizes["obs_file"])]
        except OSError as e:
            log("无法监听端口: %s" % e)
            return EXIT_INPUT
        log("监听 127.0.0.1:%d(差分)与 127.0.0.1:%d(观测),等待 rtcm_bridge 连接(最多 %.0f s)"
            % (a.corr_port, a.obs_port, a.connect_timeout))
        deadline = time.monotonic() + a.connect_timeout
        for p in ports:
            if not p.accept(deadline, stop):
                log("rtcm_bridge 未连接(%s 端口 %d 在 %.0f s 内无人连接)" % (p.name, p.port, a.connect_timeout))
                return EXIT_NO_CONNECT

        if a.wait_fresh_ports:
            if not wait_fresh_ports(a.wait_fresh_ports, a.downstream_timeout, stop):
                log("下游未连接(本机端口 %s 在 %.0f s 内没有出现新连接;rtkrcv_node/rtkrcv 没起来?)"
                    % (a.wait_fresh_ports, a.downstream_timeout))
                return EXIT_NO_CONNECT

        clock = ClockPublisher(t0, a.speed)
        w0 = time.monotonic()
        clock.start(w0)
        log("两个端口都已连接,开始回放 (W0)")

        if a.can_log:
            canplayer = CanPlayer(a.can_log, a.can_iface, a.can_log_iface)
        can_due = None if canplayer is None else rp.wall_due(can_t0, t0, w0, a.speed)
        can_stop_due = None
        if canplayer is not None and t_end is not None:
            can_stop_due = can_due + (t_end - can_t0)  # canplayer 始终 1×,不随 --speed
        events = heapq.merge(((c.t, 0, i, c) for i, c in enumerate(corr_plan)),
                             ((c.t, 1, i, c) for i, c in enumerate(obs_plan)))
        next_progress = w0 + PROGRESS_PERIOD_S

        def progress(tag: str = "进度") -> None:
            log("%s: 数据时刻 %s(+%.1f s)| 差分 %s %d 块/%d 帧/%d 字节 | 观测 %s %d 块/%d 帧/%d 字节 | canplayer %s" % (
                tag, fmt_t(clock.now_data()), clock.now_data() - t0,
                ports[0].state(), ports[0].sent_chunks, ports[0].sent_frames, ports[0].sent_bytes,
                ports[1].state(), ports[1].sent_chunks, ports[1].sent_frames, ports[1].sent_bytes,
                "-" if canplayer is None else canplayer.state()))

        def service(now: float) -> None:
            """等待期间要做的杂事:按时起停 canplayer、打进度、检查对端是否断开。"""
            nonlocal next_progress
            if stop.is_set():
                raise Stopped()
            if canplayer is not None and canplayer.proc is None and now >= can_due:
                try:
                    canplayer.start()
                except OSError as e:
                    raise CanplayerFailed("canplayer 启动失败: %s" % e)
            if canplayer is not None and can_stop_due is not None and now >= can_stop_due \
                    and canplayer.proc is not None and canplayer.proc.poll() is None:
                log("到达 --duration-s,停止 canplayer")
                canplayer.stop()
            if now >= next_progress:
                next_progress += PROGRESS_PERIOD_S
                for p in ports:
                    if p.conn is not None and p.peer_closed():
                        p.drop("对端关闭")
                progress()

        def wait_until(due: float) -> None:
            while True:
                now = time.monotonic()
                service(now)
                if now >= due:
                    return
                time.sleep(min(due - now, 0.05))

        def send(p: StreamPort, chunk: rp.Chunk) -> None:
            while True:
                if p.conn is None:
                    log("%s 端口 %d 等待重连(最多 %.0f s)" % (p.name, p.port, a.reconnect_timeout))
                    if not p.accept(time.monotonic() + a.reconnect_timeout, stop):
                        raise ConnectionLost("%s 端口 %d 断开后 %.0f s 内未重连"
                                             % (p.name, p.port, a.reconnect_timeout))
                    p.reconnects += 1
                try:
                    p.conn.sendall(chunk.data)
                    break
                except OSError as e:
                    p.drop(str(e))   # 这一块重发给新连接(块边界 = 帧边界,不会半帧)
            p.sent_bytes += len(chunk.data)
            if chunk.kind == "nav":
                p.sent_nav_bytes += len(chunk.data)
            else:
                p.sent_data_bytes += len(chunk.data)
            p.sent_chunks += 1
            p.sent_frames += chunk.frames

        for t, idx, _, chunk in events:
            wait_until(rp.wall_due(t, t0, w0, a.speed))
            send(ports[idx], chunk)
        if canplayer is not None and canplayer.proc is None:
            wait_until(can_due)   # CAN 首帧比两路 TCP 的最后一块还晚(不会发生在真实数据上)
        log("两路 TCP 全部发完")
        progress("发完")

        if canplayer is not None:
            while canplayer.proc.poll() is None:
                wait_until(time.monotonic() + 0.2)
            rc = canplayer.proc.returncode
            if rc != 0 and not canplayer.stopped_by_us:
                log("canplayer 非正常退出 rc=%d" % rc)
                code = EXIT_CANPLAYER
            else:
                log("canplayer 已退出 rc=%d%s,播放 %.1f s" % (
                    rc, "(被 --duration-s 停止)" if canplayer.stopped_by_us else "",
                    time.monotonic() - canplayer.start_wall))

        log("/clock 再走 %.0f s" % a.tail_clock_s)
        tail_end = time.monotonic() + a.tail_clock_s
        wait_until(tail_end)
    except Stopped:
        code = EXIT_SIGNAL
    except ConnectionLost as e:
        log(str(e))
        code = EXIT_LOST
    except CanplayerFailed as e:
        log(str(e))
        code = EXIT_CANPLAYER
    finally:
        if canplayer is not None:
            canplayer.stop()
        for p in ports:
            p.close()
        if clock is not None:
            clock.shutdown()
        if clock is not None:   # 开始回放之后才有字节核对的意义
            summarize(ports, sizes, t_end is not None)
    return code


class ConnectionLost(Exception):
    pass


class CanplayerFailed(Exception):
    pass


def summarize(ports: List[StreamPort], sizes: dict, truncated: bool) -> None:
    """结尾字节核对:整段回放时 数据字节 = 文件大小,总字节 = 文件大小 + 注入星历字节。"""
    corr, obs = ports
    for p, file_key, plan_key in ((corr, "corr_file", "corr_plan_bytes"), (obs, "obs_file", "obs_plan_bytes")):
        expect = sizes[plan_key]
        if truncated:
            verdict = "按 --duration-s 截断,计划 %d 字节" % expect
        else:
            verdict = "文件原始字节全部发出" if p.sent_data_bytes == sizes[file_key] else "与文件大小不符!"
        extra = ""
        if p is corr and p.sent_nav_bytes:
            extra = " + 注入星历 %d 字节(%d 次)" % (p.sent_nav_bytes, p.sent_nav_bytes // max(sizes["nav"], 1))
        log("字节核对 %s: 已发 %d 字节 = 数据 %d%s;文件 %d 字节;%s;%s" % (
            p.name, p.sent_bytes, p.sent_data_bytes, extra, sizes[file_key], verdict,
            "计划内全部发出" if p.sent_bytes == expect else "计划 %d 字节,少发 %d" % (expect, expect - p.sent_bytes)))


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
