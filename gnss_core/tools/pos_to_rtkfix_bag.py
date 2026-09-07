#!/usr/bin/env python3
"""pos_to_rtkfix_bag.py —— 标准 .pos → gnss_msgs/RtkFix rosbag2(spec §12.3,Task 13 Step 5)

**必须在 Orin(ROS 2 + rosbag2_py + gnss_msgs)上跑;本脚本在无 ROS 的云端环境编写,未在本环境运行过**,
现场首次使用请先用一个短 .pos 试跑并 `ros2 bag info` 核对消息数与 topic。

usage:
  pos_to_rtkfix_bag.py <in.pos> --out <bag_dir> [--topic /gnss_cgi610/rtk_fix] [--frame gnss]
                       [--merge <lidar_bag_dir>] [--pos-time UTC|GPST] [--leap 18]
                       [--header-delay 0.05] [--storage sqlite3|mcap]

字段映射(PLAN Task 13 Step 5):
  header.stamp = t + header_delay(默认 0.05,模拟接收延迟,让 stamp_source 的差别可见)
  gnss_time    = t
  quality      = q_to_quality(Q)   1→FIXED(4) 2→FLOAT(3) 4→DGPS(2) 5→SINGLE(1) 其它→NONE(0)
  latitude/longitude/altitude = lat/lon/height   (字段名与 gnss_msgs/RtkFix.msg 一致)
  sigma_enu    = [sde, sdn, sdu]   (.pos 列顺序是 sdn sde sdu,注意换序)
  diff_age     = age
  sats_used    = ns
  heading      = 0, heading_valid = False

--merge:把 LiDAR/IMU bag 的全部消息与合成 RtkFix 按时间戳归并写进同一个 bag,避免双 bag 回放时钟对齐问题。
"""
import argparse
import heapq
import os
import sys
from datetime import datetime, timezone

# ---- .pos 解析(与 gnss_core::read_pos 同语义:GPST 减闰秒,UTC 原样,无头按 --pos-time) ----

def parse_pos(path, default_time="GPST", leap=18):
    ts = default_time
    recs = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("%"):
                if "time=GPST" in line:
                    ts = "GPST"
                elif "time=UTC" in line:
                    ts = "UTC"
                else:
                    # RTKLIB 列名行 "%  GPST  latitude(deg) ..." / "%  UTC ...":第一个 token 即时间系统
                    tok = line[1:].split()
                    if tok and tok[0] in ("GPST", "UTC"):
                        ts = tok[0]
                continue
            c = line.split()
            if len(c) < 10:
                continue
            try:
                dt = datetime.strptime(c[0] + " " + c[1][:12], "%Y/%m/%d %H:%M:%S.%f").replace(tzinfo=timezone.utc)
            except ValueError:
                continue
            t = dt.timestamp()
            if ts == "GPST":
                t -= leap
            r = {
                "t": t,
                "lat": float(c[2]), "lon": float(c[3]), "h": float(c[4]),
                "q": int(c[5]), "ns": int(c[6]),
                "sdn": float(c[7]), "sde": float(c[8]), "sdu": float(c[9]),
                "age": float(c[13]) if len(c) > 13 else 0.0,
            }
            recs.append(r)
    return recs


Q_TO_QUALITY = {1: 4, 2: 3, 4: 2, 5: 1}   # RTKLIB Q → gnss_msgs 归一化质量(spec §4.1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("pos")
    ap.add_argument("--out", required=True, help="输出 bag 目录(不得已存在)")
    ap.add_argument("--topic", default="/gnss_cgi610/rtk_fix")
    ap.add_argument("--frame", default="gnss")
    ap.add_argument("--merge", default=None, help="要合并的 LiDAR/IMU bag 目录")
    ap.add_argument("--pos-time", default="GPST", choices=["GPST", "UTC"], help="无头部标注时的时间系统")
    ap.add_argument("--leap", type=int, default=18)
    ap.add_argument("--header-delay", type=float, default=0.05)
    ap.add_argument("--storage", default="sqlite3", choices=["sqlite3", "mcap"])
    args = ap.parse_args()

    recs = parse_pos(args.pos, args.pos_time, args.leap)
    if not recs:
        print("no epochs parsed from", args.pos, file=sys.stderr)
        return 1
    if os.path.exists(args.out):
        print("output exists:", args.out, file=sys.stderr)
        return 1

    # ---- ROS 2 依赖:仅在 Orin 上可用 ----
    try:
        import rosbag2_py
        from rclpy.serialization import serialize_message
        from builtin_interfaces.msg import Time
        from gnss_msgs.msg import RtkFix
    except ImportError as e:
        print("需要 ROS 2 环境(rosbag2_py / rclpy / gnss_msgs):", e, file=sys.stderr)
        print("请先 source ~/driver_ws/install/setup.bash", file=sys.stderr)
        return 1

    def to_time(sec):
        s = int(sec)
        ns = int(round((sec - s) * 1e9))
        if ns >= 1_000_000_000:
            s += 1
            ns -= 1_000_000_000
        return Time(sec=s, nanosec=ns)

    def make_msg(r):
        m = RtkFix()
        m.header.stamp = to_time(r["t"] + args.header_delay)
        m.header.frame_id = args.frame
        m.gnss_time = float(r["t"])
        m.quality = Q_TO_QUALITY.get(r["q"], 0)
        m.latitude = r["lat"]
        m.longitude = r["lon"]
        m.altitude = r["h"]
        m.sigma_enu = [r["sde"], r["sdn"], r["sdu"]]
        m.diff_age = r["age"]
        m.sats_used = r["ns"]
        m.heading = 0.0
        m.heading_valid = False
        return m

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=args.out, storage_id=args.storage),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    # Jazzy 的 TopicMetadata 需要 id 位置参数(Humble 不需要);两者都接受 keyword 形式。
    writer.create_topic(rosbag2_py.TopicMetadata(
        id=0, name=args.topic, type="gnss_msgs/msg/RtkFix", serialization_format="cdr",
        offered_qos_profiles=""))

    # 合成 RtkFix 序列(bag 接收时间 = header.stamp;元组第 2 项用于同时刻排序:LiDAR 侧先写)
    def rtk_iter():
        for r in sorted(recs, key=lambda x: x["t"]):
            t_ns = int(round((r["t"] + args.header_delay) * 1e9))
            yield (t_ns, 1, args.topic, serialize_message(make_msg(r)))

    n_rtk = 0
    n_merge = 0
    if args.merge is None:
        for t_ns, _, topic, data in rtk_iter():
            writer.write(topic, data, t_ns)
            n_rtk += 1
    else:
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=args.merge, storage_id=""),   # 空 storage_id:自动识别
            rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
        )
        for i, tm in enumerate(reader.get_all_topics_and_types(), start=1):
            if tm.name == args.topic:
                print(f"warning: source bag already has {args.topic}; its messages are dropped", file=sys.stderr)
                continue
            writer.create_topic(rosbag2_py.TopicMetadata(
                id=i, name=tm.name, type=tm.type, serialization_format=tm.serialization_format,
                offered_qos_profiles=getattr(tm, "offered_qos_profiles", "")))   # 保留源 QoS

        def lidar_iter():
            # SequentialReader 按接收时间升序输出
            while reader.has_next():
                topic, data, t_ns = reader.read_next()
                if topic == args.topic:
                    continue
                yield (t_ns, 0, topic, data)

        # 两路均按时间升序 → heapq.merge 线性归并,不必整包读入内存
        for t_ns, src, topic, data in heapq.merge(lidar_iter(), rtk_iter(), key=lambda x: (x[0], x[1])):
            writer.write(topic, data, t_ns)
            if src == 1:
                n_rtk += 1
            else:
                n_merge += 1

    if hasattr(writer, "close"):
        writer.close()   # Jazzy 提供显式 close;老版本靠析构 flush
    del writer
    print(f"wrote {n_rtk} RtkFix on {args.topic}" + (f" + {n_merge} merged messages" if args.merge else "")
          + f" -> {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
