#!/usr/bin/env python3
"""pos_to_rtkfix_bag.py —— 标准 .pos → gnss_msgs/RtkFix rosbag2(spec §12.3,Task 13 Step 5)

**必须在 Orin(ROS 2 + rosbag2_py + gnss_msgs)上跑;本脚本在无 ROS 的云端环境编写,未在本环境运行过**,
现场首次使用请先用一个短 .pos 试跑并 `ros2 bag info` 核对消息数与 topic。

usage:
  pos_to_rtkfix_bag.py [<in.pos>] --out <bag_dir> [--topic /gnss_cgi610/rtk_fix] [--frame gnss]
                       [--merge <lidar_bag_dir>] [--pos-time UTC|GPST] [--leap 18]
                       [--header-delay 0.05] [--storage sqlite3|mcap]
                       [--pose-enu <f.enu> --pose-topic /gnss/pose] [--pose-frame enu]

<in.pos> 与 --pose-enu 至少给一个;两者都给时一次调用同时写两个话题(轮 5 Task 3,
spec §12.4 的 A/B/C 三组共用一份合并 bag,差异只剩模块本身)。

字段映射(PLAN Task 13 Step 5):
  header.stamp = t + header_delay(默认 0.05,模拟接收延迟,让 stamp_source 的差别可见)
  gnss_time    = t
  quality      = q_to_quality(Q)   1→FIXED(4) 2→FLOAT(3) 4→DGPS(2) 5→SINGLE(1) 其它→NONE(0)
  latitude/longitude/altitude = lat/lon/height   (字段名与 gnss_msgs/RtkFix.msg 一致)
  sigma_enu    = [sde, sdn, sdu]   (.pos 列顺序是 sdn sde sdu,注意换序)
  diff_age     = age
  sats_used    = ns
  heading      = 0, heading_valid = False

--pose-enu:读 pos_to_enu 产出的 .enu(轮 5 Task 2),写 geometry_msgs/PoseWithCovarianceStamped
到 --pose-topic(默认 /gnss/pose),给只订阅该类型的 libgnss_global.so 用。position.{x,y,z} = e/n/u,
orientation 固定为单位四元数(ENU 轨迹不含姿态);covariance 填 sde/sdn/sdu 的平方与一个大值
(姿态"未知"),但 gnss_global 完全忽略协方差(gnss_global_module.hpp:89-95),这里只是让别的
消费者与肉眼核对用。

--merge:把 LiDAR/IMU bag 的全部消息与合成的 RtkFix/Pose 按时间戳归并写进同一个 bag,避免多 bag
回放时钟对齐问题。源 bag 里与 --topic 或 --pose-topic 同名的话题会被丢弃并打警告。
"""
import argparse
import heapq
import os
import sys
from datetime import datetime, timezone

# ---- .pos 解析(与 gnss_core::read_pos 同语义:GPST 减闰秒,UTC 原样,无头按 --pos-time) ----

GPS_EPOCH_UNIX = 315964800.0        # 1980-01-06T00:00:00Z
SECONDS_PER_WEEK = 604800.0


def _parse_stamp(c0, c1):
    """时间两列 → unix 秒(尚未做 GPST→UTC 换算);解析不出来返回 None。

    两种格式(与 gnss_core::read_pos 一致):
      - 日历:  "2026/09/15" "08:49:50.000"
      - 周/tow: "2436" "204589.000"(rnx2rtkp 默认)
    """
    if "/" in c0:
        try:
            dt = datetime.strptime(c0 + " " + c1[:12], "%Y/%m/%d %H:%M:%S.%f")
        except ValueError:
            return None
        return dt.replace(tzinfo=timezone.utc).timestamp()
    try:
        week = int(c0)
        tow = float(c1)
    except ValueError:
        return None
    if week < 0 or not (0.0 <= tow < SECONDS_PER_WEEK):
        return None
    return GPS_EPOCH_UNIX + week * SECONDS_PER_WEEK + tow


def parse_pos(path, default_time="GPST", leap=18):
    ts = default_time
    ts_explicit = False
    recs = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("%"):
                if "time=GPST" in line:
                    ts = "GPST"
                    ts_explicit = True
                elif "time=UTC" in line:
                    ts = "UTC"
                    ts_explicit = True
                elif not ts_explicit:
                    # RTKLIB 列名行 "%  GPST  latitude(deg) ..." / "%  UTC ...":第一个 token 即时间系统。
                    # 只在没有更早的显式 "time=GPST/UTC)" 标注时才信它——列名行本身不代表文件
                    # 真实的时间系统发生了变化,只是同一份文件里重复出现的另一种写法。
                    tok = line[1:].split()
                    if tok and tok[0] in ("GPST", "UTC"):
                        ts = tok[0]
                continue
            c = line.split()
            if len(c) < 10:
                continue
            t = _parse_stamp(c[0], c[1])
            if t is None:
                continue
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


def parse_enu(path):
    """读 pos_to_enu 写出的 .enu(格式见 gnss_core/enu_track.hpp)。

    返回 (origin_lla | None, samples);samples 每项
    {"t","e","n","u","sde","sdn","sdu","q"},q 是归一化质量枚举(0..4)。
    注释行、列数不足的行、非数值行一律跳过,不中断。
    """
    origin = None
    samples = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("#"):
                tok = line[1:].split()
                if len(tok) >= 4 and tok[0] == "origin_lla":
                    try:
                        origin = (float(tok[1]), float(tok[2]), float(tok[3]))
                    except ValueError:
                        origin = None
                continue
            c = line.split()
            if len(c) < 8:
                continue
            try:
                s = {"t": float(c[0]), "e": float(c[1]), "n": float(c[2]), "u": float(c[3]),
                     "sde": float(c[4]), "sdn": float(c[5]), "sdu": float(c[6]), "q": int(c[7])}
            except ValueError:
                continue
            if not 0 <= s["q"] <= 4:
                continue
            samples.append(s)
    return origin, samples


Q_TO_QUALITY = {1: 4, 2: 3, 4: 2, 5: 1}   # RTKLIB Q → gnss_msgs 归一化质量(spec §4.1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("pos", nargs="?", default=None)
    ap.add_argument("--out", required=True, help="输出 bag 目录(不得已存在)")
    ap.add_argument("--topic", default="/gnss_cgi610/rtk_fix")
    ap.add_argument("--frame", default="gnss")
    ap.add_argument("--merge", default=None, help="要合并的 LiDAR/IMU bag 目录")
    ap.add_argument("--pos-time", default="GPST", choices=["GPST", "UTC"], help="无头部标注时的时间系统")
    ap.add_argument("--leap", type=int, default=18)
    ap.add_argument("--header-delay", type=float, default=0.05)
    ap.add_argument("--storage", default="sqlite3", choices=["sqlite3", "mcap"])
    ap.add_argument("--pose-enu", default=None,
                    help="pos_to_enu 产出的 .enu;给上游 gnss_global 写 PoseWithCovarianceStamped")
    ap.add_argument("--pose-topic", default="/gnss/pose")
    ap.add_argument("--pose-frame", default="enu")
    args = ap.parse_args()

    if args.pos is None and args.pose_enu is None:
        print("需要 <in.pos> 或 --pose-enu 至少一个", file=sys.stderr)
        return 2

    recs = []
    if args.pos is not None:
        recs = parse_pos(args.pos, args.pos_time, args.leap)
        if not recs:
            print("no epochs parsed from", args.pos, file=sys.stderr)
            return 1

    pose_samples = []
    if args.pose_enu is not None:
        _, pose_samples = parse_enu(args.pose_enu)

    if os.path.exists(args.out):
        print("output exists:", args.out, file=sys.stderr)
        return 1

    # ---- ROS 2 依赖:仅在 Orin 上可用 ----
    try:
        import rosbag2_py
        from rclpy.serialization import serialize_message
        from builtin_interfaces.msg import Time
        from gnss_msgs.msg import RtkFix
        from geometry_msgs.msg import PoseWithCovarianceStamped
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

    def make_pose(s):
        m = PoseWithCovarianceStamped()
        m.header.stamp = to_time(s["t"] + args.header_delay)
        m.header.frame_id = args.pose_frame
        m.pose.pose.position.x = s["e"]
        m.pose.pose.position.y = s["n"]
        m.pose.pose.position.z = s["u"]
        m.pose.pose.orientation.w = 1.0
        # gnss_global 完全忽略协方差(gnss_global_module.hpp:89-95),这里照样填上,
        # 方便别的消费者与肉眼核对;姿态部分给一个大值表示"未知"。
        cov = [0.0] * 36
        cov[0] = s["sde"] ** 2
        cov[7] = s["sdn"] ** 2
        cov[14] = s["sdu"] ** 2
        cov[21] = cov[28] = cov[35] = 1e6
        m.pose.covariance = cov
        return m

    # Jazzy 的 TopicMetadata 需要 id 位置/关键字参数;本环境装的 rosbag2_py 0.15.17(Humble)
    # 的构造函数完全不接受 id(连关键字形式都会 TypeError——本脚本第一次实际运行时在这里炸的)。
    # 两种都试,id 分配约定不变:0 给 --topic(RtkFix),1 给 --pose-topic(Pose),merge 来源从 2 起。
    def create_topic(topic_id, name, type_, serialization_format="cdr", offered_qos_profiles=""):
        try:
            tm = rosbag2_py.TopicMetadata(
                id=topic_id, name=name, type=type_, serialization_format=serialization_format,
                offered_qos_profiles=offered_qos_profiles)
        except TypeError:
            tm = rosbag2_py.TopicMetadata(
                name=name, type=type_, serialization_format=serialization_format,
                offered_qos_profiles=offered_qos_profiles)
        writer.create_topic(tm)

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=args.out, storage_id=args.storage),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    if recs:
        create_topic(0, args.topic, "gnss_msgs/msg/RtkFix")
    if pose_samples:
        create_topic(1, args.pose_topic, "geometry_msgs/msg/PoseWithCovarianceStamped")

    # 合成 RtkFix 序列(bag 接收时间 = header.stamp;元组第 2 项用于同时刻排序:LiDAR 侧先写)
    def rtk_iter():
        for r in sorted(recs, key=lambda x: x["t"]):
            t_ns = int(round((r["t"] + args.header_delay) * 1e9))
            yield (t_ns, 1, args.topic, serialize_message(make_msg(r)))

    def pose_iter():
        for s in sorted(pose_samples, key=lambda x: x["t"]):
            t_ns = int(round((s["t"] + args.header_delay) * 1e9))
            yield (t_ns, 1, args.pose_topic, serialize_message(make_pose(s)))

    n_rtk = 0
    n_pose = 0
    n_merge = 0

    def write_and_count(topic, data, t_ns):
        writer.write(topic, data, t_ns)
        nonlocal n_rtk, n_pose, n_merge
        if topic == args.topic:
            n_rtk += 1
        elif topic == args.pose_topic:
            n_pose += 1
        else:
            n_merge += 1

    if args.merge is None:
        # 无源 bag 可合并时,RtkFix 与 Pose 两路(谁非空就用谁)仍按时间升序归并写出。
        for t_ns, _, topic, data in heapq.merge(rtk_iter(), pose_iter(), key=lambda x: (x[0], x[1])):
            write_and_count(topic, data, t_ns)
    else:
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=args.merge, storage_id=""),   # 空 storage_id:自动识别
            rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
        )
        for i, tm in enumerate(reader.get_all_topics_and_types(), start=2):
            if tm.name in (args.topic, args.pose_topic):
                print(f"warning: source bag already has {tm.name}; its messages are dropped", file=sys.stderr)
                continue
            create_topic(i, tm.name, tm.type, tm.serialization_format,
                         getattr(tm, "offered_qos_profiles", ""))   # 保留源 QoS

        def lidar_iter():
            # SequentialReader 按接收时间升序输出
            while reader.has_next():
                topic, data, t_ns = reader.read_next()
                if topic in (args.topic, args.pose_topic):
                    continue
                yield (t_ns, 0, topic, data)

        # 三路均按时间升序 → heapq.merge 线性归并,不必整包读入内存
        for t_ns, src, topic, data in heapq.merge(lidar_iter(), rtk_iter(), pose_iter(), key=lambda x: (x[0], x[1])):
            write_and_count(topic, data, t_ns)

    if hasattr(writer, "close"):
        writer.close()   # Jazzy 提供显式 close;老版本靠析构 flush
    del writer
    print(f"wrote {n_rtk} RtkFix on {args.topic}, {n_pose} poses on {args.pose_topic}"
          + (f" + {n_merge} merged messages" if args.merge else "") + f" -> {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
