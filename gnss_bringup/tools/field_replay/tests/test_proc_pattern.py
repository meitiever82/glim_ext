#!/usr/bin/env python3
"""run_field_integration.sh 残留进程检查模式的自检(`python3 -m unittest`,不需要 ROS)。

脚本用 `pgrep -af "$PROC_PAT"` 找回放链路的残留进程。模式锚定在程序名上,避免把命令行里恰好
带这些词的 shell/grep/tail 误报成残留;反过来也不能漏掉任何一个真实进程(Task 3 审阅发现
`rtkrcv_node` 曾经漏掉)。这里用 `grep -E`(与 pgrep 同为 glibc ERE,都认 `\\S`)对样例命令行跑
脚本 `--print-proc-pat` 打印出来的同一个模式。
"""
import os
import shutil
import subprocess
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(HERE, "..", "run_field_integration.sh")

# 必须命中:回放链路里真实出现过的命令行形式(取自 2026-09-16 dry 试跑的 pgrep -af 输出)
MUST_MATCH = [
    "/home/steve/glim_ws/install/gnss_bringup/lib/gnss_bringup/rtkrcv_node --ros-args -r __node:=rtkrcv_node --params-file /x/params.yaml",
    "/home/steve/glim_ws/install/gnss_bringup/lib/gnss_bringup/rtcm_bridge --ros-args -r __node:=rtcm_bridge",
    "/home/steve/glim_ws/install/gnss_bringup/lib/gnss_bringup/pos_writer --ros-args -r __node:=pos_writer",
    "/home/steve/glim_ws/install/gnss_bringup/lib/gnss_bringup/gnss_diag_node --ros-args --params-file /x/params.yaml",
    "/home/steve/glim_ws/install/gnss_bringup/lib/gnss_bringup/gnss_cleanup_node --ros-args",
    "/home/steve/driver_ws/install/gnss_chcnav/lib/gnss_chcnav/gnss_chcnav_can --ros-args -r __node:=gnss_cgi610",
    "rtkrcv -s -nc -r 2 -o /x/run_dry/rtkrcv/rtkrcv.conf",
    "/usr/local/bin/rtkrcv -s -nc -r 2 -o /x/rtkrcv.conf",
    "canplayer -I /x/can_with_sigma.log vcan0=can7",
    "/usr/bin/canplayer -I /x/can_with_sigma.log vcan0=can7",
    "/usr/bin/python3 /opt/ros/humble/bin/ros2 bag record -o /x/bags/gnss_1 --storage sqlite3 /clock",
    "/usr/bin/python3 /opt/ros/humble/bin/ros2 launch gnss_bringup gnss_bringup.launch.py params_file:=/x",
    "/usr/bin/python3 /opt/ros/humble/bin/ros2 run gnss_chcnav gnss_chcnav_can --ros-args",
    "ros2 bag record -o /x/bags /clock",
    "/usr/bin/python3 -u /home/steve/glim_ws/src/glim_ext/gnss_bringup/tools/field_replay/field_replay.py --rtcm /x",
    "python3 /home/steve/glim_ws/install/gnss_bringup/lib/gnss_bringup/field_replay.py --rtcm /x",
    "/home/steve/.pyenv/versions/3.10.12/bin/python3.10 -u /x/field_replay.py --rtcm /x",
]

# 必须不命中:命令行里含这些词、但不是链路进程
MUST_NOT_MATCH = [
    "bash -c something rtkrcv_node",
    "/bin/bash -c until grep -q pos_writer /tmp/x.out; do sleep 5; done",
    "bash /home/steve/glim_ws/src/glim_ext/gnss_bringup/tools/field_replay/run_field_integration.sh dry /x 60",
    "tee /x/run_dry/logs/field_replay.log",
    "grep rtkrcv_node /x/logs/bringup.log",
    "tail -f /x/run_dry/rtkrcv/rtkrcv.conf",
    "less /x/logs/canplayer.log",
    "/usr/bin/python3 -c from ros2cli.daemon.daemonize import main; main() --name ros2-daemon --ros-domain-id 66",
    "/usr/bin/python3 /opt/ros/humble/bin/ros2 topic hz /clock",
    "vim /x/field_replay.py",
    "/home/steve/bin/rtkrcv_node_backup --ros-args",
]


def matches(pattern: str, line: str) -> bool:
    r = subprocess.run(["grep", "-qE", "--", pattern], input=line + "\n", text=True)
    if r.returncode not in (0, 1):
        raise RuntimeError("grep -E 执行失败(模式非法?): rc=%d" % r.returncode)
    return r.returncode == 0


@unittest.skipUnless(shutil.which("bash") and shutil.which("grep"), "需要 bash 与 grep")
class ProcPatternTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        out = subprocess.run(["bash", SCRIPT, "--print-proc-pat"], capture_output=True, text=True, check=True)
        cls.pattern = out.stdout.strip()

    def test_pattern_is_printed(self):
        self.assertTrue(self.pattern)

    def test_every_pipeline_process_matches(self):
        missed = [ln for ln in MUST_MATCH if not matches(self.pattern, ln)]
        self.assertEqual(missed, [], "残留检查会漏掉这些进程")

    def test_unrelated_command_lines_do_not_match(self):
        false_hits = [ln for ln in MUST_NOT_MATCH if matches(self.pattern, ln)]
        self.assertEqual(false_hits, [], "残留检查会误报这些进程")


if __name__ == "__main__":
    unittest.main()
