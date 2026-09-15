#!/usr/bin/env python3
"""run_field_integration.sh 残留进程检查模式的自检(`python3 -m unittest`,不需要 ROS)。

脚本用 `pgrep -af "$PROC_PAT"` 找回放链路的残留进程。模式锚定在程序名上,避免把命令行里恰好
带这些词的 shell/grep/tail 误报成残留;反过来也不能漏掉任何一个真实进程(Task 3 审阅发现
`rtkrcv_node` 曾经漏掉)。这里用 `grep -E`(与 pgrep 同为 glibc ERE,都认 `\\S`)对样例命令行跑
脚本 `--print-proc-pat` 打印出来的同一个模式。

最终修复轮:通用的 `ros2 bag|launch|run` 拆成单独的模式,脚本只把 ROS_DOMAIN_ID 等于本次 domain(66)
的算作残留(别的终端里无关的 ros2 launch 不再误报),这里用真进程验证;收尾兜底杀 rtkrcv 的模式
只认 rtkrcv 程序且 `-o` 正好是本次运行的 conf。
"""
import os
import shutil
import subprocess
import time
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
    # 通用 ros2 命令不在具体程序名模式里(改由 ROS2_CLI 模式 + domain 过滤处理)
    "/usr/bin/python3 /opt/ros/humble/bin/ros2 launch some_pkg other.launch.py",
    "ros2 bag record -o /x/bags /clock",
]

# ROS2_CLI 模式必须命中的(再经 domain 过滤)
ROS2_CLI_MUST_MATCH = [
    "/usr/bin/python3 /opt/ros/humble/bin/ros2 bag record -o /x/bags/gnss_1 --storage sqlite3 /clock",
    "/usr/bin/python3 /opt/ros/humble/bin/ros2 launch gnss_bringup gnss_bringup.launch.py params_file:=/x",
    "/usr/bin/python3 /opt/ros/humble/bin/ros2 run gnss_chcnav gnss_chcnav_can --ros-args",
    "ros2 bag record -o /x/bags /clock",
]
ROS2_CLI_MUST_NOT_MATCH = [
    "/usr/bin/python3 /opt/ros/humble/bin/ros2 topic hz /clock",
    "/usr/bin/python3 -c from ros2cli.daemon.daemonize import main; main() --name ros2-daemon --ros-domain-id 66",
    "grep ros2 launch /x/logs/bringup.log",
]

# 收尾兜底杀 rtkrcv 的模式;RUN 故意带 ERE 元字符,验证按字面匹配
RTK_RUN = "/data/seg.1/integration_20260916/fix+check(1)/run_dry"
RTK_MUST_MATCH = [
    "rtkrcv -s -nc -r 2 -o %s/rtkrcv/rtkrcv.conf" % RTK_RUN,
    "/usr/local/bin/rtkrcv -s -nc -r 2 -o %s/rtkrcv/rtkrcv.conf -t 3" % RTK_RUN,
]
RTK_MUST_NOT_MATCH = [
    "tail -f %s/rtkrcv/rtkrcv.conf" % RTK_RUN,
    "vim %s/rtkrcv/rtkrcv.conf" % RTK_RUN,
    "bash -c rtkrcv -s -nc -r 2 -o %s/rtkrcv/rtkrcv.conf" % RTK_RUN,
    "/usr/local/bin/rtkrcv_node --ros-args -p run_dir:=%s/rtkrcv -o %s/rtkrcv/rtkrcv.conf" % (RTK_RUN, RTK_RUN),
    "rtkrcv -s -nc -r 2 -o %s/rtkrcv/rtkrcv.conf.bak" % RTK_RUN,
    "rtkrcv -s -nc -r 2 -o /data/seg.1/integration_20260916/run_dry/rtkrcv/rtkrcv.conf",          # 别的运行目录
    "rtkrcv -s -nc -r 2 -o /data/segX1/integration_20260916/fix+check(1)/run_dry/rtkrcv/rtkrcv.conf",  # . 不能当通配
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

    def test_ros2_cli_pattern(self):
        pat = subprocess.run(["bash", SCRIPT, "--print-ros2-cli-pat"], capture_output=True, text=True,
                             check=True).stdout.strip()
        self.assertEqual([ln for ln in ROS2_CLI_MUST_MATCH if not matches(pat, ln)], [])
        self.assertEqual([ln for ln in ROS2_CLI_MUST_NOT_MATCH if matches(pat, ln)], [])

    def test_rtkrcv_fallback_pattern_only_matches_this_runs_rtkrcv(self):
        pat = subprocess.run(["bash", SCRIPT, "--print-rtkrcv-pat", RTK_RUN], capture_output=True, text=True,
                             check=True).stdout.strip()
        self.assertEqual([ln for ln in RTK_MUST_MATCH if not matches(pat, ln)], [], "兜底会漏杀本次的 rtkrcv")
        self.assertEqual([ln for ln in RTK_MUST_NOT_MATCH if matches(pat, ln)], [], "兜底会误杀这些进程")


@unittest.skipUnless(os.path.isdir("/proc/self") and shutil.which("pgrep") and shutil.which("sleep"),
                     "需要 Linux /proc 与 pgrep")
class LeftoverDomainFilterTest(unittest.TestCase):
    """用 argv[0] 伪装成 `ros2 launch …` 的 sleep 进程验证:只有 ROS_DOMAIN_ID=66 的才算残留。"""

    def _spawn(self, tag, domain):
        env = {k: v for k, v in os.environ.items() if k != "ROS_DOMAIN_ID"}
        if domain is not None:
            env["ROS_DOMAIN_ID"] = domain
        p = subprocess.Popen(["ros2 launch field_integ_leftover_test %s" % tag, "60"],
                             executable=shutil.which("sleep"), env=env)
        self.addCleanup(lambda: (p.kill(), p.wait()))
        return p

    def test_only_this_domain_counts(self):
        ours = self._spawn("ours", "66")
        other = self._spawn("other", "12")
        unset = self._spawn("unset", None)
        # 等 exec 完成(cmdline 变成伪装的 argv[0])
        deadline = time.time() + 5
        for p in (ours, other, unset):
            while time.time() < deadline:
                try:
                    with open("/proc/%d/cmdline" % p.pid, "rb") as f:
                        if f.read().startswith(b"ros2 launch field_integ_leftover_test"):
                            break
                except OSError:
                    pass
                time.sleep(0.02)
        out = subprocess.run(["bash", SCRIPT, "--list-leftovers"], capture_output=True, text=True,
                             check=True).stdout
        pids = {ln.split()[0] for ln in out.splitlines() if ln.strip()}
        self.assertIn(str(ours.pid), pids)
        self.assertNotIn(str(other.pid), pids)
        self.assertNotIn(str(unset.pid), pids)


if __name__ == "__main__":
    unittest.main()
