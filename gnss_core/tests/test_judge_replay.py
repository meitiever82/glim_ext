#!/usr/bin/env python3
"""judge_replay CLI 骨架测试。

构造一个极小的北云/华测 ASCII 样本流，运行 judge_replay，断言：
- 输出 CSV 行数与主帧数一致
- 首行字段格式正确
- 固定解帧的 is_valid=1、rtk_status=4（RtkFix）、fault_code=0
"""
import os
import subprocess
import tempfile
import unittest


def find_judge_replay() -> str:
    """从测试文件位置反推 workspace root，查找 build/install 里的二进制。"""
    test_dir = os.path.dirname(os.path.abspath(__file__))
    # src/glim_ext/gnss_core/tests -> workspace root
    ws_root = os.path.abspath(os.path.join(test_dir, "../../../.."))
    candidates = [
        os.path.join(ws_root, "install/gnss_core/lib/gnss_core/judge_replay"),
        os.path.join(ws_root, "build/gnss_core/judge_replay"),
    ]
    for c in candidates:
        if os.path.isfile(c) and os.access(c, os.X_OK):
            return c
    raise FileNotFoundError("judge_replay binary not found; build gnss_core first")


class JudgeReplayTest(unittest.TestCase):
    def run_replay(self, brand: str, log: str, origin: str = "0,0,0") -> tuple:
        exe = find_judge_replay()
        with tempfile.NamedTemporaryFile(mode="w", suffix=".log", delete=False) as f:
            f.write(log)
            path = f.name
        try:
            result = subprocess.run(
                [exe, brand, path, "--origin", origin],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(result.returncode, 0, msg=f"stderr: {result.stderr}")
            lines = result.stdout.strip().splitlines()
            return lines
        finally:
            os.unlink(path)

    def test_beiyun_basic(self):
        log = (
            "#GPGGA,123519.00,4400.000,N,09200.000,E,4,18,0.9,753.8,M,0.0,M,1.0,0000*7B\n"
            "#GPFPD,2204,325283.000,90.0,0.0,0.0,44.0000,92.0000,753.8000,0.0,0.0,0.0,1.601,18,20,4*XX\n"
            "#HEADINGA,COM1,0,49.5,FINESTEERING,2204,325283.000,00000000,0000,1272;"
            "SOL,NARROW_INT,1.601,90.0,0.1,0.2,0.3,0.4,1.0,1.0,20,18*XX\n"
            "#INSPVAXA,COM1,0,49.5,FINESTEERING,2204,325284.000,00000000,0000,1272;"
            "INS_SOLUTION_GOOD,INS_RTKFIXED,44.0000,92.0000,753.8000,0.0000,"
            "1.0,2.0,3.0,0.1,0.2,90.0*XX\n"
            "#INSPVAXA,COM1,0,49.5,FINESTEERING,2204,325285.000,00000000,0000,1272;"
            "INS_SOLUTION_GOOD,INS_RTKFLOAT,44.0000,92.0000,753.8000,0.0000,"
            "1.0,2.0,3.0,0.1,0.2,90.0*XX\n"
        )
        lines = self.run_replay("beiyun", log, origin="44.0,92.0,753.8")
        self.assertEqual(lines[0], "gps_sec,is_valid,rtk_status,fault_code,invalid_reason")
        self.assertEqual(len(lines), 3)  # header + 2 main frames
        parts = lines[1].split(",")
        self.assertEqual(len(parts), 5)
        self.assertEqual(parts[1], "1")  # is_valid
        self.assertEqual(parts[2], "4")  # RtkFix
        self.assertEqual(parts[3], "0")  # fault_code None
        self.assertEqual(parts[4], "0")  # invalid_reason LOC_VALID

    def test_huace_basic(self):
        log = (
            "$GPCHC,2204,325284.000,90.0,0.0,0.0,0.0,0.0,0.0,"
            "0.0,0.0,0.0,44.0000,92.0000,753.8000,1.0,2.0,3.0,"
            "0.0,18,20,0x42,1,0x0000*XX\n"
        )
        lines = self.run_replay("huace", log, origin="44.0,92.0,753.8")
        self.assertEqual(lines[0], "gps_sec,is_valid,rtk_status,fault_code,invalid_reason")
        self.assertEqual(len(lines), 2)
        parts = lines[1].split(",")
        self.assertEqual(len(parts), 5)
        self.assertEqual(parts[1], "1")  # is_valid
        self.assertEqual(parts[2], "4")  # RtkFix
        self.assertEqual(parts[3], "0")  # fault_code None
        self.assertEqual(parts[4], "0")  # invalid_reason LOC_VALID


if __name__ == "__main__":
    unittest.main()
