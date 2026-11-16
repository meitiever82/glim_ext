import os
import re
import shlex
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]

# RTKLIB-EX v2.5.1 tag 对应的 commit。rtkrcv_node 依赖该版本才有的 -nc 选项
# (见 gnss_bringup README「安装 RTKLIB-EX 2.5.1」),所以镜像里的版本不能漂。
# 刷新方式:git ls-remote https://github.com/rtklibexplorer/RTKLIB.git refs/tags/v2.5.1
RTKLIB_V251_SHA = "62d4677ed8425a4e2748c6d390b500d1afb493fc"


def test_dockerfile_has_two_stages():
    text = (ROOT / "Dockerfile").read_text()
    assert text.count("FROM ") == 2 and "rtkbuild" in text
    assert "web ./web" in text and "/usr/local/bin/rtkrcv" in text


def test_dockerfile_pins_rtklib_to_a_commit():
    text = (ROOT / "Dockerfile").read_text()
    # 钉 tag 的同时校验 commit sha:tag 被上游移动时构建失败,而不是静默换版本。
    assert "RTKLIB_SHA=" in text
    assert re.search(r"RTKLIB_SHA=[0-9a-f]{40}\b", text)
    assert RTKLIB_V251_SHA in text
    assert "rev-parse HEAD" in text and '"$RTKLIB_SHA"' in text


def test_build_rtkrcv_pins_tag_not_branch():
    text = (ROOT / "scripts" / "build_rtkrcv.sh").read_text()
    # demo5 分支头会移动,只允许按 tag(默认 v2.5.1)取源码。
    assert "RTKLIB_REF:-v2.5.1" in text
    assert '--branch "$RTKLIB_REF"' in text
    assert "--branch demo5" not in text


@pytest.mark.skipif(shutil.which("docker") is None, reason="docker not available")
@pytest.mark.slow
def test_docker_image_builds():
    extra = shlex.split(os.environ.get("RTK_DOCKER_BUILD_ARGS", ""))
    r = subprocess.run(["docker", "build", "-q", *extra, "."], cwd=ROOT,
                       capture_output=True, text=True, timeout=900)
    assert r.returncode == 0, r.stderr[-2000:]
