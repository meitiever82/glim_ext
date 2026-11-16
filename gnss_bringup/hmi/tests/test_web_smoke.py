# tests/test_web_smoke.py — full-page module-graph smoke + JS syntax gate.
# Ruling 1: _FakeApp is copied locally (no tests/__init__.py packaging), mirroring
# tests/test_api.py's version — only the attributes create_api touches.
# Ruling 2: the module-graph walker resolves both absolute ("/js/x.js", "/vendor/x.js")
# and relative ("./x.js", resolved against the importing file's directory) imports.
import posixpath
import re
import shutil
import subprocess
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

WEB = Path(__file__).resolve().parents[1] / "web"


class _FakeApp:
    """Minimal stand-in exposing exactly what create_api consumes.

    Copied locally per controller ruling 1 (no tests/__init__.py packaging to
    import from tests.test_api). Mirrors tests/test_api.py's _FakeApp.
    """
    def __init__(self, tmp_path):
        from gnss_hmi.broadcast import Broadcaster
        from gnss_hmi.diagnosis.base_station import BaseStationMonitor
        from gnss_hmi.storage.epochs import EpochStore
        from gnss_hmi.storage.events import EventStore
        self.epochs = EpochStore(tmp_path / "a.db")
        self.events = EventStore(tmp_path / "a.db")
        self.broadcaster = Broadcaster()
        self.base_monitor = BaseStationMonitor(self.epochs, warmup_s=1.0)
        self.last_status = {"type": "status", "t": 123.0}
        self.tile_store = None


def _client(tmp_path):
    from gnss_hmi.api import create_api
    return TestClient(create_api(_FakeApp(tmp_path)))


_IMPORT_RE = re.compile(
    r'import\s*(?:\*\s+as\s+\w+\s+from\s+|[^"\';]*?\bfrom\s+|)"([^"]+)"'
)


def _local_imports(js_text, from_path):
    """Return absolute site-root paths for every local import in js_text.

    Handles: `import X from "..."`, `import * as X from "..."`,
    `import { a, b } from "..."`, and bare `import "..."`. Resolves absolute
    specifiers ("/js/x.js", "/vendor/x.js") as-is, and relative specifiers
    ("./x.js", "../x.js") against from_path's directory.
    """
    out = []
    for spec in _IMPORT_RE.findall(js_text):
        if spec.startswith("/"):
            out.append(spec)
        elif spec.startswith("."):
            base_dir = posixpath.dirname(from_path)
            resolved = posixpath.normpath(posixpath.join(base_dir, spec))
            out.append(resolved)
        # else: bare/external specifier (e.g. a package name) — not local, skip.
    return out


def test_module_graph_resolves_over_http(tmp_path):
    c = _client(tmp_path)
    seen, todo = set(), ["/app.js"]
    while todo:
        path = todo.pop()
        if path in seen:
            continue
        seen.add(path)
        r = c.get(path)
        assert r.status_code == 200, f"unresolved module {path}"
        if path.endswith(".js"):
            todo.extend(_local_imports(r.text, path))
    assert "/js/protocol.js" in seen and "/js/mapview.js" in seen


@pytest.mark.skipif(shutil.which("node") is None, reason="node not available")
def test_all_js_files_parse():
    for p in sorted(WEB.rglob("*.js")):
        if "vendor" in p.parts:
            continue
        r = subprocess.run(["node", "--check", str(p)], capture_output=True, text=True)
        assert r.returncode == 0, f"{p}: {r.stderr}"
