import re
from pathlib import Path

WEB = Path(__file__).resolve().parents[1] / "web"
VENDOR = WEB / "vendor"

REQUIRED = [
    "vue.esm-browser.prod.js",
    "leaflet/leaflet.js", "leaflet/leaflet.css", "leaflet/images/marker-icon.png",
    "uplot/uPlot.iife.min.js", "uplot/uPlot.min.css",
    "VENDOR.md",
]


def test_vendor_assets_present_and_nonempty():
    for rel in REQUIRED:
        p = VENDOR / rel
        assert p.is_file(), f"missing {rel}"
        assert p.stat().st_size > 100, f"suspiciously small: {rel}"


def test_vendor_manifest_pins_versions():
    text = (VENDOR / "VENDOR.md").read_text()
    for needle in ("vue@3.4.38", "leaflet@1.9.4", "uplot@1.6.30"):
        assert needle in text


def test_index_references_resolve():
    html = (WEB / "index.html").read_text()
    refs = re.findall(r'(?:src|href)="/([^"]+)"', html)
    assert refs, "index.html should reference local assets"
    for ref in refs:
        assert (WEB / ref).is_file(), f"broken reference: /{ref}"


def test_index_is_dark_chinese_ui():
    html = (WEB / "index.html").read_text()
    assert 'lang="zh-CN"' in html and "gnss-hmi" in html
    css = (WEB / "style.css").read_text()
    assert "--bg" in css                      # theme variables present
