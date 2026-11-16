from pathlib import Path
from gnss_hmi.config import load_config

EXAMPLE = Path(__file__).resolve().parents[1] / "config.yaml.example"

def test_load_example_config():
    cfg = load_config(EXAMPLE)
    assert cfg.data_root == Path("/data/gnsslog")
    assert cfg.corrections.host == "192.168.10.1"
    assert cfg.corrections.port == 6001
    assert cfg.corrections.listen is False          # default
    assert cfg.gnss_solution.listen is True
    assert cfg.can_channel == "can0"
    assert cfg.reserve_corrections_port == 15010
    assert cfg.reserve_raw_obs_port == 15011
    assert cfg.retention_days == 14
    assert cfg.disk_watermark_pct == 85.0

def test_missing_key_raises(tmp_path):
    p = tmp_path / "bad.yaml"
    p.write_text("data_root: /tmp/x\n")
    try:
        load_config(p)
        assert False, "should raise"
    except KeyError:
        pass

def test_plan2_sections_defaults(tmp_path):
    # A Plan-1-era config with no rtkrcv/diagnosis/publish sections must still load.
    p = tmp_path / "old.yaml"
    p.write_text(EXAMPLE.read_text())  # example will gain the sections; strip them
    lines = p.read_text().splitlines()
    # Strip Plan 2 sections by skipping lines that start with their section names
    # or belong to them (indented children)
    in_plan2_section = False
    text_lines = []
    for line in lines:
        if line.startswith(("rtkrcv", "diagnosis", "publish")):
            in_plan2_section = True
        elif line and not line[0].isspace():
            # Top-level key (not indented) - exit Plan 2 section
            in_plan2_section = False
            text_lines.append(line)
        elif not in_plan2_section:
            text_lines.append(line)
    text = "\n".join(text_lines)
    p.write_text(text)
    cfg = load_config(p)
    assert cfg.rtkrcv.binary == "" and cfg.rtkrcv.sol_port == 15020
    assert cfg.diagnosis.corr_gap_s == 3.0 and cfg.diagnosis.min_sats == 6
    assert cfg.diagnosis.close_hysteresis_s == 10.0
    assert cfg.publish.enabled is False and cfg.publish.port == 15030

def test_plan2_sections_explicit():
    cfg = load_config(EXAMPLE)
    assert cfg.rtkrcv.sol_port == 15020          # example carries the sections
    assert cfg.publish.host == "127.0.0.1"

def test_web_and_retention_defaults(tmp_path):
    # A Plan-3a-era config with no web section or db_retention_days must still load with defaults.
    p = tmp_path / "old.yaml"
    p.write_text(EXAMPLE.read_text())  # example will gain the sections; strip them
    lines = p.read_text().splitlines()
    # Strip Plan 3a sections by skipping lines that start with their section names
    # or belong to them (indented children), and skip db_retention_days
    in_web_section = False
    text_lines = []
    for line in lines:
        if line.startswith("web:"):
            in_web_section = True
        elif line.startswith("db_retention_days"):
            # Skip db_retention_days line entirely
            pass
        elif line and not line[0].isspace():
            # Top-level key (not indented) - exit web section
            in_web_section = False
            text_lines.append(line)
        elif not in_web_section:
            text_lines.append(line)
    text = "\n".join(text_lines)
    p.write_text(text)
    cfg = load_config(p)
    assert cfg.web.port == 8080 and cfg.web.tiles_path == ""
    assert cfg.db_retention_days == 30

def test_web_explicit():
    cfg = load_config(EXAMPLE)
    assert cfg.web.port == 8080          # example carries the section

def test_web_host_and_static_dir_defaults():
    cfg = load_config(EXAMPLE)
    assert cfg.web.host == "0.0.0.0" and cfg.web.static_dir == ""
