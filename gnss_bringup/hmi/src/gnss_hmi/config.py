"""Load and validate config.yaml into typed dataclasses."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass(frozen=True)
class StreamCfg:
    host: str
    port: int
    listen: bool = False
    enabled: bool = True       # P0 只读配置显式置 false，禁用旧自采集路径


@dataclass(frozen=True)
class RtkrcvCfg:
    binary: str = ""
    sol_port: int = 15020
    extra_args: tuple[str, ...] = ()


@dataclass(frozen=True)
class DiagnosisCfg:
    corr_gap_s: float = 3.0
    age_max_s: float = 10.0
    base_shift_m: float = 0.1
    min_sats: int = 6
    resid_max_m: float = 2.0
    low_el_deg: float = 20.0
    low_snr_dbhz: float = 35.0
    min_ratio: float = 3.0
    slip_max_per_30s: int = 5
    divergence_sigma: float = 3.0
    divergence_hold_s: float = 5.0
    close_hysteresis_s: float = 10.0
    sol_stale_s: float = 5.0
    abs_ref_max_m: float = 0.2       # control-point deviation alarm threshold
    abs_ref_radius_m: float = 3.0    # how close to a control point counts as "at" it
    local_fallback: bool = False     # 仅话题缺失且显式开启时跑本地规则


@dataclass(frozen=True)
class ControlPoint:
    """A surveyed reference point with known WGS-84 coordinates (§4.2 absolute
    baseline verification). The vehicle passing/parking here lets the app compare
    its own solution against ground truth to catch a whole-mine coordinate shift."""
    name: str
    lat: float
    lon: float
    alt: float = 0.0


@dataclass(frozen=True)
class PublishCfg:
    enabled: bool = False
    host: str = "127.0.0.1"
    port: int = 15030


@dataclass(frozen=True)
class WebCfg:
    port: int = 8080
    tiles_path: str = ""
    host: str = "0.0.0.0"
    static_dir: str = ""


@dataclass(frozen=True)
class RosTopicCfg:
    """一条只读 ROS 订阅。kind 决定解码器，source 仅对 rtkfix 有意义。"""
    topic: str
    kind: str                      # rtkfix | stat | diagnostic | rtcm_rate
    qos: str = "auto"              # auto | reliable | best_effort
    source: str = ""               # app | fused（仅 rtkfix）
    enabled: bool = True


DEFAULT_ROS_TOPICS: tuple[RosTopicCfg, ...] = (
    RosTopicCfg("/rtkrcv_node/rtk_fix", "rtkfix", "reliable", "app"),
    RosTopicCfg("/gnss_cgi610/rtk_fix", "rtkfix", "auto", "fused"),
    RosTopicCfg("/rtkrcv_node/stat", "stat", "reliable"),
    RosTopicCfg("/gnss/diagnostics", "diagnostic", "auto"),
    RosTopicCfg("/rtkrcv_node/diagnostics", "diagnostic", "auto"),
    RosTopicCfg("/gnss/rtcm_corrections", "rtcm_rate", "auto"),
    RosTopicCfg("/gnss/raw_obs", "rtcm_rate", "auto"),
)


@dataclass(frozen=True)
class RosCfg:
    enabled: bool = True
    node_name: str = "gnss_hmi"
    use_sim_time: bool = False
    topics: tuple[RosTopicCfg, ...] = DEFAULT_ROS_TOPICS
    stale_s: float = 3.0           # 超过则该路“数据中断”
    startup_grace_s: float = 10.0  # 启动宽限，期内不告警无数据


@dataclass(frozen=True)
class StorageCfg:
    enabled: bool = True           # P0 只读模式关闭：不写 SQLite / 原始日志


@dataclass(frozen=True)
class TilesCfg:
    """在线 XYZ 底图 → 本地 MBTiles 缓存（plan Task 8）。"""
    enabled: bool = True
    source_name: str = "esri"
    source_url: str = ("https://server.arcgisonline.com/ArcGIS/rest/services/"
                       "World_Imagery/MapServer/tile/{z}/{y}/{x}")
    attribution: str = "Esri, Maxar, Earthstar Geographics"
    format: str = "jpg"
    cache_dir: str = ""            # 空则用 data_root/tiles
    bbox: tuple[float, float, float, float] | None = None   # min_lon,min_lat,max_lon,max_lat
    zoom_min: int = 13
    zoom_max: int = 16
    max_tiles: int = 20000
    timeout_s: float = 10.0
    warm_on_start: bool = False    # 启动时后台预热；失败只记日志，不影响只读链路


@dataclass(frozen=True)
class Config:
    data_root: Path
    db_path: Path
    corrections: StreamCfg
    raw_obs: StreamCfg
    gnss_solution: StreamCfg
    can_channel: str
    reserve_corrections_port: int
    reserve_raw_obs_port: int
    retention_days: int
    disk_watermark_pct: float
    rtkrcv: RtkrcvCfg
    diagnosis: DiagnosisCfg
    publish: PublishCfg
    web: WebCfg
    db_retention_days: int
    control_points: tuple[ControlPoint, ...] = ()
    ros: RosCfg = RosCfg()
    storage: StorageCfg = StorageCfg()
    can_enabled: bool = True
    tiles: TilesCfg = TilesCfg()


def _stream(d: dict) -> StreamCfg:
    return StreamCfg(host=d["host"], port=int(d["port"]), listen=bool(d.get("listen", False)),
                     enabled=bool(d.get("enabled", True)))


def _ros(raw: dict | None) -> RosCfg:
    raw = raw or {}
    topics = raw.get("topics")
    if topics is None:
        parsed = DEFAULT_ROS_TOPICS
    else:
        parsed = tuple(
            RosTopicCfg(topic=str(t["topic"]), kind=str(t["kind"]),
                        qos=str(t.get("qos", "auto")),
                        source=str(t.get("source", "")),
                        enabled=bool(t.get("enabled", True)))
            for t in topics)
    return RosCfg(enabled=bool(raw.get("enabled", True)),
                  node_name=str(raw.get("node_name", "gnss_hmi")),
                  use_sim_time=bool(raw.get("use_sim_time", False)),
                  topics=parsed,
                  stale_s=float(raw.get("stale_s", 3.0)),
                  startup_grace_s=float(raw.get("startup_grace_s", 10.0)))


def _tiles(raw: dict | None, data_root: Path) -> TilesCfg:
    t = raw or {}
    bbox = t.get("bbox")
    return TilesCfg(
        enabled=bool(t.get("enabled", True)),
        source_name=str(t.get("source_name", TilesCfg.source_name)),
        source_url=str(t.get("source_url", TilesCfg.source_url)),
        attribution=str(t.get("attribution", TilesCfg.attribution)),
        format=str(t.get("format", TilesCfg.format)),
        cache_dir=str(t.get("cache_dir") or (Path(data_root) / "tiles")),
        bbox=tuple(float(v) for v in bbox) if bbox else None,
        zoom_min=int(t.get("zoom_min", TilesCfg.zoom_min)),
        zoom_max=int(t.get("zoom_max", TilesCfg.zoom_max)),
        max_tiles=int(t.get("max_tiles", TilesCfg.max_tiles)),
        timeout_s=float(t.get("timeout_s", TilesCfg.timeout_s)),
        warm_on_start=bool(t.get("warm_on_start", False)),
    )


def load_config(path: str | Path) -> Config:
    raw = yaml.safe_load(Path(path).read_text())
    r = raw.get("rtkrcv") or {}
    d = raw.get("diagnosis") or {}
    p = raw.get("publish") or {}
    w = raw.get("web") or {}
    return Config(
        data_root=Path(raw["data_root"]),
        db_path=Path(raw["db_path"]),
        corrections=_stream(raw["corrections"]),
        raw_obs=_stream(raw["raw_obs"]),
        gnss_solution=_stream(raw["gnss_solution"]),
        can_channel=raw["can_channel"],
        reserve_corrections_port=int(raw["reserve"]["corrections_port"]),
        reserve_raw_obs_port=int(raw["reserve"]["raw_obs_port"]),
        retention_days=int(raw["retention_days"]),
        disk_watermark_pct=float(raw["disk_watermark_pct"]),
        rtkrcv=RtkrcvCfg(binary=str(r.get("binary", "")),
                         sol_port=int(r.get("sol_port", 15020)),
                         extra_args=tuple(r.get("extra_args", []))),
        diagnosis=DiagnosisCfg(**{k: type(getattr(DiagnosisCfg, k))(v)
                                  for k, v in d.items()}),
        publish=PublishCfg(enabled=bool(p.get("enabled", False)),
                           host=str(p.get("host", "127.0.0.1")),
                           port=int(p.get("port", 15030))),
        web=WebCfg(port=int(w.get("port", 8080)),
                   tiles_path=str(w.get("tiles_path", "")),
                   host=str(w.get("host", "0.0.0.0")),
                   static_dir=str(w.get("static_dir", ""))),
        db_retention_days=int(raw.get("db_retention_days", 30)),
        control_points=tuple(
            ControlPoint(name=str(cp.get("name", f"CP{i}")),
                         lat=float(cp["lat"]), lon=float(cp["lon"]),
                         alt=float(cp.get("alt", 0.0)))
            for i, cp in enumerate(raw.get("control_points") or [])),
        ros=_ros(raw.get("ros")),
        storage=StorageCfg(enabled=bool((raw.get("storage") or {}).get("enabled", True))),
        can_enabled=bool(raw.get("can_enabled", True)),
        tiles=_tiles(raw.get("tiles"), Path(raw["data_root"])),
    )
