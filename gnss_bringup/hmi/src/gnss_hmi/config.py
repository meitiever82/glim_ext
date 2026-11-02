"""Load and validate config.yaml into typed dataclasses."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass(frozen=True)
class StreamCfg:
    host: str = "127.0.0.1"
    port: int = 0
    listen: bool = False
    enabled: bool = False


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
    local_fallback: bool = True      # run old local rules only when ROS diag is absent


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
class StorageCfg:
    enabled: bool = True


@dataclass(frozen=True)
class CanCfg:
    enabled: bool = False
    channel: str = "can0"


@dataclass(frozen=True)
class RosTopicCfg:
    topic: str
    kind: str  # "rtkfix" | "stat" | "diagnostic" | "rate" | "gga" | "navsat" | "bestpos"
    qos: str = "auto"          # "auto" | "reliable" | "best_effort"
    source: str = ""           # rtkfix/diagnostic: "app"|"fused" / "gnss"|"rtkrcv"
    name: str = ""             # rate: "corr"|"obs"


@dataclass(frozen=True)
class RosCfg:
    enabled: bool = False
    topics: tuple[RosTopicCfg, ...] = ()
    timeout_warn_s: float = 5.0


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
    can: CanCfg = CanCfg()


def _stream(d: dict) -> StreamCfg:
    return StreamCfg(host=d.get("host", "127.0.0.1"), port=int(d.get("port", 0)),
                     listen=bool(d.get("listen", False)),
                     enabled=bool(d.get("enabled", True)))


def load_config(path: str | Path) -> Config:
    raw = yaml.safe_load(Path(path).read_text())
    r = raw.get("rtkrcv") or {}
    d = raw.get("diagnosis") or {}
    p = raw.get("publish") or {}
    w = raw.get("web") or {}
    s = raw.get("storage") or {}
    c = raw.get("can") or {}
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
        ros=_ros(raw.get("ros") or {}),
        storage=StorageCfg(enabled=bool(s.get("enabled", True))),
        can=CanCfg(enabled=bool(c.get("enabled", True)),
                   channel=str(c.get("channel", raw["can_channel"]))),
    )


def _ros(d: dict) -> RosCfg:
    return RosCfg(
        enabled=bool(d.get("enabled", False)),
        timeout_warn_s=float(d.get("timeout_warn_s", 5.0)),
        topics=tuple(
            RosTopicCfg(topic=str(t["topic"]), kind=str(t["kind"]),
                        qos=str(t.get("qos", "auto")),
                        source=str(t.get("source", "")),
                        name=str(t.get("name", "")))
            for t in (d.get("topics") or [])))
