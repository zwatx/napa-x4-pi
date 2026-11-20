"""Settings, from a TOML file (see config.example.toml). Every key is optional; unknown keys are errors."""

from __future__ import annotations

import tomllib
from dataclasses import dataclass, field, fields

from .mount import Mount


@dataclass
class CameraConfig:
    device: str | None = None  # None: find the X4 by its USB vendor ID
    width: int = 2880
    height: int = 1440
    fps: int = 30
    reduce: int = 1  # 2: libjpeg half-size decode (1440x720), cheaper but halves the range
    reconnect_s: float = 2.0  # wait between attempts when the camera is missing or stalls


@dataclass
class DetectorConfig:
    min_tag_px: int = 22
    threads: int = 1  # NAPA threads per call (Pi 5: up to 4 for lower latency)
    refine_bow_px: float | None = 0.2  # refine tags whose sides the frame bends more than this; None: never
    seam_px: int = 192  # 0: don't search across the left/right seam
    family: str = "tag36h11"
    extra: dict = field(default_factory=dict)  # any other NapaDetector keyword


@dataclass
class TagsConfig:
    size_m: float | None = 0.1651  # black-square side; None: no pose, bearings only
    max_pose_error_px: float = 4.0  # drop poses whose corners reproject worse than this
    ids: list[int] | None = None  # only report these IDs (None: all)


@dataclass
class OutputConfig:
    print: bool = False  # one JSON line per frame on stdout
    udp: str | None = None  # "host:port": one JSON datagram per frame
    nt_server: str | None = None  # NetworkTables 4 server: "10.TE.AM.2", a team number like "1234", or "localhost"
    nt_table: str = "x4tags"
    nt_client_name: str = "x4tags"
    stats_every_s: float = 10.0  # log timing and health this often (0: never)


@dataclass
class Config:
    camera: CameraConfig = field(default_factory=CameraConfig)
    detector: DetectorConfig = field(default_factory=DetectorConfig)
    tags: TagsConfig = field(default_factory=TagsConfig)
    mount: Mount = field(default_factory=Mount)
    output: OutputConfig = field(default_factory=OutputConfig)


def _build(cls, data: dict, where: str):
    names = {f.name for f in fields(cls)}
    unknown = set(data) - names
    if unknown:
        raise ValueError(f"unknown key(s) in [{where}]: {', '.join(sorted(unknown))}")
    return cls(**data)


def load(path: str | None) -> Config:
    if path is None:
        return Config()
    with open(path, "rb") as f:
        data = tomllib.load(f)
    sections = {"camera": CameraConfig, "detector": DetectorConfig, "tags": TagsConfig, "mount": Mount,
                "output": OutputConfig}
    unknown = set(data) - set(sections)
    if unknown:
        raise ValueError(f"unknown section(s): {', '.join(sorted(unknown))}")
    built = {name: _build(cls, data.get(name, {}), name) for name, cls in sections.items()}
    # TOML has no null: "none"/"off" in a few optional values means unset.
    for obj, key in ((built["detector"], "refine_bow_px"), (built["tags"], "size_m"), (built["camera"], "device")):
        if isinstance(getattr(obj, key), str) and getattr(obj, key).lower() in ("none", "off", "auto"):
            setattr(obj, key, None)
    return Config(**built)
