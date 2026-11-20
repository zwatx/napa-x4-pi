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
    # Time from the camera taking a frame to the frame arriving here (stitch, encode, USB). Subtracted from every
    # timestamp. Unknown until measured on your setup (e.g. film a running stopwatch); 0 assumes none.
    latency_ms: float = 0.0


@dataclass
class DetectorConfig:
    min_tag_px: int = 22
    threads: int = 1  # NAPA threads per call (Pi 5: 4 saves only ~1 ms; JPEG decode dominates)
    refine_bow_px: float | None = 0.2  # refine tags whose sides the frame bends more than this; None: never
    seam_px: int = 192  # 0: don't search across the left/right seam
    near_scale: float = 1 / 3  # straight views for close, strongly bent tags, at this fraction of the resolution; 0: off
    near_per_frame: int = 2  # of the 12 straight views, search this many per frame in rotation (0: all every frame)
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


_NUMBER = (int, float)
_TYPES = {
    "camera": dict(device=(str,), width=(int,), height=(int,), fps=(int,), reduce=(int,), reconnect_s=_NUMBER,
                   latency_ms=_NUMBER),
    "detector": dict(min_tag_px=(int,), threads=(int,), refine_bow_px=_NUMBER + (str,), seam_px=(int,),
                     near_scale=_NUMBER, near_per_frame=(int,), family=(str,), extra=(dict,)),
    "tags": dict(size_m=_NUMBER + (str,), max_pose_error_px=_NUMBER, ids=(list,)),
    "mount": {k: _NUMBER for k in ("x_m", "y_m", "z_m", "roll_deg", "pitch_deg", "yaw_deg")},
    "output": dict(print=(bool,), udp=(str,), nt_server=(str, int), nt_table=(str,), nt_client_name=(str,),
                   stats_every_s=_NUMBER),
}


def _build(cls, data: dict, where: str):
    names = {f.name for f in fields(cls)}
    unknown = set(data) - names
    if unknown:
        raise ValueError(f"unknown key(s) in [{where}]: {', '.join(sorted(unknown))}")
    for key, value in data.items():
        allowed = _TYPES[where][key]
        if isinstance(value, bool) and bool not in allowed or not isinstance(value, allowed):
            raise ValueError(f"[{where}] {key} = {value!r}: expected {' or '.join(t.__name__ for t in allowed)}")
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
    # TOML has no null: "none"/"off" means unset for these, "auto" means find the camera.
    for obj, key, words in ((built["detector"], "refine_bow_px", ("none", "off")), (built["tags"], "size_m", ("none", "off")),
                            (built["camera"], "device", ("auto",))):
        value = getattr(obj, key)
        if isinstance(value, str):
            if value.lower() not in words and key != "device":
                raise ValueError(f"{key} = {value!r}: expected a number or {' / '.join(repr(w) for w in words)}")
            if value.lower() in words:
                setattr(obj, key, None)
    if built["camera"].reduce not in (1, 2, 4):
        raise ValueError(f"[camera] reduce = {built['camera'].reduce}: expected 1, 2 or 4")
    if built["output"].nt_server is not None:
        built["output"].nt_server = str(built["output"].nt_server)  # a bare team number is fine
    return Config(**built)
