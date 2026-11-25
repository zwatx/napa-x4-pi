"""Settings, from a TOML file (see config.example.toml). Every key is optional; unknown keys are errors."""

from __future__ import annotations

import tomllib
from dataclasses import dataclass, fields
from dataclasses import field as _field  # a section is called `field`

from .mount import Mount


MODE_SIZES = {"equirect": (2880, 1440), "dual1080": (1920, 1080)}
# Measured on one X4 with check_latency (median 154-163 ms and 133 ms), less ~20 ms for the screen's own delay.
# Measured on one X4 as 140 and 115 ms to when read() returned a frame; frame times now come from V4L2, when the
# frame's USB transfer began, 5.5 and 3 ms earlier on a Pi 5.
MODE_LATENCY_MS = {"equirect": 135.0, "dual1080": 112.0}


@dataclass
class CameraConfig:
    device: str | None = None  # None: find the X4 by its USB vendor ID
    # "equirect": the 360-degree 2880x1440 picture. "dual1080": the 1920x1080 picture, front and back views stacked
    # (1.3-1.8x the pixels per degree and ~20 ms less latency, but +-76 degrees each side and +-24 up/down).
    mode: str = "equirect"
    width: int | None = None  # None: the mode's size
    height: int | None = None
    fps: int = 30
    reduce: int = 1  # 2: libjpeg half-size decode (1440x720), cheaper but halves the range
    reconnect_s: float = 2.0  # wait between attempts when the camera is missing or stalls
    # Time from the camera taking a frame to the frame arriving here (stitch, encode, USB), subtracted from every
    # timestamp. None: the mode's value measured on one X4 (MODE_LATENCY_MS); check yours with check_latency.
    latency_ms: float | None = None

    def size(self) -> tuple[int, int]:
        default = MODE_SIZES[self.mode]
        return (self.width or default[0], self.height or default[1])

    def latency(self) -> float:
        """Camera latency in ms: the configured value, or the mode's measured default."""
        return MODE_LATENCY_MS[self.mode] if self.latency_ms is None else float(self.latency_ms)


@dataclass
class DetectorConfig:
    min_tag_px: int = 16  # NAPA's own default is 22: too big for field tags across the 360-degree picture
    threads: int = 1  # NAPA threads per search (Pi 5: more don't help in equirect, 2 save ~0.6 ms in dual1080)
    refine_bow_px: float | None = 0.2  # refine tags whose sides the frame bends more than this; None: never
    seam_px: int = 192  # 0: don't search across the left/right seam
    near_scale: float = 1 / 3  # straight views for close, strongly bent tags, at this fraction of the resolution; 0: off
    near_per_frame: int = 2  # of the 12 straight views, search this many per frame in rotation (0: all every frame)
    family: str = "tag36h11"
    # The gamma the X4 encodes its picture with, undone before fitting tag edges (NapaDetector's camera_gamma). NAPA's
    # estimate_camera_gamma read 1.7-2.4 from X4 clips; 2.0 cut corner error 36% on rendered frames and 10% against a
    # screen's exact geometry, at no cost. None: treat the picture as linear.
    camera_gamma: float | None = 2.0
    extra: dict = _field(default_factory=dict)  # any other NapaDetector keyword


@dataclass
class TagsConfig:
    size_m: float | None = 0.1651  # black-square side; None: no pose, bearings only
    max_pose_error_px: float = 4.0  # drop poses whose corners reproject worse than this
    ids: list[int] | None = None  # only report these IDs (None: all)


@dataclass
class FieldConfig:
    layout: str | None = None  # WPILib AprilTag field layout JSON; set it to get a field pose from all tags
    max_rms_px: float = 1.5  # every tag in a field pose must fit within this (pixels); others are left out
    min_tags: int = 2  # fewer tags give no field pose (one tag's pose is ambiguous: two fits, metres apart)
    corner_noise_px: float = 0.5  # assumed corner error, for the published expected pose error
    max_xy_std_m: float = 1.0  # give no field pose when its expected position error is larger


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
    camera: CameraConfig = _field(default_factory=CameraConfig)
    detector: DetectorConfig = _field(default_factory=DetectorConfig)
    tags: TagsConfig = _field(default_factory=TagsConfig)
    mount: Mount = _field(default_factory=Mount)
    field: FieldConfig = _field(default_factory=FieldConfig)
    output: OutputConfig = _field(default_factory=OutputConfig)


_NUMBER = (int, float)
_TYPES = {
    "camera": dict(device=(str,), mode=(str,), width=(int,), height=(int,), fps=(int,), reduce=(int,), reconnect_s=_NUMBER,
                   latency_ms=_NUMBER),
    "detector": dict(min_tag_px=(int,), threads=(int,), refine_bow_px=_NUMBER + (str,), seam_px=(int,),
                     camera_gamma=_NUMBER + (str,),
                     near_scale=_NUMBER, near_per_frame=(int,), family=(str,), extra=(dict,)),
    "tags": dict(size_m=_NUMBER + (str,), max_pose_error_px=_NUMBER, ids=(list,)),
    "mount": {k: _NUMBER for k in ("x_m", "y_m", "z_m", "roll_deg", "pitch_deg", "yaw_deg")},
    "field": dict(layout=(str,), max_rms_px=_NUMBER, min_tags=(int,), corner_noise_px=_NUMBER, max_xy_std_m=_NUMBER),
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
                "field": FieldConfig, "output": OutputConfig}
    unknown = set(data) - set(sections)
    if unknown:
        raise ValueError(f"unknown section(s): {', '.join(sorted(unknown))}")
    built = {name: _build(cls, data.get(name, {}), name) for name, cls in sections.items()}
    # TOML has no null: "none"/"off" means unset for these, "auto" means find the camera.
    for obj, key, words in ((built["detector"], "refine_bow_px", ("none", "off")), (built["tags"], "size_m", ("none", "off")),
                            (built["detector"], "camera_gamma", ("none", "off")),
                            (built["camera"], "device", ("auto",))):
        value = getattr(obj, key)
        if isinstance(value, str):
            if value.lower() not in words and key != "device":
                raise ValueError(f"{key} = {value!r}: expected a number or {' / '.join(repr(w) for w in words)}")
            if value.lower() in words:
                setattr(obj, key, None)
    if built["camera"].mode not in MODE_SIZES:
        raise ValueError(f"[camera] mode = {built['camera'].mode!r}: expected {' or '.join(map(repr, MODE_SIZES))}")
    cam = built["camera"]
    if (cam.width is None) != (cam.height is None):
        raise ValueError("[camera] width and height: set both or neither (neither: the mode's size)")
    want = MODE_SIZES[cam.mode]
    if cam.width is not None and cam.width * want[1] != cam.height * want[0]:
        raise ValueError(f"[camera] {cam.width}x{cam.height} isn't the shape of mode {cam.mode!r} ({want[0]}x{want[1]}): "
                         f"remove width/height, or set the mode")
    if built["camera"].reduce not in (1, 2, 4):
        raise ValueError(f"[camera] reduce = {built['camera'].reduce}: expected 1, 2 or 4")
    if built["field"].layout is not None and built["tags"].size_m is None:
        raise ValueError("[field] layout needs [tags] size_m")
    if built["output"].nt_server is not None:
        built["output"].nt_server = str(built["output"].nt_server)  # a bare team number is fine
    return Config(**built)
