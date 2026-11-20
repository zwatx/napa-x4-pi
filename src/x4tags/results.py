"""What one frame produces, in robot terms, ready to publish."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .detector import X4Tag, X4TagDetector
from .localize import FieldPose
from .mount import Mount, yaw_pitch_deg


@dataclass
class TagResult:
    tag_id: int
    yaw_deg: float  # robot frame: left of forward
    pitch_deg: float  # robot frame: above horizontal
    corners_px: np.ndarray  # (4, 2) in the stitched frame
    refined: bool
    seam: bool
    near: bool
    hamming: int | None
    decision_margin: float | None
    distance_m: float | None = None  # camera to tag centre (needs tags.size_m)
    position_m: np.ndarray | None = None  # tag centre in the robot frame
    rotation: np.ndarray | None = None  # tag's OpenCV axes in robot axes (3x3)
    pose_error_px: float | None = None

    def to_dict(self) -> dict:
        d = {"id": self.tag_id, "yaw_deg": round(self.yaw_deg, 3), "pitch_deg": round(self.pitch_deg, 3),
             "corners_px": np.round(self.corners_px, 2).tolist(), "refined": self.refined, "seam": self.seam,
             "near": self.near,
             "hamming": self.hamming,
             "margin": None if self.decision_margin is None else round(float(self.decision_margin), 1)}
        if self.position_m is not None:
            d.update(distance_m=round(self.distance_m, 4), position_m=np.round(self.position_m, 4).tolist(),
                     rotation=np.round(self.rotation, 5).tolist(), pose_error_px=round(self.pose_error_px, 3))
        return d


@dataclass
class FrameResult:
    seq: int
    capture_time: float  # time.monotonic() when the frame arrived from the camera
    width: int
    height: int
    decode_ms: float
    detect_ms: float
    tags: list[TagResult] = field(default_factory=list)
    field_pose: FieldPose | None = None

    def to_dict(self, now: float) -> dict:
        d = {"seq": self.seq, "age_ms": round((now - self.capture_time) * 1e3, 2), "decode_ms": round(self.decode_ms, 2),
             "detect_ms": round(self.detect_ms, 2), "size": [self.width, self.height],
             "tags": [t.to_dict() for t in self.tags]}
        if self.field_pose is not None:
            d["field_pose"] = self.field_pose.to_dict()
        return d


def tag_results(tags: list[X4Tag], mount: Mount, frame_height: int, size_m: float | None, max_pose_error_px: float,
                ids: list[int] | None = None) -> list[TagResult]:
    out = []
    for t in tags:
        if ids is not None and t.tag_id not in ids:
            continue
        yaw, pitch = yaw_pitch_deg(mount.direction(t.center_ray))
        r = TagResult(t.tag_id, yaw, pitch, t.corners, t.refined, t.seam, t.near, t.hamming, t.decision_margin)
        if size_m:
            pose = X4TagDetector.pose(t, size_m, frame_height)
            if pose is not None and pose[2] <= max_pose_error_px:
                rot, pos = mount.pose(pose[0], pose[1])
                r.distance_m, r.position_m, r.rotation, r.pose_error_px = float(np.linalg.norm(pose[1])), pos, rot, pose[2]
        out.append(r)
    return out
