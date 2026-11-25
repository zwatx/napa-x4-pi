"""AprilTag field layouts in WPILib's JSON format, and tag corners in the field frame.

WPILib's tag frame: origin at the tag's centre, +X out of the printed face (towards whoever looks at it), +Y to
that viewer's right, +Z up. A tag hangs upright as printed, so in the tag frame its printed corners are
bottom-right (0, +s/2, -s/2), bottom-left (0, -s/2, -s/2), top-left (0, -s/2, +s/2), top-right (0, +s/2, +s/2).
NAPA's corners come in that order (corner 0 is the printed bottom-right; upstream AprilTag's p[1], p[0], p[3], p[2]).
"""

from __future__ import annotations

import json
from dataclasses import dataclass

import numpy as np

# NAPA's corner order in WPILib tag coordinates, per unit side (multiply by the tag's black-square side).
NAPA_CORNERS_IN_TAG = np.array([[0.0, 0.5, -0.5], [0.0, -0.5, -0.5], [0.0, -0.5, 0.5], [0.0, 0.5, 0.5]])

# OpenCV/NAPA tag axes (x to the printed left, y to the printed bottom, z out of the face) as columns in WPILib tag
# axes: WPILib X = OpenCV z, Y = -OpenCV x, Z = -OpenCV y.
OPENCV_TAG_TO_WPILIB_TAG = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def quaternion_to_matrix(w: float, x: float, y: float, z: float) -> np.ndarray:
    n = np.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quaternion(r: np.ndarray) -> np.ndarray:
    """(w, x, y, z) of a rotation matrix, w >= 0."""
    m = np.asarray(r, float)
    t = np.trace(m)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        q = [0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
    else:
        i = int(np.argmax(np.diag(m)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = np.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k]) * 2
        q = [0.0, 0.0, 0.0, 0.0]
        q[0] = (m[k, j] - m[j, k]) / s
        q[1 + i] = 0.25 * s
        q[1 + j] = (m[j, i] + m[i, j]) / s
        q[1 + k] = (m[k, i] + m[i, k]) / s
    q = np.array(q)
    return q if q[0] >= 0 else -q


def matrix_to_rpy(r: np.ndarray) -> tuple[float, float, float]:
    """WPILib Rotation3d (roll, pitch, yaw) in radians of R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    pitch = float(np.arcsin(np.clip(-r[2, 0], -1.0, 1.0)))
    return float(np.arctan2(r[2, 1], r[2, 2])), pitch, float(np.arctan2(r[1, 0], r[0, 0]))


@dataclass
class FieldTag:
    tag_id: int
    position: np.ndarray  # field frame, metres
    rotation: np.ndarray  # tag axes (WPILib tag frame) as columns in field axes

    def corners(self, size_m: float) -> np.ndarray:
        """(4, 3) field coordinates of the black square's corners, in NAPA's order."""
        return self.position + (NAPA_CORNERS_IN_TAG * size_m) @ self.rotation.T


class FieldLayout:
    def __init__(self, tags: dict[int, FieldTag], length_m: float | None = None, width_m: float | None = None):
        self.tags, self.length_m, self.width_m = tags, length_m, width_m

    @classmethod
    def from_dict(cls, data: dict) -> FieldLayout:
        """From WPILib's layout JSON (``{"tags": [{"ID", "pose": {"translation", "rotation": {"quaternion"}}}],
        "field": {"length", "width"}}``). ValueError, saying where, when it doesn't look like one."""
        tags = {}
        if not isinstance(data, dict) or not isinstance(data.get("tags"), list):
            raise ValueError('not a WPILib AprilTag layout: expected {"tags": [...], "field": {...}}')
        for k, t in enumerate(data["tags"]):
            try:
                tid = int(t["ID"])
                tr, q = t["pose"]["translation"], t["pose"]["rotation"]["quaternion"]
                tag = FieldTag(tid, np.array([tr["x"], tr["y"], tr["z"]], float),
                               quaternion_to_matrix(q["W"], q["X"], q["Y"], q["Z"]))
            except (KeyError, TypeError, ValueError) as e:
                raise ValueError(f"tag {k} in the layout: missing or bad {e}") from e
            if tid in tags:
                raise ValueError(f"tag ID {tid} is in the layout twice")
            tags[tid] = tag
        if not tags:
            raise ValueError("the layout has no tags")
        field = data.get("field", {})
        return cls(tags, field.get("length"), field.get("width"))

    @classmethod
    def load(cls, path: str) -> FieldLayout:
        with open(path) as f:
            return cls.from_dict(json.load(f))

    def __contains__(self, tag_id: int) -> bool:
        return tag_id in self.tags

    def __getitem__(self, tag_id: int) -> FieldTag:
        return self.tags[tag_id]
