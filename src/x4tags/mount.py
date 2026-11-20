"""Where the camera sits on the robot: camera-frame directions and poses -> WPILib robot frame.

Camera frame (OpenCV): x right, y down, z forward, where "forward" is the middle of the stitched X4 picture (the
direction the camera's front lens faces, unless the stitch is re-centred in the camera app).
Robot frame (WPILib): x forward, y left, z up, origin wherever the robot code puts it (usually the robot's centre
on the floor).

The mount is the camera's position on the robot and its rotation as WPILib's ``Rotation3d(roll, pitch, yaw)``:
roll about x, then pitch about y, then yaw about z, all about the robot's axes. Zero rotation means the picture's
middle looks along the robot's +x with the picture upright. Positive pitch tips the camera's forward view down,
positive yaw turns it left, as in WPILib.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

# Camera axes in robot terms for a level camera looking forward: camera +z (forward) = robot +x,
# camera +x (right) = robot -y, camera +y (down) = robot -z.
CAMERA_TO_ROBOT_AXES = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def rotation_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """WPILib Rotation3d(roll, pitch, yaw) as a matrix (radians): Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


@dataclass(frozen=True)
class Mount:
    x_m: float = 0.0
    y_m: float = 0.0
    z_m: float = 0.0
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0

    @property
    def rotation(self) -> np.ndarray:
        """Robot-from-camera rotation (camera-frame vectors -> robot-frame vectors)."""
        r = rotation_rpy(math.radians(self.roll_deg), math.radians(self.pitch_deg), math.radians(self.yaw_deg))
        return r @ CAMERA_TO_ROBOT_AXES

    @property
    def translation(self) -> np.ndarray:
        return np.array([self.x_m, self.y_m, self.z_m])

    def direction(self, camera_ray: np.ndarray) -> np.ndarray:
        """A camera-frame direction in robot axes (no translation)."""
        return self.rotation @ np.asarray(camera_ray, float)

    def point(self, camera_point: np.ndarray) -> np.ndarray:
        """A camera-frame point (metres) in the robot frame."""
        return self.rotation @ np.asarray(camera_point, float) + self.translation

    def pose(self, rotation_cam: np.ndarray, translation_cam: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """A tag pose from ``X4TagDetector.pose`` (OpenCV tag axes, camera frame) in the robot frame.

        The rotation's columns are the tag's own OpenCV axes (x right, y down as printed, z into the tag) expressed
        in robot axes."""
        return self.rotation @ rotation_cam, self.point(translation_cam)


def yaw_pitch_deg(direction_robot: np.ndarray) -> tuple[float, float]:
    """(yaw left of robot forward, pitch above the robot's horizontal) of a robot-frame direction, degrees."""
    x, y, z = direction_robot
    return math.degrees(math.atan2(y, x)), math.degrees(math.atan2(z, math.hypot(x, y)))
