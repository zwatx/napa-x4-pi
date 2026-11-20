"""Camera (and robot) pose on the field from every visible tag at once.

A 360-degree camera sees tags all around, which pins its position down far better than any single tag: the solve
fits one camera pose to the directions of all tag corners (angles on the sphere, so it works across the whole
picture, where a pinhole camera model can't). It starts from each tag's own pose and keeps the best fit.

Position comes out right even if the X4 rotates its picture (it stabilizes webcam output in the camera): the
picture turns about the lens centre, which doesn't move it. The solved rotation is the picture's, so robot heading
from it is only right if the picture stays fixed to the camera body; otherwise take heading from the robot's gyro
and use ``camera_position`` (see ``robot_position_from_heading``).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .detector import X4Tag, X4TagDetector
from .equirect import tangent_bases
from .field import OPENCV_TAG_TO_WPILIB_TAG, FieldLayout, matrix_to_rpy
from .mount import Mount, rotation_rpy


@dataclass
class FieldPose:
    camera_position: np.ndarray  # field frame, metres
    camera_rotation: np.ndarray  # camera axes (x right, y down, z forward) as columns in field axes
    robot_position: np.ndarray  # robot origin in the field, through the mount (assumes the picture is body-fixed)
    robot_rotation: np.ndarray  # robot axes as columns in field axes (same assumption)
    rms_px: float  # corner direction error, in pixels at the frame's scale
    tag_ids: list[int]

    @property
    def robot_rpy_deg(self) -> tuple[float, float, float]:
        return tuple(float(np.degrees(a)) for a in matrix_to_rpy(self.robot_rotation))

    def to_dict(self) -> dict:
        roll, pitch, yaw = self.robot_rpy_deg
        return {"robot_xyz_m": np.round(self.robot_position, 4).tolist(), "robot_rpy_deg": [round(roll, 3), round(pitch, 3), round(yaw, 3)],
                "camera_xyz_m": np.round(self.camera_position, 4).tolist(), "rms_px": round(self.rms_px, 3), "tags": self.tag_ids}


def robot_position_from_heading(camera_position: np.ndarray, robot_yaw_rad: float, mount: Mount) -> np.ndarray:
    """Robot origin on the field from the camera's solved position and a heading from elsewhere (the robot's gyro),
    for a robot driving level: use this if the camera's picture isn't fixed to its body."""
    return camera_position - rotation_rpy(0.0, 0.0, robot_yaw_rad) @ mount.translation


def _rodrigues(w: np.ndarray) -> np.ndarray:
    theta = float(np.linalg.norm(w))
    if theta < 1e-12:
        return np.eye(3)
    k = w / theta
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(theta) * kx + (1 - np.cos(theta)) * kx @ kx


class FieldLocalizer:
    """``solve(tags)`` -> FieldPose or None, for tags from X4TagDetector.

    ``max_rms_px``: reject a solution whose corner directions fit worse than this (after dropping the worst tag
    once, when there are 3 or more). ``frame_height``: of the stitched frame, to express errors in its pixels.
    """

    def __init__(self, layout: FieldLayout, tag_size_m: float, mount: Mount = Mount(), max_rms_px: float = 3.0,
                 frame_height: int = 1440, huber_px: float = 1.5):
        self.layout, self.size, self.mount = layout, float(tag_size_m), mount
        self.max_rms_px, self.px_per_rad, self.huber = max_rms_px, frame_height / np.pi, huber_px / (frame_height / np.pi)

    def solve(self, tags: list[X4Tag]) -> FieldPose | None:
        tags = [t for t in tags if t.tag_id in self.layout]
        if not tags:
            return None
        pose = self._solve(tags)
        if pose is not None and pose[2] * self.px_per_rad > self.max_rms_px and len(tags) >= 3:
            worst = int(np.argmax(pose[3]))  # per-tag error: drop the worst tag once and solve again
            tags = tags[:worst] + tags[worst + 1:]
            pose = self._solve(tags)
        if pose is None or pose[2] * self.px_per_rad > self.max_rms_px:
            return None
        r_fc, c, rms, _ = pose
        r_rc = self.mount.rotation  # camera axes in robot axes
        r_fr = r_fc @ r_rc.T
        return FieldPose(c, r_fc, c - r_fr @ self.mount.translation, r_fr, rms * self.px_per_rad, [t.tag_id for t in tags])

    # -- internals
    def _solve(self, tags):
        points = np.concatenate([self.layout[t.tag_id].corners(self.size) for t in tags])  # (4n, 3) field
        rays = np.concatenate([t.rays for t in tags])
        rays = rays / np.linalg.norm(rays, axis=1, keepdims=True)
        bases = tangent_bases(rays)[:, :2]  # (4n, 2, 3): directions across each observed ray
        best = None
        for start in self._starts(tags):
            fit = self._refine(start, points, rays, bases)
            if fit is not None and (best is None or fit[2] < best[2]):
                best = fit
            if best is not None and best[2] * self.px_per_rad < 1.0:
                break  # a good fit from the biggest tag's start: the other start won't do better
        if best is None:
            return None
        r, c, rms, res = best
        per_tag = np.sqrt((res.reshape(len(tags), 4, 2) ** 2).sum(2).mean(1))
        return r, c, rms, per_tag

    def _starts(self, tags):
        """Camera poses from each tag's own pose (largest tags first, which are the most reliable)."""
        out = []
        for t in sorted(tags, key=lambda t: -np.linalg.norm(t.corners[0] - t.corners[2]))[:2]:
            pose = X4TagDetector.pose(t, self.size)
            if pose is None:
                continue
            r_ct = pose[0] @ OPENCV_TAG_TO_WPILIB_TAG.T  # WPILib tag axes as columns in camera axes
            ft = self.layout[t.tag_id]
            r_fc = ft.rotation @ r_ct.T
            out.append((r_fc, ft.position - r_fc @ pose[1]))
        return out

    def _residuals(self, r, c, points, rays, bases, jacobian=False):
        """Residuals (~ angle across each observed ray, radians, 2 per corner) and optionally their Jacobian with
        respect to (rotation update w, applied as R @ exp(w), position update)."""
        v = (points - c) @ r  # field -> camera: R^T (P - c), row-wise
        n = np.maximum(np.linalg.norm(v, axis=1, keepdims=True), 1e-9)
        u = v / n
        res = np.einsum("kij,kj->ki", bases, u - rays).ravel()
        if not jacobian:
            return res
        # dv/dw = [v]x (R' = R exp(w) => v' = exp(-w) v ~ v - w x v), dv/dc = -R^T; du/dv = (I - u u^T) / |v|
        vx = np.zeros((len(v), 3, 3))
        vx[:, 0, 1], vx[:, 0, 2], vx[:, 1, 2] = -v[:, 2], v[:, 1], -v[:, 0]
        vx[:, 1, 0], vx[:, 2, 0], vx[:, 2, 1] = v[:, 2], -v[:, 1], v[:, 0]
        dv = np.concatenate([vx, np.broadcast_to(-r.T, (len(v), 3, 3))], axis=2)  # (k, 3, 6)
        du = (np.eye(3) - u[:, :, None] * u[:, None, :]) / n[:, :, None]
        return res, np.einsum("kij,kjl,klm->kim", bases, du, dv).reshape(-1, 6)

    def _refine(self, start, points, rays, bases, iterations: int = 20):
        r, c = start
        lam = 1e-3
        res, jac = self._residuals(r, c, points, rays, bases, jacobian=True)
        cost = self._cost(res)
        for _ in range(iterations):
            w = self._weights(res)
            a = jac.T @ (jac * w[:, None])
            g = jac.T @ (w * res)
            while True:
                step = np.linalg.solve(a + lam * np.diag(np.diag(a) + 1e-12), -g)
                r_new, c_new = r @ _rodrigues(step[:3]), c + step[3:]
                res_new = self._residuals(r_new, c_new, points, rays, bases)
                cost_new = self._cost(res_new)
                if cost_new <= cost:
                    r, c, cost, lam = r_new, c_new, cost_new, max(lam / 10, 1e-9)
                    res, jac = self._residuals(r, c, points, rays, bases, jacobian=True)
                    break
                lam *= 10
                if lam > 1e8:
                    break
            if lam > 1e8 or np.linalg.norm(step) < 1e-7:  # 1e-7 rad / 0.1 micrometre: converged
                break
        u, _, vt = np.linalg.svd(r)  # keep it a rotation
        r = u @ vt
        return r, c, float(np.sqrt(np.mean(res ** 2) * 2)), res  # rms angle per corner (2 components)

    def _weights(self, res):
        a = np.abs(res)
        return np.where(a <= self.huber, 1.0, self.huber / np.maximum(a, 1e-12))

    def _cost(self, res):
        a = np.abs(res)
        return float(np.sum(np.where(a <= self.huber, 0.5 * a * a, self.huber * (a - 0.5 * self.huber))))
