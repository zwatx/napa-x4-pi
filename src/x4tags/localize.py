"""Camera (and robot) pose on the field from every visible tag at once.

A 360-degree camera sees tags all around, which pins its position down far better than any single tag: the solve
fits one camera pose to the directions of all tag corners (angles on the sphere, so it works across the whole
picture, where a pinhole camera model can't). It starts from each tag's own pose and keeps the best fit.

Position comes out right even though the X4 rotates its picture (it stabilizes webcam output in the camera): the
picture turns about the lens centre, which doesn't move it. The solved rotation is the picture's, which lags the
body by a few degrees for several seconds after a quick turn (measured: 3-8 degrees, ~5 s time constant; see
``check_stabilization``), so take heading from the robot's gyro and use ``camera_position`` with it (see
``robot_position_from_heading``).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from napa_vision import PoseWithCovariance
from napa_vision.covariance import cap_correlations

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
    xy_std_m: float = float("nan")  # expected camera position error, in its least certain direction
    yaw_std_deg: float = float("nan")  # expected heading error
    covariance: np.ndarray | None = None  # 6x6: field-frame rotation (rad) then camera position (m)
    robot_covariance: np.ndarray | None = None  # 3x3: robot (x, y, yaw) on the field, m and rad
    camera_xy_covariance: np.ndarray | None = None  # 2x2: camera (x, y) on the field, m

    @property
    def robot_rpy_deg(self) -> tuple[float, float, float]:
        return tuple(float(np.degrees(a)) for a in matrix_to_rpy(self.robot_rotation))

    def gyro_std_devs(self) -> tuple[float, float]:
        """Standard deviations (m) of the camera's position along the robot's forward and left axes: the x and y
        standard deviations for WPILib's ``addVisionMeasurement`` when the heading comes from the gyro. Its estimators
        apply them along the robot's axes, not the field's. The picture's heading is used to turn them; a few degrees
        of lag changes them very little."""
        if self.camera_xy_covariance is None:
            return float("nan"), float("nan")
        yaw = matrix_to_rpy(self.robot_rotation)[2]
        c, s = np.cos(yaw), np.sin(yaw)
        to_robot = np.array([[c, s], [-s, c]])
        var = np.diag(to_robot @ self.camera_xy_covariance @ to_robot.T)
        return float(np.sqrt(var[0])), float(np.sqrt(var[1]))

    def to_dict(self) -> dict:
        roll, pitch, yaw = self.robot_rpy_deg
        d = {"robot_xyz_m": np.round(self.robot_position, 4).tolist(), "robot_rpy_deg": [round(roll, 3), round(pitch, 3), round(yaw, 3)],
             "camera_xyz_m": np.round(self.camera_position, 4).tolist(), "rms_px": round(self.rms_px, 3), "tags": self.tag_ids,
             "xy_std_m": round(self.xy_std_m, 4), "yaw_std_deg": round(self.yaw_std_deg, 3)}
        if self.robot_covariance is not None:
            d["robot_covariance"] = [[float(f"{x:.6g}") for x in row] for row in self.robot_covariance]
            d["camera_xy_covariance"] = [[float(f"{x:.6g}") for x in row] for row in self.camera_xy_covariance]
            d["gyro_std_devs_m"] = [round(x, 5) for x in self.gyro_std_devs()]
        return d


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
    """``solve(tags, frame_height)`` -> FieldPose or None, for tags from X4TagDetector.

    ``min_tags``: fewer tags give no pose. One tag's pose has two nearly equally good solutions, metres apart, that
    its fit error can't tell apart, so the default is 2.
    ``max_rms_px``: a pose is accepted only if every tag's corners fit within this (pixels of the frame). When some
    don't, each tag is left out in turn (up to 8 tags) and the best fit without it is kept, if it still has
    ``min_tags`` tags and all of them fit: one misread or misplaced tag can't pull the pose, or hide in it.
    ``corner_noise_px``: assumed corner error, for the pose's covariances and expected errors. A good fit can still be
    weak, e.g. two small, distant tags in nearly the same direction: these say how much to trust it.
    ``max_xy_std_m``: give no pose when the expected position error is larger than this.
    """

    def __init__(self, layout: FieldLayout, tag_size_m: float, mount: Mount = Mount(), max_rms_px: float = 1.5,
                 min_tags: int = 2, huber_px: float = 1.0, corner_noise_px: float = 0.5, max_xy_std_m: float = 1.0):
        self.layout, self.size, self.mount = layout, float(tag_size_m), mount
        self.max_rms_px, self.min_tags, self.huber_px = float(max_rms_px), max(1, int(min_tags)), float(huber_px)
        self.corner_noise_px, self.max_xy_std_m = float(corner_noise_px), float(max_xy_std_m)
        self._set_scale(1440 / np.pi)

    def _set_scale(self, px_per_rad: float) -> None:
        self.px_per_rad = float(px_per_rad)
        self.huber = self.huber_px / self.px_per_rad

    def solve(self, tags: list[X4Tag], frame_height: int = 1440, px_per_rad: float | None = None) -> FieldPose | None:
        """Pixel thresholds are at ``px_per_rad`` (default: the equirect frame's, ``frame_height / pi``; pass
        ``X4TagDetector.px_per_rad`` for another mode)."""
        self._set_scale(frame_height / np.pi if px_per_rad is None else px_per_rad)
        tags = [t for t in tags if t.tag_id in self.layout]
        if len(tags) < self.min_tags:
            return None
        limit = self.max_rms_px / self.px_per_rad
        pose = self._solve(tags)
        if (pose is None or pose[3].max() > limit) and len(tags) > self.min_tags and len(tags) <= 8:
            best = None  # leave each tag out in turn; keep the best fit in which every remaining tag fits
            for k in range(len(tags)):
                rest = tags[:k] + tags[k + 1:]
                fit = self._solve(rest)
                if fit is not None and fit[3].max() <= limit and (best is None or fit[2] < best[0][2]):
                    best = (fit, rest)
            if best is not None:
                pose, tags = best
        if pose is None or pose[3].max() > limit:
            return None
        r_fc, c, rms, _, jac = pose
        cov = self._covariance(r_fc, jac)
        if cov is None:
            return None
        xy_std = float(np.sqrt(np.linalg.eigvalsh(cov[3:5, 3:5]).max()))
        if xy_std > self.max_xy_std_m:
            return None
        r_rc = self.mount.rotation  # camera axes in robot axes
        r_fr = r_fc @ r_rc.T
        robot = self._robot_pose2d(r_fc, c, cov)
        return FieldPose(c, r_fc, c - r_fr @ self.mount.translation, r_fr, rms * self.px_per_rad, [t.tag_id for t in tags],
                         xy_std, float(np.degrees(np.sqrt(cov[2, 2]))), cov, robot.covariance, cov[3:5, 3:5].copy())

    def _covariance(self, r_fc, jac):
        """6x6 covariance of (field-frame rotation, camera position) for ``corner_noise_px`` of corner noise, from
        the fit's Jacobian (rotation updates there are in camera axes: rotated into field axes here).

        Correlations are capped with ``napa_vision.covariance.cap_correlations``, here in the field frame: from two tags
        the linearized covariance is too sure of some combinations of axes. In a Monte Carlo of the test room
        (Gaussian corner noise, 400 poses per case), 11-28% of two-tag robot (x, y, yaw) errors fell outside the 95%
        ellipse without the cap and 3.5-5.5% with it; with every tag in view, 5-6% without and 3.5-4.5% with."""
        sigma = self.corner_noise_px / self.px_per_rad
        try:
            cov = np.linalg.inv(jac.T @ jac) * sigma * sigma
        except np.linalg.LinAlgError:
            return None
        t = np.zeros((6, 6))
        t[:3, :3], t[3:, 3:] = r_fc, np.eye(3)
        cov = cap_correlations(t @ cov @ t.T)
        return cov if np.all(np.isfinite(cov)) else None

    def _robot_pose2d(self, r_fc, c, cov):
        """The robot's (x, y, yaw) and its 3x3 covariance, through the mount, with NAPA's pose types (which order
        position before rotation, both in the field frame: the same convention as ``cov``, reordered)."""
        camera = np.eye(4)
        camera[:3, :3], camera[:3, 3] = r_fc, c
        r_rc, t_rc = self.mount.rotation, self.mount.translation
        camera_to_robot = np.eye(4)
        camera_to_robot[:3, :3], camera_to_robot[:3, 3] = r_rc.T, -r_rc.T @ t_rc
        napa_cov = np.block([[cov[3:, 3:], cov[3:, :3]], [cov[:3, 3:], cov[:3, :3]]])
        return PoseWithCovariance(camera, napa_cov).transform_by(camera_to_robot).to_pose2d()

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
        # Every corner must be seen the way the fit says it is, not from behind (a mirror solution can fit the
        # directions with every one pointing backwards).
        v = (points - c) @ r
        if np.min(np.sum(v / np.linalg.norm(v, axis=1, keepdims=True) * rays, 1)) < np.cos(np.radians(5.0)):
            return None
        per_tag = np.sqrt((res.reshape(len(tags), 4, 2) ** 2).sum(2).mean(1))
        return r, c, rms, per_tag, self._residuals(r, c, points, rays, bases, jacobian=True)[1]

    def _starts(self, tags):
        """Camera poses from each tag's own pose (largest tags first, by angle: the most reliable)."""
        out = []
        for t in sorted(tags, key=lambda t: -float(np.arccos(np.clip(t.rays[0] @ t.rays[2], -1, 1))))[:2]:
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
