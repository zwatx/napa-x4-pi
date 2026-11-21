"""Geometry of the X4's 1920x1080 webcam mode: two 1920x540 views stacked, the back (top half) and the front (bottom
half). Insta360 doesn't document it; this model is calibrated (one X4, 2026-11) against the 2880x1440 360-degree
mode, by tag corners on a screen seen in both modes from 20 camera poses (about 1150 corners).

Each half covers +-76 degrees across at the horizon and +-24.5 degrees up and down at its middle (+-17 near its
ends), so there is a 28 degree blind gap on each side of the camera, and nothing high or low. In exchange, it has
615 pixels per radian at the middle and about 820 at +-60 degrees, against 458 everywhere in the 360-degree mode.
Calibration error on held-out poses: about 0.3 px median, 0.9-1.2 px at the 95th percentile.

Directions are in the same camera frame as ``x4tags.equirect`` (x right, y down, z forward = the middle of the
front view), so a ``[mount]`` set up for the 360-degree mode still holds. Per half, in that half's own frame
(rotated by the fitted ``view`` angles; the back half is turned 180 degrees), with azimuth ``a`` and elevation
``e`` in radians and pixel centres at integers:

    x = x0 + a * (c0 + c1 e^2 + c2 e^4 + c3 a^2 + c4 a^2 e^2 + c5 a^4)
    y = y0 + e * (d0 + d1 e^2 + d2 e^4 + d3 a^2 + d4 a^2 e^2 + d5 a^4)    (y within the half: 0 at its top row;
                                                                         d0 < 0: up is towards row 0)
"""

from __future__ import annotations

import numpy as np

# Fitted per half: view rotation (yaw, pitch, roll, radians; see _rotation), then (x0, c0..c5) and (y0, d0..d5).
HALVES = {
    "back": dict(row0=0, view=(3.14334163, 9.56322907e-05, 1.69195536e-05),
                 cx=(960.616342, 614.53266, -143.407882, -53.4717798, 50.8931083, -76.7282583, 6.17304755),
                 cy=(272.14187, -620.022193, -58.8664103, 17.5313889, -148.956775, 39.2179308, -34.8141106)),
    "front": dict(row0=540, view=(0.000881518481, -0.000168222024, -1.32565805e-05),
                  cx=(959.849729, 614.619179, -143.580153, -32.163296, 50.4061885, -77.375981, 6.45331601),
                  cy=(272.013473, -620.123498, -44.1348714, -84.4545559, -147.856, 4.86773688, -35.3204526)),
}
FULL_W, FULL_H, HALF_H = 1920, 1080, 540


def _rotation(yaw, pitch, roll):
    cy, sy, cp, sp, cr, sr = np.cos(yaw), np.sin(yaw), np.cos(pitch), np.sin(pitch), np.cos(roll), np.sin(roll)
    ry = np.array([[cy, 0, -sy], [0, 1, 0], [sy, 0, cy]])
    rx = np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]])
    rz = np.array([[cr, -sr, 0], [sr, cr, 0], [0, 0, 1]])
    return rz @ rx @ ry  # camera -> half's frame


def _terms(a, e):
    a2, e2 = a * a, e * e
    return np.stack([np.ones_like(a), e2, e2 * e2, a2, a2 * e2, a2 * a2], -1)


class _Half:
    def __init__(self, row0, view, cx, cy):
        self.row0 = row0
        self.rot = _rotation(*view)
        self.cx, self.cy = np.asarray(cx, float), np.asarray(cy, float)

    def angles(self, rays):
        v = np.asarray(rays, float) @ self.rot.T
        v = v / np.linalg.norm(v, axis=-1, keepdims=True)
        return np.arctan2(v[..., 0], v[..., 2]), -np.arcsin(np.clip(v[..., 1], -1.0, 1.0))

    def forward(self, a, e):
        t = _terms(a, e)
        return self.cx[0] + a * (t @ self.cx[1:]), self.cy[0] + e * (t @ self.cy[1:])

    def inverse(self, x, y, iterations: int = 8):
        """(azimuth, elevation) for half-local pixels, by Newton's method on ``forward``."""
        a = (x - self.cx[0]) / self.cx[1]
        e = (y - self.cy[0]) / self.cy[1]
        h = 1e-6
        for _ in range(iterations):
            fx, fy = self.forward(a, e)
            ax, ay = self.forward(a + h, e)
            ex, ey = self.forward(a, e + h)
            j11, j12, j21, j22 = (ax - fx) / h, (ex - fx) / h, (ay - fy) / h, (ey - fy) / h
            rx, ry = x - fx, y - fy
            det = j11 * j22 - j12 * j21
            a = a + (j22 * rx - j12 * ry) / det
            e = e + (j11 * ry - j21 * rx) / det
        return a, e

    def ray(self, a, e):
        local = np.stack([np.cos(e) * np.sin(a), -np.sin(e), np.cos(e) * np.cos(a)], -1)
        return local @ self.rot  # half's frame -> camera


class DualView:
    """Pixels <-> directions for the 1920x1080 mode, at ``width`` x ``height`` (1920x1080, or a reduced decode such
    as 960x540: pixel centres scale about the image's corner as libjpeg's reduced decode does)."""

    def __init__(self, width: int = FULL_W, height: int = FULL_H):
        if width * FULL_H != height * FULL_W:
            raise ValueError(f"expected a 16:9 frame for the 1920x1080 mode, got {width}x{height}")
        self.width, self.height = int(width), int(height)
        self.scale = width / FULL_W
        self.px_per_rad = float(HALVES["front"]["cx"][1]) * self.scale  # at the middle of each half
        self.halves = {k: _Half(**v) for k, v in HALVES.items()}
        self.boundary = HALF_H * self.scale - 0.5  # between the halves, in this frame's pixels

    def _full(self, u, v):
        return (np.asarray(u, float) + 0.5) / self.scale - 0.5, (np.asarray(v, float) + 0.5) / self.scale - 0.5

    def half_of(self, v):
        """True where row ``v`` is in the front (bottom) half."""
        return np.asarray(v, float) > self.boundary

    def pixel_to_ray(self, u, v) -> np.ndarray:
        """Pixels -> unit rays (..., 3); each pixel through the half it lies in."""
        x, y = self._full(u, v)
        x, y = np.broadcast_arrays(x, y)
        out = np.empty(x.shape + (3,))
        front = self.half_of(v)
        for name, half in self.halves.items():
            m = np.broadcast_to(front if name == "front" else ~front, x.shape)
            if m.any():
                a, e = half.inverse(x[m], y[m] - half.row0)
                out[m] = half.ray(a, e)
        return out

    def facing(self, rays) -> np.ndarray:
        """True where a direction is on the front half's side (within 90 degrees of the front view's middle)."""
        a, _ = self.halves["front"].angles(rays)
        return np.abs(a) <= np.pi / 2

    def ray_to_pixel(self, rays, half: str | None = None) -> np.ndarray:
        """Rays (..., 3) -> pixels (..., 2), each through the half facing it, or all through ``half`` ("front" or
        "back"; smooth, extrapolating beyond that half's picture: see ``outside_half``)."""
        rays = np.asarray(rays, float)
        use_front = self.facing(rays) if half is None else np.full(rays.shape[:-1], half == "front")
        out = np.empty(rays.shape[:-1] + (2,))
        for name, h in self.halves.items():
            m = use_front if name == "front" else ~use_front
            if m.any():
                a, e = h.angles(rays[m])
                x, y = h.forward(a, e)
                out[m] = np.stack([x, y + h.row0], -1)
        return (out + 0.5) * self.scale - 0.5

    def local_px_per_rad(self, ray) -> float:
        """The largest pixels per radian around ``ray`` (the picture is stretched towards each half's ends: ~615
        at the middle, ~820 at +-60 degrees)."""
        r = np.asarray(ray, float) / np.linalg.norm(ray)
        half = "front" if self.facing(r) else "back"
        side = np.cross(r, [0.0, 1.0, 0.0])
        side = side / np.linalg.norm(side) if np.linalg.norm(side) > 1e-9 else np.array([1.0, 0.0, 0.0])
        up = np.cross(side, r)
        h = 1e-4
        p = self.ray_to_pixel(np.stack([r, r + h * side, r + h * up]), half=half)
        return float(max(np.linalg.norm(p[1] - p[0]), np.linalg.norm(p[2] - p[0])) / h)

    def outside_half(self, u, v, half: str) -> np.ndarray:
        """True where pixels lie outside ``half``'s part of the image."""
        lo, hi = (self.boundary, self.height - 0.5) if half == "front" else (-0.5, self.boundary)
        u, v = np.asarray(u), np.asarray(v)
        return (u < -0.5) | (u > self.width - 0.5) | (v < lo) | (v > hi)
