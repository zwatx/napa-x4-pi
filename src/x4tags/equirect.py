"""Geometry of the X4's stitched webcam frame: a 2:1 equirectangular image covering every direction.

Camera frame is OpenCV's: x right, y down, z forward (the middle of the image). Longitude turns about y from +z
towards +x; latitude is positive above the horizon. Pixel centres are at integers, like OpenCV and NAPA corners:
column u = (lon / 2pi + 0.5) * W - 0.5, row v = (0.5 - lat / pi) * H - 0.5. The left and right edges are the same
direction (straight behind the camera), so u is periodic in W.
"""

from __future__ import annotations

import cv2
import numpy as np


def pixel_to_ray(u, v, width: int, height: int) -> np.ndarray:
    """Pixels -> unit rays, shape (..., 3)."""
    lon = ((np.asarray(u, float) + 0.5) / width - 0.5) * 2 * np.pi
    lat = (0.5 - (np.asarray(v, float) + 0.5) / height) * np.pi
    c = np.cos(lat)
    return np.stack([c * np.sin(lon), -np.sin(lat), c * np.cos(lon)], -1)


def ray_to_pixel(rays, width: int, height: int) -> np.ndarray:
    """Rays (any length, ..., 3) -> pixels (..., 2); u in [-0.5, W - 0.5)."""
    rays = np.asarray(rays, float)
    x, y, z = rays[..., 0], rays[..., 1], rays[..., 2]
    lon = np.arctan2(x, z)
    lat = np.arcsin(np.clip(-y / np.linalg.norm(rays, axis=-1), -1.0, 1.0))
    return np.stack([(lon / (2 * np.pi) + 0.5) * width - 0.5, (0.5 - lat / np.pi) * height - 0.5], -1)


def unwrap_near(u, ref, width: int):
    """Shift columns by whole turns so they lie within half a turn of ``ref`` (for polygons that cross the seam)."""
    return u - np.round((u - ref) / width) * width


def tangent_basis(direction) -> np.ndarray:
    """Rows (right, down, forward): a camera looking along ``direction``, kept upright (image down ~ world y)."""
    x, y, z = (float(v) for v in direction)
    n = (x * x + y * y + z * z) ** 0.5
    x, y, z = x / n, y / n, z / n
    rx, rz = z, -x  # (0, 1, 0) x f
    m = (rx * rx + rz * rz) ** 0.5
    rx, ry, rz = (rx / m, 0.0, rz / m) if m > 1e-6 else (1.0, 0.0, 0.0)  # looking straight up or down
    return np.array([[rx, ry, rz], [y * rz - z * ry, z * rx - x * rz, x * ry - y * rx], [x, y, z]])


def tangent_bases(directions) -> np.ndarray:
    """``tangent_basis`` for many directions at once: (n, 3, 3), rows (right, down, forward) per direction."""
    f = np.asarray(directions, float)
    f = f / np.linalg.norm(f, axis=1, keepdims=True)
    right = np.stack([f[:, 2], np.zeros(len(f)), -f[:, 0]], 1)  # (0, 1, 0) x f
    n = np.linalg.norm(right, axis=1, keepdims=True)
    right = np.where(n < 1e-6, [1.0, 0.0, 0.0], right / np.maximum(n, 1e-12))
    down = np.stack([f[:, 1] * right[:, 2] - f[:, 2] * right[:, 1], f[:, 2] * right[:, 0] - f[:, 0] * right[:, 2],
                     f[:, 0] * right[:, 1] - f[:, 1] * right[:, 0]], 1)  # f x right
    return np.stack([right, down, f], 1)


class TangentView:
    """A small pinhole view of the sphere centred on ``direction``: straight lines in the world stay straight here.

    ``focal`` is in pixels per radian at the centre; ``size`` is the square view's side in pixels. ``projection``:
    the source image's geometry if it isn't this module's equirect frame: an ``x4tags.dual.DualView``, through the
    half the view faces; whatever falls outside that half renders black.
    """

    def __init__(self, direction, focal: float, size: int, width: int, height: int, projection=None):
        self.basis = tangent_basis(direction)
        self.focal, self.size, self.width, self.height = float(focal), int(size), width, height
        self.projection = projection
        self.half = None if projection is None else ("front" if projection.facing(self.basis[2]) else "back")
        self.c = (self.size - 1) / 2.0

    def view_to_ray(self, px) -> np.ndarray:
        px = np.asarray(px, float)
        local = np.stack([(px[..., 0] - self.c) / self.focal, (px[..., 1] - self.c) / self.focal,
                          np.ones(px.shape[:-1])], -1)
        return local @ self.basis

    def ray_to_view(self, rays) -> np.ndarray:
        local = np.asarray(rays, float) @ self.basis.T
        return np.stack([local[..., 0] / local[..., 2], local[..., 1] / local[..., 2]], -1) * self.focal + self.c

    def render(self, image: np.ndarray, step: int = 8, max_error_px: float = 0.03, min_step: int = 2) -> np.ndarray:
        """Resample the equirectangular ``image`` into this view (bilinear; wraps across the seam).

        The pixel map is computed exactly at knots ``step`` pixels apart and linearly interpolated between them by
        ``cv2.resize`` (float, so unquantized). Its error is measured once at the middles of the outer and middle rows
        of knot cells, where it is largest; if over ``max_error_px``, the step shrinks by the square root of the ratio
        (linear interpolation error grows with the step squared), to a power of two no smaller than ``min_step``.
        Coarse near the horizon, finer for big views far from it."""
        n = self.size
        centre_u = ray_to_pixel(self.basis[2], self.width, self.height)[0] if self.projection is None else 0.0
        k = int(step)
        g, uv = self._knots(n, k, centre_u)
        m = len(g)
        rows = np.unique(np.r_[0, (m - 1) // 2 - 1, (m - 1) // 2, m - 2].clip(0, m - 2))
        mid = g[:-1] + k / 2.0
        exact = self._uv(mid, mid[rows], centre_u)
        linear = (uv[rows, :-1] + uv[rows + 1, :-1] + uv[rows, 1:] + uv[rows + 1, 1:]) / 4
        err = float(np.abs(linear - exact).max())
        if err > max_error_px and k > min_step:
            want = k * np.sqrt(max_error_px / err)
            k = max(int(min_step), 1 << max(0, int(np.floor(np.log2(want)))))
            g, uv = self._knots(n, k, centre_u)
            m = len(g)
        big = (m * k, m * k)
        mx = cv2.resize(uv[..., 0].astype(np.float32), big, interpolation=cv2.INTER_LINEAR)[k:k + n, k:k + n]
        my = cv2.resize(uv[..., 1].astype(np.float32), big, interpolation=cv2.INTER_LINEAR)[k:k + n, k:k + n]
        self.last_step = k
        if self.projection is not None:
            off = self.projection.outside_half(mx, my, self.half)
            mx[off], my[off] = -1e4, -1e4
            return cv2.remap(image, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        if uv[..., 0].min() < -0.5 or uv[..., 0].max() >= self.width - 0.5:  # this view crosses the seam
            mx = np.mod(mx + 0.5, np.float32(self.width)) - np.float32(0.5)
        return cv2.remap(image, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_WRAP)

    def _knots(self, n, k, centre_u):
        # cv2.resize (linear, x k) puts source sample i at output pixel k*i + (k-1)/2; knot i sits at view pixel
        # k*(i-1) + (k-1)/2 (one spare knot on each side), so view pixel x is output pixel x + k.
        g = (np.arange(-(-n // k) + 2) - 1) * k + (k - 1) / 2.0
        return g, self._uv(g, g, centre_u)

    def _uv(self, xs, ys, centre_u):
        gx, gy = np.meshgrid(xs, ys)
        if self.projection is not None:
            return self.projection.ray_to_pixel(self.view_to_ray(np.stack([gx, gy], -1)), half=self.half)
        uv = ray_to_pixel(self.view_to_ray(np.stack([gx, gy], -1)), self.width, self.height)
        uv[..., 0] = unwrap_near(uv[..., 0], centre_u, self.width)  # continuous across the seam
        return uv


def side_bow_px(corners: np.ndarray, width: int, height: int) -> float:
    """How far (pixels) the equirect frame bends the tag's straightest-possible sides: the largest gap between each
    side's chord midpoint and where the true (great-circle) side's midpoint lands. Near 0 for small tags near the
    horizon, where fitting straight sides in pixels is already right."""
    rays = pixel_to_ray(corners[:, 0], corners[:, 1], width, height)
    mids = ray_to_pixel(rays + np.roll(rays, -1, 0), width, height)
    u = unwrap_near(corners[:, 0], corners[0, 0], width)
    chord = (np.stack([u, corners[:, 1]], -1) + np.roll(np.stack([u, corners[:, 1]], -1), -1, 0)) / 2
    mids[:, 0] = unwrap_near(mids[:, 0], chord[:, 0], width)
    return float(np.linalg.norm(mids - chord, axis=1).max())


def angular_radius(rays: np.ndarray, centre: np.ndarray) -> float:
    c = centre / np.linalg.norm(centre)
    r = rays / np.linalg.norm(rays, axis=1, keepdims=True)
    return float(np.arccos(np.clip(r @ c, -1, 1)).max())


__all__ = ["pixel_to_ray", "ray_to_pixel", "unwrap_near", "tangent_basis", "TangentView", "side_bow_px",
           "angular_radius"]
