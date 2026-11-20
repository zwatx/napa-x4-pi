"""Tiny renderer for tests: real tag36h11 tags as flat squares in 3D, seen by a 2:1 equirectangular camera.

Each pixel is the average of ss x ss rays (antialiasing); the truth corners are the black square's corners
projected exactly, in the same pixel convention as x4tags.equirect.
"""

from __future__ import annotations

import math

import numpy as np
from napa_vision.tags import tag_image

from x4tags.equirect import pixel_to_ray, ray_to_pixel, tangent_basis, unwrap_near

W, H = 2880, 1440


def blank(width: int = W, height: int = H, level: int = 110, noise: float = 2.0, seed: int = 0) -> np.ndarray:
    rng = np.random.default_rng(seed)
    img = np.full((height, width), float(level))
    img += rng.normal(0, noise, img.shape)
    return img


def _rot(axis, angle):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k


def add_tag(img: np.ndarray, tag_id: int, lon_deg: float, lat_deg: float, distance_m: float, size_m: float = 0.1651,
            turn_deg: float = 0.0, tilt_deg: float = 0.0, roll_deg: float = 0.0, ss: int = 4):
    """Draw a tag facing the camera (then turned about its vertical axis, tilted about its horizontal one and
    rolled about its normal) whose centre is ``distance_m`` away towards (lon, lat). Returns (truth corners (4, 2),
    centre point in camera coordinates)."""
    height, width = img.shape
    lon, lat = math.radians(lon_deg), math.radians(lat_deg)
    d = np.array([math.cos(lat) * math.sin(lon), -math.sin(lat), math.cos(lat) * math.cos(lon)])
    right, down, fwd = tangent_basis(d)
    r = _rot(fwd, math.radians(roll_deg)) @ _rot(right, math.radians(tilt_deg)) @ _rot(down, math.radians(turn_deg))
    ex, ey = r @ right, r @ down  # tag's own right and down, as printed
    n = np.cross(ex, ey)
    centre = d * distance_m
    cell = size_m / 8
    pattern = tag_image(tag_id, cell_px=1, quiet_zone_cells=1).astype(float)  # 10 x 10 cells incl. white margin
    half_out = 5 * cell

    def project(a, b):
        return ray_to_pixel(centre + np.outer(a, ex) + np.outer(b, ey), width, height)

    s = np.linspace(-half_out, half_out, 41)
    edge = np.concatenate([np.stack([s, np.full_like(s, -half_out)], 1), np.stack([s, np.full_like(s, half_out)], 1),
                           np.stack([np.full_like(s, -half_out), s], 1), np.stack([np.full_like(s, half_out), s], 1)])
    outline = project(edge[:, 0], edge[:, 1])
    cu = ray_to_pixel(centre, width, height)[0]
    outline[:, 0] = unwrap_near(outline[:, 0], cu, width)
    u0, u1 = int(math.floor(outline[:, 0].min())) - 2, int(math.ceil(outline[:, 0].max())) + 2
    v0, v1 = max(0, int(math.floor(outline[:, 1].min())) - 2), min(height - 1, int(math.ceil(outline[:, 1].max())) + 2)
    us, vs = np.arange(u0, u1 + 1), np.arange(v0, v1 + 1)
    offs = (np.arange(ss) + 0.5) / ss - 0.5
    background = img[np.ix_(vs, np.mod(us, width))]
    acc = np.zeros((len(vs), len(us)))
    hit_any = np.zeros_like(acc, bool)
    for du in offs:
        for dv in offs:
            uu, vv = np.meshgrid(us + du, vs + dv)
            ray = pixel_to_ray(uu, vv, width, height)
            denom = ray @ n
            t = (centre @ n) / np.where(np.abs(denom) > 1e-12, denom, 1e-12)
            p = ray * t[..., None] - centre
            a, b = p @ ex, p @ ey
            inside = (t > 0) & (np.abs(a) < half_out) & (np.abs(b) < half_out)
            col = np.clip(((a + half_out) / cell).astype(int), 0, 9)
            row = np.clip(((b + half_out) / cell).astype(int), 0, 9)
            acc += np.where(inside, pattern[row, col], 0.0)
            hit_any |= inside
            acc[~inside] += background[~inside]
    acc /= ss * ss
    region = background.copy()
    region[hit_any] = acc[hit_any]
    img[np.ix_(vs, np.mod(us, width))] = region
    h = 4 * cell
    truth = project(np.array([-h, h, h, -h]), np.array([-h, -h, h, h]))
    truth[:, 0] = unwrap_near(truth[:, 0], cu, width)
    return truth, centre


def to_u8(img: np.ndarray) -> np.ndarray:
    return np.clip(img + 0.5, 0, 255).astype(np.uint8)


def worst_corner_error(corners: np.ndarray, truth: np.ndarray, width: int = W) -> float:
    c = np.asarray(corners, float).copy()
    c[:, 0] = unwrap_near(c[:, 0], truth[:, 0].mean(), width)
    orders = [np.roll(o, k, 0) for o in (c, c[::-1]) for k in range(4)]
    return min(float(np.linalg.norm(o - truth, axis=1).max()) for o in orders)
