"""Geometry of the stitched frame and the straight-sided views used for refinement."""

import numpy as np
import pytest

from x4tags.equirect import TangentView, pixel_to_ray, ray_to_pixel, side_bow_px, unwrap_near

W, H = 2880, 1440


def test_pixel_ray_round_trip_and_axes():
    rng = np.random.default_rng(1)
    u, v = rng.uniform(-0.5, W - 0.5, 5000), rng.uniform(0, H - 1, 5000)
    back = ray_to_pixel(pixel_to_ray(u, v, W, H), W, H)
    assert np.abs(back - np.stack([u, v], -1)).max() < 1e-9
    np.testing.assert_allclose(pixel_to_ray((W - 1) / 2, (H - 1) / 2, W, H), [0, 0, 1], atol=1e-12)  # middle = forward
    assert pixel_to_ray(0, -0.5, W, H)[1] == pytest.approx(-1)  # top edge = straight up (-y)


@pytest.mark.parametrize("u0,v0,size", [(100, 700, 201), (1000, 450, 301), (1500, 330, 401), (2879, 650, 257)])
def test_view_map_accuracy(u0, v0, size):
    """The interpolated resampling map matches the exact one (checked by resampling coordinate images)."""
    view = TangentView(pixel_to_ray(u0, v0, W, H), H / np.pi, size, W, H)
    g = np.arange(size, dtype=float)
    exact = ray_to_pixel(view.view_to_ray(np.stack(np.meshgrid(g, g), -1)), W, H)
    cols = np.tile(np.arange(W, dtype=np.float32), (H, 1))
    rows = np.tile(np.arange(H, dtype=np.float32)[:, None], (1, W))
    du = np.abs(unwrap_near(view.render(cols), exact[..., 0], W) - exact[..., 0])
    dv = np.abs(view.render(rows) - exact[..., 1])
    away = (np.abs(unwrap_near(exact[..., 0], 0, W)) > 2) & (np.abs(unwrap_near(exact[..., 0], W - 1, W)) > 2)
    # 0.03 px map target plus cv2.remap's own 1/32 px position rounding (OpenCV 4.10 reaches 0.043)
    assert du[away].max() < 0.05 and dv.max() < 0.05


def test_bow_is_small_on_the_horizon_and_larger_above_it():
    sq = np.array([[-20, -20], [20, -20], [20, 20], [-20, 20]], float)
    assert side_bow_px(sq + [1440, 719.5], W, H) < 0.05  # top and bottom sides are just off the horizon
    assert side_bow_px(sq + [1440, 500], W, H) > 0.15  # 27 degrees up: 0.19 px


def test_tangent_bases_match_the_single_version():
    from x4tags.equirect import tangent_basis, tangent_bases

    rng = np.random.default_rng(2)
    dirs = np.concatenate([rng.normal(size=(50, 3)), [[0, 1.0, 0], [0, -2.0, 0]]])
    np.testing.assert_allclose(tangent_bases(dirs), np.stack([tangent_basis(d) for d in dirs]), atol=1e-12)
