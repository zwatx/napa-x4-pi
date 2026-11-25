"""The X4's 1920x1080 mode: its calibrated geometry, and tags found in it."""

import numpy as np
import pytest
from synth import add_tag, blank, equirect_to_dual, to_u8

from x4tags.detector import X4TagDetector
from x4tags.dual import DualView
from x4tags.equirect import pixel_to_ray

# Corners measured in the calibration (1080-mode pixel, direction from the 2880x1440 mode), from steady poses.
CALIBRATION_SAMPLE = [
    (1218.696, 773.117, 0.4040896, -0.0600548, 0.9127459),
    (1216.886, 919.378, 0.3987028, 0.1646498, 0.9021788),
    (798.257, 894.548, -0.2566428, 0.1302016, 0.9576962),
    (932.394, 889.779, -0.0436212, 0.1249175, 0.9912077),
    (1202.359, 236.555, -0.3800668, -0.0552173, -0.9233094),
    (1000.673, 294.17, -0.0671083, 0.035724, -0.9971059),
    (322.016, 232.346, 0.8164661, -0.0499329, -0.5752302),
    (141.059, 393.054, 0.9164322, 0.1341832, -0.3770238),
    (126.18, 780.699, -0.9286109, -0.0351681, 0.3693847),
    (16.828, 843.913, -0.9653302, 0.0327014, 0.2589755),
    (1449.644, 868.221, 0.6868629, 0.0775057, 0.7226425),
    (1771.284, 772.214, 0.9193012, -0.0443017, 0.3910533),
    (1180.595, 443.48, -0.3428451, 0.2626962, -0.9019135),
    (1794.626, 455.309, -0.9162116, 0.1993099, -0.3476089),
    (663.9, 231.418, 0.4536872, -0.0621395, -0.8889919),
    (596.569, 437.469, 0.5338357, 0.2410624, -0.8104988),
]


def test_pixels_and_directions_round_trip_in_both_halves():
    d = DualView()
    u, v = np.meshgrid(np.linspace(0, 1919, 40), np.r_[np.linspace(0, 539, 15), np.linspace(540, 1079, 15)])
    rays = d.pixel_to_ray(u, v)
    assert np.allclose(np.linalg.norm(rays, axis=-1), 1)
    assert np.abs(d.ray_to_pixel(rays) - np.stack([u, v], -1)).max() < 1e-6
    assert d.pixel_to_ray(960, 812)[2] > 0.999 and d.pixel_to_ray(960, 272)[2] < -0.999  # front below, back above
    small = DualView(960, 540)  # libjpeg's half-size decode: same directions at the scaled pixel centres
    assert np.abs(small.pixel_to_ray((u + 0.5) / 2 - 0.5, (v + 0.5) / 2 - 0.5) - rays).max() < 1e-9
    with pytest.raises(ValueError):
        DualView(2880, 1440)


def test_model_matches_the_calibration_measurements():
    d = DualView()
    pts = np.array(CALIBRATION_SAMPLE)
    err = np.linalg.norm(d.ray_to_pixel(pts[:, 2:]) - pts[:, :2], axis=1)
    assert np.median(err) < 0.5 and err.max() < 1.5


def _dual_frame(width=960, height=540):
    """A 360-degree scene rendered with tags, resampled into the 1080 mode's layout through the model; and the
    tags' true corner directions. (Made with the same model the detector uses: this checks the detector, not the
    calibration; ``CALIBRATION_SAMPLE`` checks that.)"""
    img = blank(2880, 1440)
    truth = {}
    for tag_id, lon, lat, dist in [(1, 0, 0, 1.6), (2, 55, 8, 1.6), (3, -40, -12, 1.4), (4, 178, 5, 1.6),
                                   (5, -150, 10, 1.5), (6, 25, -5, 0.6)]:
        corners, _ = add_tag(img, tag_id, lon, lat, dist, turn_deg=15 * (tag_id % 3 - 1), ss=3)
        truth[tag_id] = pixel_to_ray(corners[:, 0], corners[:, 1], 2880, 1440)
    return equirect_to_dual(to_u8(img), width, height), truth, DualView(width, height)


def _worst_angle(rays, truth):
    best = np.inf
    for o in (rays, rays[::-1]):
        for k in range(4):
            r = np.roll(o, k, 0)
            best = min(best, float(np.arccos(np.clip(np.sum(r * truth, 1), -1, 1)).max()))
    return best


def test_finds_tags_in_both_halves_with_accurate_directions():
    frame, truth, d = _dual_frame()
    tags = X4TagDetector(mode="dual1080").detect(frame)
    found = {t.tag_id: t for t in tags}
    assert set(found) == set(truth)
    for tag_id, t in found.items():
        err_px = _worst_angle(t.rays, truth[tag_id]) * d.px_per_rad
        assert err_px < 0.6, (tag_id, err_px)
    assert found[6].refined  # the close tag is bent enough to be looked at again through a straight view
    det = X4TagDetector(mode="dual1080")
    assert det.px_per_rad(960, 540) == pytest.approx(d.px_per_rad)
    pose = X4TagDetector.pose(found[1], 0.1651, px_per_rad=d.px_per_rad)
    assert pose is not None and abs(np.linalg.norm(pose[1]) - 1.6) < 0.03


def test_dual_mode_rejects_a_360_frame():
    with pytest.raises(ValueError):
        X4TagDetector(mode="dual1080").detect(np.zeros((1440, 2880), np.uint8))
    with pytest.raises(ValueError):
        X4TagDetector(mode="fisheye")
