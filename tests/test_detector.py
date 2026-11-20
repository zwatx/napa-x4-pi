"""X4TagDetector on rendered 360-degree frames with exact corners."""

import numpy as np
import pytest
from napa_vision import NapaDetector
from synth import add_tag, blank, to_u8, worst_corner_error

from x4tags import X4TagDetector


def frame_with(*specs):
    img = blank()
    truths = {}
    for tag_id, kw in specs:
        truths[tag_id] = add_tag(img, tag_id, **kw)
    return to_u8(img), truths


def by_id(tags):
    return {t.tag_id: t for t in tags}


def test_tag_on_the_horizon_is_found_and_left_alone():
    img, truth = frame_with((7, dict(lon_deg=20, lat_deg=0, distance_m=2.0)))
    found = by_id(X4TagDetector().detect(img))
    assert 7 in found and not found[7].refined and not found[7].seam
    assert worst_corner_error(found[7].corners, truth[7][0]) < 0.3


def test_bent_tag_is_refined_and_its_corners_improve():
    img, truth = frame_with((12, dict(lon_deg=-60, lat_deg=38, distance_m=0.55, tilt_deg=25)))
    plain = by_id(X4TagDetector(refine_bow_px=None).detect(img))[12]
    refined = by_id(X4TagDetector().detect(img))[12]
    assert refined.refined and refined.bow_px > 0.5
    before, after = worst_corner_error(plain.corners, truth[12][0]), worst_corner_error(refined.corners, truth[12][0])
    assert after < 0.35 and after < before - 0.2, (before, after)


def test_tag_across_the_seam_is_found_only_by_the_seam_search():
    img, truth = frame_with((3, dict(lon_deg=180, lat_deg=5, distance_m=1.5)))
    assert 3 not in {int(o.tag_id) for o in NapaDetector().detect(img)}
    found = by_id(X4TagDetector().detect(img))
    assert 3 in found and found[3].seam
    c = found[3].corners
    assert c[:, 0].min() < 0 or c[:, 0].max() > img.shape[1] - 1  # the quad stays whole across the edge
    assert worst_corner_error(c, truth[3][0]) < 0.3


def test_bearing_and_pose_match_where_the_tag_was_put():
    img, truth = frame_with((21, dict(lon_deg=-35, lat_deg=12, distance_m=1.2, turn_deg=30)))
    det = X4TagDetector()
    tag = by_id(det.detect(img))[21]
    az, el = tag.bearing_deg()
    assert az == pytest.approx(-35, abs=0.3) and el == pytest.approx(12, abs=0.3)
    rot, t, err = det.pose(tag, 0.1651)
    np.testing.assert_allclose(t, truth[21][1], atol=0.012)  # within 1% of 1.2 m
    assert err < 1.0


def test_no_tags_no_detections():
    assert X4TagDetector().detect(to_u8(blank())) == []


def test_rejects_frames_that_are_not_2_to_1():
    with pytest.raises(ValueError):
        X4TagDetector().detect(np.zeros((1080, 1920), np.uint8))
