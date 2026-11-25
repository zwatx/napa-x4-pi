"""X4TagDetector on rendered 360-degree frames with exact corners."""

import numpy as np
import pytest
from napa_vision import NapaDetector
from synth import add_tag, blank, to_u8, worst_corner_error

from x4tags import X4TagDetector
from x4tags.detector import lens_centre


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


@pytest.mark.parametrize("lon", [-35, 150])  # in the front lens's half and in the back lens's
def test_bearing_and_pose_match_where_the_tag_was_put(lon):
    img, truth = frame_with((21, dict(lon_deg=lon, lat_deg=12, distance_m=1.2, turn_deg=30)))
    det = X4TagDetector()
    tag = by_id(det.detect(img))[21]
    az, el = tag.bearing_deg()
    assert az == pytest.approx(lon, abs=0.3) and el == pytest.approx(12, abs=0.3)
    _rot, t, err = det.pose(tag, 0.1651)
    np.testing.assert_allclose(t, truth[21][1], atol=0.012)  # within 1% of 1.2 m
    assert err < 1.0


def test_no_tags_no_detections():
    assert X4TagDetector().detect(to_u8(blank())) == []


def test_rejects_frames_that_are_not_2_to_1():
    with pytest.raises(ValueError):
        X4TagDetector().detect(np.zeros((1080, 1920), np.uint8))


def test_corner_order_and_pose_axes_follow_the_printed_tag():
    """Corner 0 is the printed bottom-right; pose axes: x to the printed left, y to the printed bottom, z out."""
    img, truth = frame_with((7, dict(lon_deg=0, lat_deg=0, distance_m=1.0, turn_deg=20)))
    det = X4TagDetector()
    tag = by_id(det.detect(img))[7]
    tl, tr, br, bl = truth[7][0]  # synth's truth order: printed top-left, top-right, bottom-right, bottom-left
    np.testing.assert_allclose(tag.corners, [br, bl, tl, tr], atol=0.3)
    rot, t, _ = det.pose(tag, 0.1651)
    assert rot[:, 0] @ [-1, 0, 0] > 0.9  # printed left = camera's left (tag faces the camera, turned 20 deg)
    assert rot[:, 1] @ [0, 1, 0] > 0.9  # printed bottom = down
    assert rot[:, 2] @ (-t / np.linalg.norm(t)) > 0.9  # out of the face, towards the camera


def test_centre_ray_is_the_true_centre_for_a_close_turned_tag():
    img, truth = frame_with((30, dict(lon_deg=25, lat_deg=10, distance_m=0.4, turn_deg=55, tilt_deg=30)))
    tag = by_id(X4TagDetector().detect(img))[30]
    centre = truth[30][1] - lens_centre(truth[30][1])  # as the lens sees it
    centre /= np.linalg.norm(centre)
    mean = tag.rays.sum(0) / np.linalg.norm(tag.rays.sum(0))
    assert np.degrees(np.arccos(np.clip(tag.center_ray @ centre, -1, 1))) < 0.1
    assert np.degrees(np.arccos(np.clip(mean @ centre, -1, 1))) > 0.3  # what the corner mean would have been


@pytest.mark.parametrize("lat,dist,tilt,lon",
                         [(40, 0.35, 0, 30), (50, 0.3, 0, 30), (35, 0.25, 30, 30), (-35, 0.4, 0, -135)])
def test_close_tag_too_bent_for_the_frame_is_found_in_a_straight_view(lat, dist, tilt, lon):
    img, truth = frame_with((44, dict(lon_deg=lon, lat_deg=lat, distance_m=dist, tilt_deg=tilt)))
    assert 44 not in {int(o.tag_id) for o in NapaDetector().detect(img)}  # the frame bends it too much
    assert 44 not in by_id(X4TagDetector(near_scale=0).detect(img))
    tag = by_id(X4TagDetector(near_per_frame=0).detect(img))[44]
    assert tag.near and worst_corner_error(tag.corners, truth[44][0]) < 0.35


def test_rotating_near_views_find_a_close_tag_within_a_cycle_then_every_frame():
    img, _truth = frame_with((44, dict(lon_deg=30, lat_deg=40, distance_m=0.35)))
    det = X4TagDetector(near_per_frame=2)
    seen = [44 in by_id(det.detect(img)) for _ in range(12)]
    first = seen.index(True)
    assert first < 6 and all(seen[first:])  # 12 views / 2 per frame; then its view is searched every frame
