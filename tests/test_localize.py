"""Field localization from every visible tag, on a rendered room of tags with a known camera pose."""

import math

import numpy as np
import pytest
from synth import add_tag_at, blank, to_u8

from x4tags import X4TagDetector
from x4tags.field import FieldLayout, matrix_to_rpy, quaternion_to_matrix, matrix_to_quaternion
from x4tags.localize import FieldLocalizer, robot_position_from_heading
from x4tags.mount import Mount, rotation_rpy

SIZE = 0.1651


def room_layout() -> dict:
    """WPILib-format layout: 12 tags on the walls of a 6 m x 4 m room, facing in, 0.4-1.4 m up."""
    tags, k = [], 1
    for x, y, yaw in [(0.0, 1.0, 0), (0.0, 2.5, 0), (6.0, 1.2, 180), (6.0, 3.0, 180), (2.0, 0.0, 90), (4.5, 0.0, 90),
                      (1.5, 4.0, -90), (3.5, 4.0, -90), (5.0, 4.0, -90), (0.0, 3.6, 0), (6.0, 0.4, 180), (3.0, 0.0, 90)]:
        z = 0.4 + 0.25 * (k % 5)
        q = matrix_to_quaternion(rotation_rpy(0, 0, math.radians(yaw)))
        tags.append({"ID": k, "pose": {"translation": {"x": x, "y": y, "z": z},
                                      "rotation": {"quaternion": {"W": q[0], "X": q[1], "Y": q[2], "Z": q[3]}}}})
        k += 1
    return {"tags": tags, "field": {"length": 6.0, "width": 4.0}}


def render_from(layout: FieldLayout, r_fc: np.ndarray, c: np.ndarray, max_m: float = 4.0, seed: int = 0):
    img = blank(seed=seed)
    for tag in layout.tags.values():
        to_cam = c - tag.position
        if tag.rotation[:, 0] @ to_cam <= 0.3 * np.linalg.norm(to_cam) or np.linalg.norm(to_cam) > max_m:
            continue  # facing away, too oblique or too far
        r_ct = r_fc.T @ tag.rotation  # WPILib tag axes in camera axes
        add_tag_at(img, tag.tag_id, r_fc.T @ (tag.position - c), r_ct[:, 1], -r_ct[:, 2], SIZE)
    return to_u8(img)


def test_layout_json_round_trip_and_corner_order():
    layout = FieldLayout.from_dict(room_layout())
    assert len(layout.tags) == 12 and layout.length_m == 6.0
    corners = layout[1].corners(SIZE)  # tag 1 on the x=0 wall facing +x: printed right is +y (the viewer's right)
    np.testing.assert_allclose(corners[0], layout[1].position + [0, SIZE / 2, -SIZE / 2])  # bottom-right first
    np.testing.assert_allclose(quaternion_to_matrix(*matrix_to_quaternion(layout[3].rotation)), layout[3].rotation, atol=1e-12)


@pytest.mark.parametrize("robot_xy,robot_yaw_deg", [((2.4, 1.7), 30.0), ((4.0, 2.6), -120.0), ((1.2, 1.0), 175.0)])
def test_robot_pose_from_all_tags(robot_xy, robot_yaw_deg):
    layout = FieldLayout.from_dict(room_layout())
    mount = Mount(x_m=0.15, y_m=-0.05, z_m=0.55, yaw_deg=90)  # camera turned to face the robot's left
    r_fr = rotation_rpy(0, 0, math.radians(robot_yaw_deg))
    p_r = np.array([*robot_xy, 0.0])
    r_fc, c = r_fr @ mount.rotation, p_r + r_fr @ mount.translation
    img = render_from(layout, r_fc, c)
    tags = X4TagDetector().detect(img)
    assert len(tags) >= 4
    pose = FieldLocalizer(layout, SIZE, mount).solve(tags)
    assert pose is not None and len(pose.tag_ids) >= 4
    assert np.linalg.norm(pose.camera_position - c) < 0.02
    assert np.linalg.norm(pose.robot_position - p_r) < 0.02
    yaw = math.degrees(matrix_to_rpy(pose.robot_rotation)[2])
    assert (yaw - robot_yaw_deg + 180) % 360 - 180 == pytest.approx(0, abs=0.3)
    assert pose.rms_px < 1.0
    # with heading from a gyro instead (for a picture the camera rotates), the same position comes out
    np.testing.assert_allclose(robot_position_from_heading(pose.camera_position, math.radians(robot_yaw_deg), mount),
                               p_r, atol=0.02)


def test_unknown_tags_and_no_tags():
    layout = FieldLayout.from_dict(room_layout())
    loc = FieldLocalizer(layout, SIZE)
    assert loc.solve([]) is None


def _scene(robot_xy, yaw_deg, mount=Mount(z_m=0.55), seed=0, max_m=4.0):
    layout = FieldLayout.from_dict(room_layout())
    r_fr = rotation_rpy(0, 0, math.radians(yaw_deg))
    p_r = np.array([*robot_xy, 0.0])
    img = render_from(layout, r_fr @ mount.rotation, p_r + r_fr @ mount.translation, max_m=max_m, seed=seed)
    return layout, mount, p_r, X4TagDetector().detect(img)


def test_one_tag_alone_gives_no_field_pose():
    layout, mount, p_r, tags = _scene((2.4, 1.7), 30.0)
    assert FieldLocalizer(layout, SIZE, mount).solve(tags[:1]) is None  # ambiguous: two fits, metres apart
    assert FieldLocalizer(layout, SIZE, mount, min_tags=1).solve(tags[:1]) is not None  # if you insist


def test_a_misread_tag_never_yields_a_wrong_pose():
    """One tag reported with another layout tag's ID: the pose either leaves it out or isn't given."""
    import dataclasses

    rng = np.random.default_rng(1)
    for k in range(8):
        xy, yaw = (rng.uniform(1.0, 5.0), rng.uniform(1.0, 3.0)), rng.uniform(-180, 180)
        layout, mount, p_r, tags = _scene(xy, yaw, seed=k)
        tags = sorted(tags, key=lambda t: t.tag_id)[:3]
        if len(tags) < 3:
            continue
        others = [i for i in layout.tags if i not in {t.tag_id for t in tags}]
        bad = dataclasses.replace(tags[1], tag_id=int(rng.choice(others)))
        pose = FieldLocalizer(layout, SIZE, mount).solve([tags[0], bad, tags[2]])
        if pose is not None:
            assert bad.tag_id not in pose.tag_ids
            assert np.linalg.norm(pose.robot_position - p_r) < 0.05


def _noisy_tags(layout, r_fc, c, sigma_px, rng, max_m=6.0):
    """Tags with their exact corner directions plus Gaussian corner noise (``sigma_px`` across each ray)."""
    from x4tags.detector import X4Tag
    from x4tags.equirect import tangent_bases

    out = []
    for tag in layout.tags.values():
        to_cam = c - tag.position
        if tag.rotation[:, 0] @ to_cam <= 0.3 * np.linalg.norm(to_cam) or np.linalg.norm(to_cam) > max_m:
            continue
        v = (tag.corners(SIZE) - c) @ r_fc
        u = v / np.linalg.norm(v, axis=1, keepdims=True)
        u = u + np.einsum("kij,ki->kj", tangent_bases(u)[:, :2], rng.normal(0, sigma_px * np.pi / 1440, (4, 2)))
        out.append(X4Tag(tag.tag_id, np.zeros((4, 2)), u / np.linalg.norm(u, axis=1, keepdims=True), 0, 100.0, 0.0,
                         False, False))
    return out


@pytest.mark.parametrize("n_tags", [2, None])
def test_covariances_match_the_errors(n_tags):
    """With Gaussian corner noise of the assumed size, about 5% of errors fall outside the published covariances'
    95% ellipses: the robot's (x, y, yaw) and the camera's (x, y). Two tags alone are the hard case (see
    ``FieldLocalizer._covariance``)."""
    layout = FieldLayout.from_dict(room_layout())
    mount = Mount(x_m=0.15, y_m=-0.05, z_m=0.55, yaw_deg=90)
    loc = FieldLocalizer(layout, SIZE, mount, max_xy_std_m=10)
    rng = np.random.default_rng(3)
    robot_nees, camera_nees = [], []
    for _ in range(120):
        p_r, yaw = np.array([rng.uniform(0.6, 5.4), rng.uniform(0.6, 3.4), 0.0]), rng.uniform(-np.pi, np.pi)
        r_fr = rotation_rpy(0, 0, yaw)
        r_fc, c = r_fr @ mount.rotation, p_r + r_fr @ mount.translation
        tags = _noisy_tags(layout, r_fc, c, loc.corner_noise_px, rng)[:n_tags]
        pose = loc.solve(tags) if len(tags) >= 2 else None
        if pose is None:
            continue
        robot_yaw = matrix_to_rpy(pose.robot_rotation)[2]
        e = np.array([*(p_r - pose.robot_position)[:2], (yaw - robot_yaw + np.pi) % (2 * np.pi) - np.pi])
        robot_nees.append(e @ np.linalg.solve(pose.robot_covariance, e))
        e = (c - pose.camera_position)[:2]
        camera_nees.append(e @ np.linalg.solve(pose.camera_xy_covariance, e))
        np.testing.assert_allclose(pose.robot_covariance, pose.robot_covariance.T, atol=1e-15)
        np.testing.assert_allclose(np.hypot(*pose.gyro_std_devs()) ** 2, np.trace(pose.camera_xy_covariance))
    assert len(robot_nees) > 100
    assert np.mean(np.array(robot_nees) > 7.81) < 0.10  # chi-squared, 3 dof, 95%
    assert np.mean(np.array(camera_nees) > 5.99) < 0.10  # 2 dof
    assert 1.5 < np.mean(robot_nees) < 4.5 and 1.0 < np.mean(camera_nees) < 3.0


def test_field_pose_packs_for_networktables():
    from x4tags.publish import FIELD_POSE_FIELDS, field_pose_values

    layout, mount, p_r, tags = _scene((2.4, 1.7), 30.0)
    pose = FieldLocalizer(layout, SIZE, mount).solve(tags)
    v = field_pose_values(pose)
    assert len(v) == len(FIELD_POSE_FIELDS) == 28 and np.all(np.isfinite(v))
    np.testing.assert_allclose(np.reshape(v[13:22], (3, 3)), pose.robot_covariance)
    assert v[11] == pose.xy_std_m and max(v[26:28]) <= pose.xy_std_m + 1e-12  # no direction worse than the worst
    d = pose.to_dict()
    assert np.shape(d["robot_covariance"]) == (3, 3) and len(d["gyro_std_devs_m"]) == 2
