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
