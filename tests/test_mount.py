"""Camera frame -> WPILib robot frame."""

import numpy as np
import pytest

from x4tags.mount import Mount, yaw_pitch_deg

FORWARD, RIGHT, DOWN = np.array([0, 0, 1.0]), np.array([1, 0, 0.0]), np.array([0, 1, 0.0])


def test_level_camera_maps_axes():
    m = Mount()
    np.testing.assert_allclose(m.direction(FORWARD), [1, 0, 0], atol=1e-12)
    np.testing.assert_allclose(m.direction(RIGHT), [0, -1, 0], atol=1e-12)
    np.testing.assert_allclose(m.direction(DOWN), [0, 0, -1], atol=1e-12)


def test_yaw_turns_left_and_pitch_tips_down_like_wpilib():
    np.testing.assert_allclose(Mount(yaw_deg=90).direction(FORWARD), [0, 1, 0], atol=1e-12)
    yaw, pitch = yaw_pitch_deg(Mount(pitch_deg=20).direction(FORWARD))
    assert yaw == pytest.approx(0, abs=1e-9) and pitch == pytest.approx(-20)


def test_point_adds_the_mount_position():
    m = Mount(x_m=0.2, y_m=-0.1, z_m=0.5, yaw_deg=180)
    np.testing.assert_allclose(m.point(FORWARD * 2), [0.2 - 2, -0.1, 0.5], atol=1e-12)


def test_yaw_pitch_of_robot_directions():
    assert yaw_pitch_deg(np.array([1, 1, 0])) == pytest.approx((45, 0))
    assert yaw_pitch_deg(np.array([1, 0, 1]))[1] == pytest.approx(45)
