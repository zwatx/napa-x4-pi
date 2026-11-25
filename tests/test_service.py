"""The service's edges: settings it refuses, outputs that fail, systemd, and replaying recorded MJPEG."""

import json
import os
import socket

import cv2
import numpy as np
import pytest
from synth import add_tag, blank, to_u8

from x4tags import app, config, publish
from x4tags.detector import X4TagDetector
from x4tags.field import FieldLayout


@pytest.fixture(scope="module")
def mjpeg(tmp_path_factory):
    """Three frames as one MJPEG file, like a recording of the camera."""
    path = tmp_path_factory.mktemp("rec") / "rec.mjpg"
    with open(path, "wb") as f:
        for i, (tag_id, lon) in enumerate([(5, 30), (5, 40), (9, -90)]):
            img = blank(seed=i)
            add_tag(img, tag_id, lon_deg=lon, lat_deg=3, distance_m=1.5)
            f.write(cv2.imencode(".jpg", to_u8(img), [cv2.IMWRITE_JPEG_QUALITY, 92])[1].tobytes())
    return path


def _udp():
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0))
    rx.settimeout(5)
    return rx


@pytest.mark.parametrize("text,complaint", [
    ("[camera]\nfps = -1\n", "fps"),
    ("[camera]\nreconnect_s = -2\n", "reconnect_s"),
    ("[output]\nudp = \"10.0.0.2\"\n", "udp"),
    ("[tags]\nids = [1, \"two\"]\n", "ids"),
    ("[detector]\nextra = { min_tag_px = 20 }\n", "own key"),
    ("[detector]\nnear_scale = 2\n", "near_scale"),
    ("[field]\nmin_tags = 0\n", "min_tags"),
])
def test_settings_that_would_fail_later_are_refused_up_front(tmp_path, text, complaint):
    p = tmp_path / "c.toml"
    p.write_text(text)
    with pytest.raises(ValueError, match=complaint):
        config.load(str(p))


@pytest.mark.parametrize("data,complaint", [
    ({"tags": []}, "no tags"),
    ({"field": {}}, "not a WPILib"),
    ({"tags": [{"ID": 1}]}, "tag 0"),
    ({"tags": [{"ID": 1, "pose": {"translation": {"x": 0, "y": 0, "z": 0},
                                  "rotation": {"quaternion": {"W": 1, "X": 0, "Y": 0, "Z": 0}}}}] * 2}, "twice"),
])
def test_a_bad_field_layout_says_what_is_wrong(data, complaint):
    with pytest.raises(ValueError, match=complaint):
        FieldLayout.from_dict(data)


def test_bad_settings_and_missing_files_end_with_one_line_and_78(tmp_path, mjpeg, caplog):
    p = tmp_path / "c.toml"
    p.write_text("[camera]\nfps = -1\n")
    assert app.main(["--config", str(p)]) == 78
    p.write_text(f"[field]\nlayout = \"{tmp_path / 'nope.json'}\"\n")
    assert app.main(["--config", str(p), "--replay", str(mjpeg)]) == 78
    p.write_text("[camera]\nfps = 0\n")
    assert app.main(["--config", str(p), "--replay", str(tmp_path / "missing.mjpg")]) == 78
    assert "nope.json" in caplog.text and "missing.mjpg" in caplog.text
    assert "Traceback" not in caplog.text


def test_replays_a_recorded_mjpeg(mjpeg):
    rx = _udp()
    cfg = config.load(None)
    cfg.camera.fps = 0
    cfg.output.udp = f"127.0.0.1:{rx.getsockname()[1]}"
    assert app.run(cfg, replay=str(mjpeg)) == 0
    got = [json.loads(rx.recv(65536)) for _ in range(3)]
    assert [[t["id"] for t in g["tags"]] for g in got] == [[5], [5], [9]]


def test_one_failing_output_does_not_stop_the_others(mjpeg, monkeypatch):
    class Broken(publish.Publisher):
        def publish(self, frame):
            raise BrokenPipeError("stdout went away")

        def camera_connected(self, connected):
            raise BrokenPipeError("stdout went away")

    rx = _udp()
    real = publish.make_publishers
    monkeypatch.setattr(app, "make_publishers", lambda out: [Broken(), *real(out)])
    cfg = config.load(None)
    cfg.camera.fps = 0
    cfg.output.udp = f"127.0.0.1:{rx.getsockname()[1]}"
    assert app.run(cfg, replay=str(mjpeg)) == 0
    assert [json.loads(rx.recv(65536))["seq"] for _ in range(3)] == [0, 1, 2]


def test_udp_to_a_name_that_does_not_resolve_yet_keeps_going(mjpeg):
    cfg = config.load(None)
    cfg.camera.fps = 0
    cfg.output.udp = "no-such-robot.invalid:5800"
    assert app.run(cfg, replay=str(mjpeg)) == 0


def test_tells_systemd_it_is_ready_and_alive(mjpeg, tmp_path, monkeypatch):
    path = str(tmp_path / "notify")
    rx = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    rx.bind(path)
    rx.settimeout(2)
    monkeypatch.setenv("NOTIFY_SOCKET", path)
    cfg = config.load(None)
    cfg.camera.fps = 0
    assert app.run(cfg, replay=str(mjpeg)) == 0
    got = []
    try:
        while True:
            got.append(rx.recv(256).decode())
    except TimeoutError:
        pass
    assert got[0] == "READY=1" and "WATCHDOG=1" in got and got[-1] == "STOPPING=1"
    rx.close()
    os.unlink(path)


def test_a_failed_side_search_keeps_the_full_frame_tags(mjpeg):
    frames = app.ReplaySource(str(mjpeg), fps=0)
    _, gray = frames.read()
    with X4TagDetector() as det:
        want = sorted(t.tag_id for t in det.detect(gray))

        def boom(*a):
            raise RuntimeError("near views broke")

        det._side_search = boom
        assert sorted(t.tag_id for t in det.detect(gray)) == want == [5]


def test_a_tags_centre_follows_new_corners():
    from x4tags.detector import X4Tag
    from x4tags.equirect import pixel_to_ray

    c = np.array([[100.0, 700], [140, 700], [140, 740], [100, 740]])
    t = X4Tag(1, c, pixel_to_ray(c[:, 0], c[:, 1], 2880, 1440), 0, 100.0, 0.0, False, False)
    before = t.center_ray.copy()
    assert t.center_ray is t.center_ray  # worked out once
    c2 = c + [400, 0]
    t.rays = pixel_to_ray(c2[:, 0], c2[:, 1], 2880, 1440)
    assert np.degrees(np.arccos(np.clip(before @ t.center_ray, -1, 1))) > 40
