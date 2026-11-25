"""End to end: replayed JPEG frames -> detector -> robot-frame results -> UDP / stdout / NetworkTables."""

import json
import socket
import time

import cv2
import numpy as np
import pytest
from synth import add_tag, blank, to_u8

import x4tags
from x4tags import app, config, publish
from x4tags.mount import Mount
from x4tags.publish import StdoutPublisher
from x4tags.results import FrameResult, TagResult


@pytest.fixture(scope="module")
def frames_dir(tmp_path_factory):
    d = tmp_path_factory.mktemp("frames")
    for i, (tag_id, lon) in enumerate([(5, 30), (5, 40), (9, -90)]):
        img = blank(seed=i)
        add_tag(img, tag_id, lon_deg=lon, lat_deg=3, distance_m=1.5)
        bgr = cv2.cvtColor(to_u8(img), cv2.COLOR_GRAY2BGR)
        cv2.imwrite(str(d / f"{i:03d}.jpg"), bgr, [cv2.IMWRITE_JPEG_QUALITY, 92])
    return d


def test_replay_to_udp(frames_dir):
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0))
    rx.settimeout(5)
    cfg = config.load(None)
    cfg.camera.fps = 0  # as fast as possible
    cfg.output.udp = f"127.0.0.1:{rx.getsockname()[1]}"
    cfg.mount = Mount(z_m=0.4, yaw_deg=90)  # camera turned to look out of the robot's left side
    assert app.run(cfg, replay=str(frames_dir)) == 0
    got = [json.loads(rx.recv(65536)) for _ in range(3)]
    assert [g["seq"] for g in got] == [0, 1, 2]
    assert [[t["id"] for t in g["tags"]] for g in got] == [[5], [5], [9]]
    t = got[0]["tags"][0]
    assert t["yaw_deg"] == pytest.approx(90 - 30, abs=0.5)  # 30 deg right of the camera's forward
    assert t["distance_m"] == pytest.approx(1.5, rel=0.02)
    assert t["position_m"][2] == pytest.approx(0.4 + 1.5 * np.sin(np.radians(3)), abs=0.03)


def test_stdout_line(capsys):
    frame = FrameResult(4, time.monotonic(), 2880, 1440, 9.0, 3.0,
                        [TagResult(1, 10.0, 2.0, np.zeros((4, 2)), False, False, False, 0, 80.0)])
    StdoutPublisher().publish(frame)
    line = json.loads(capsys.readouterr().out)
    assert line["seq"] == 4 and line["tags"][0]["id"] == 1 and "distance_m" not in line["tags"][0]


def test_networktables(frames_dir):
    ntcore = pytest.importorskip("ntcore")
    server = ntcore.NetworkTableInstance.create()
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    server.startServer("", "127.0.0.1", 0, port)
    try:
        table = server.getTable("x4tags")
        sub = table.getDoubleArrayTopic("tags").subscribe([], ntcore.PubSubOptions(pollStorage=50))
        status = table.getStringTopic("status").subscribe("", ntcore.PubSubOptions(pollStorage=10))
        version = table.getStringTopic("version").subscribe("", ntcore.PubSubOptions(pollStorage=10))
        cfg = config.load(None)
        cfg.camera.fps = 5
        cfg.output.nt_server = f"localhost:{port}"
        app.run(cfg, replay=str(frames_dir), loop=True, seconds=3.0)
        packed = [list(v.value) for v in sub.readQueue() if len(v.value)]
        assert packed, "no tags arrived"
        stride = len(publish.TAG_FIELDS)
        assert all(len(v) % stride == 0 for v in packed)
        assert {int(v[0]) for v in packed} <= {5, 9}
        d = packed[0][stride - 4]
        assert d == pytest.approx(1.5, rel=0.03)  # distance_m
        assert "ok" in [v.value for v in status.readQueue()]
        assert x4tags.__version__ in [v.value for v in version.readQueue()]
    finally:
        ntcore.NetworkTableInstance.destroy(server)


def test_bad_frames_do_not_stop_the_service(frames_dir, monkeypatch, tmp_path):
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0))
    rx.settimeout(5)
    calls = {"n": 0}
    real = app.X4TagDetector.detect

    def flaky(self, gray):
        calls["n"] += 1
        if calls["n"] == 2:
            raise RuntimeError("boom")
        return real(self, gray)

    monkeypatch.setattr(app.X4TagDetector, "detect", flaky)
    cfg = config.load(None)
    cfg.camera.fps = 0
    cfg.output.udp = f"127.0.0.1:{rx.getsockname()[1]}"
    assert app.run(cfg, replay=str(frames_dir)) == 0
    seqs = [json.loads(rx.recv(65536))["seq"] for _ in range(2)]
    assert seqs == [0, 2]  # frame 1 failed, the loop went on


def test_wrong_frame_shape_is_reported_not_fatal(tmp_path, caplog):
    cv2.imwrite(str(tmp_path / "flat.jpg"), np.zeros((1080, 1920), np.uint8))
    cfg = config.load(None)
    cfg.camera.fps, cfg.camera.reconnect_s = 0, 0.2
    assert app.run(cfg, replay=str(tmp_path), seconds=1.0) == 0
    assert "not the 2880x1440 picture" in caplog.text


@pytest.mark.parametrize("mode", ["equirect", "dual1080"])
def test_replay_with_a_field_layout_reports_the_robot_pose(tmp_path, mode):
    import math

    from test_localize import SIZE, render_from, room_layout

    from x4tags.field import FieldLayout
    from x4tags.mount import rotation_rpy

    data = room_layout()
    (tmp_path / "field.json").write_text(json.dumps(data))
    layout = FieldLayout.from_dict(data)
    mount = Mount(x_m=0.1, z_m=0.5)
    frames = tmp_path / "frames"
    frames.mkdir()
    truth = []
    for i, (x, y, yaw) in enumerate([(2.0, 1.5, 20.0), (3.5, 2.5, -60.0)]):
        r_fr = rotation_rpy(0, 0, math.radians(yaw))
        img = render_from(layout, r_fr @ mount.rotation, np.array([x, y, 0]) + r_fr @ mount.translation, seed=i)
        if mode == "dual1080":
            from synth import equirect_to_dual

            img = equirect_to_dual(img)
        cv2.imwrite(str(frames / f"{i}.jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 95])
        truth.append((x, y, yaw))
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0))
    rx.settimeout(5)
    cfg = config.load(None)
    cfg.camera.fps, cfg.camera.mode = 0, mode
    cfg.mount, cfg.tags.size_m, cfg.field.layout = mount, SIZE, str(tmp_path / "field.json")
    cfg.output.udp = f"127.0.0.1:{rx.getsockname()[1]}"
    assert app.run(cfg, replay=str(frames)) == 0
    for x, y, yaw in truth:
        fp = json.loads(rx.recv(65536))["field_pose"]
        assert fp["robot_xyz_m"][:2] == pytest.approx([x, y], abs=0.02)
        assert fp["robot_rpy_deg"][2] == pytest.approx(yaw, abs=0.3)
        assert len(fp["tags"]) >= 4
