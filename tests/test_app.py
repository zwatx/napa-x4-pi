"""End to end: replayed JPEG frames -> detector -> robot-frame results -> UDP / stdout / NetworkTables."""

import json
import socket
import time

import cv2
import numpy as np
import pytest
from synth import add_tag, blank, to_u8

from x4tags import app, config
from x4tags.mount import Mount
from x4tags.publish import StdoutPublisher
from x4tags.results import FrameResult, TagResult


@pytest.fixture(scope="module")
def frames_dir(tmp_path_factory):
    d = tmp_path_factory.mktemp("frames")
    for i, (tag_id, lon) in enumerate([(5, 30), (5, 40), (9, -90)]):
        img = blank(seed=i)
        add_tag(img, tag_id, lon_deg=lon, lat_deg=3, distance_m=1.5)
        cv2.imwrite(str(d / f"{i:03d}.jpg"), cv2.cvtColor(to_u8(img), cv2.COLOR_GRAY2BGR), [cv2.IMWRITE_JPEG_QUALITY, 92])
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
                        [TagResult(1, 10.0, 2.0, np.zeros((4, 2)), False, False, 0, 80.0)])
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
        sub = server.getTable("x4tags").getIntegerArrayTopic("ids").subscribe([])
        cfg = config.load(None)
        cfg.camera.fps = 5
        cfg.output.nt_server = f"localhost:{port}"
        app.run(cfg, replay=str(frames_dir), loop=True, seconds=3.0)
        values = [v.value for v in sub.readQueue()] or [sub.get()]
        assert any(list(v) == [5] for v in values) or any(list(v) == [9] for v in values), values
    finally:
        ntcore.NetworkTableInstance.destroy(server)
