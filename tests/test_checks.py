"""The analysis behind the on-camera checks (latency, stabilization), on made-up traces."""

import cv2
import numpy as np

from x4tags.check_latency import latencies
from x4tags.check_stabilization import analyze, rotation_between
from x4tags.equirect import pixel_to_ray, ray_to_pixel
from x4tags.mount import rotation_rpy


def test_latency_matches_each_flip_to_the_first_frame_showing_it():
    flips = [(1.0, 1), (3.0, 0), (5.0, 1)]
    frames = [(t, {1} if (1.12 <= t < 3.12 or t >= 5.12) else {0}) for t in np.arange(0, 7, 1 / 30)]
    ms = latencies(flips, frames)
    assert len(ms) == 3 and all(115 <= m <= 160 for m in ms)


def _trace(kind, axis=(0.0, 1.0, 0.0), moved=90.0):
    """Rotation of each frame relative to the last while the camera is turned ``moved`` degrees at 3-3.4 s about
    ``axis`` (camera axes), then left alone: body-fixed, creeping back with a 5 s time constant, or locked."""
    t = np.arange(0, 30, 1 / 15)
    left = moved * (1 - np.clip((t - 3.0) / 0.4, 0, 1))  # body: how far it still is from where it ends up
    if kind == "creep":
        left = left + np.where(t > 3.4, 4.0 * np.exp(-(t - 3.4) / 5.0), 0.0)
    elif kind == "locked":
        left = np.where((t > 3.0) & (t < 3.4), 3.0, 0.0)  # a brief wobble, then back
    return t, np.outer(left, np.asarray(axis) / np.linalg.norm(axis))


def test_stabilization_verdicts():
    assert analyze(*_trace("fixed"), 90)[0].startswith("body-fixed")
    text, numbers = analyze(*_trace("creep"), 90)
    assert text.startswith("follows the body at once but creeps") and 3.0 < numbers["creep_deg"] < 4.0
    assert 4.5 < numbers["time_constant_s"] < 5.5
    assert analyze(*_trace("locked"), 90)[0].startswith("heading-locked")
    t, w = _trace("fixed")
    assert "still moving" in analyze(t[t < 8], w[t < 8], 90)[0]  # let go too late to see it settle


def test_stabilization_tilt_counts_only_horizontal_axes():
    assert analyze(*_trace("fixed", axis=(1.0, 0.0, 0.3), moved=15), 15, "tilt")[0].startswith("body-fixed")
    # the body tilted but the picture only turned about the vertical: the horizon is held level
    assert analyze(*_trace("fixed", moved=15), 15, "tilt")[0].startswith("horizon-locked")


def test_rotation_between_recovers_a_known_rotation():
    rng = np.random.default_rng(1)
    h, w = 360, 720
    img = np.full((h, w), 128, np.uint8)  # random shapes: corners for ORB to match
    for _ in range(400):
        x, y = int(rng.integers(0, w)), int(rng.integers(40, h - 40))
        cv2.rectangle(img, (x, y), (x + int(rng.integers(4, 30)), y + int(rng.integers(4, 30))), int(rng.integers(0, 256)), -1)
    r_true = rotation_rpy(np.radians(2.0), np.radians(-1.0), np.radians(30.0))
    v, u = np.mgrid[0:h, 0:w].astype(np.float64)
    src = ray_to_pixel(pixel_to_ray(u.ravel(), v.ravel(), w, h) @ r_true, w, h).reshape(h, w, 2).astype(np.float32)
    turned = cv2.remap(img, src[..., 0], src[..., 1], cv2.INTER_LINEAR, borderMode=cv2.BORDER_WRAP)
    r, inliers = rotation_between(img, turned)
    err = np.degrees(np.arccos(np.clip((np.trace(r.T @ r_true) - 1) / 2, -1, 1)))
    assert inliers > 200 and err < 0.15


def test_latency_page_serves_tags_and_records_flips():
    import threading
    import urllib.request

    from x4tags.check_latency import make_server

    flips = []
    server = make_server({"A": 3, "B": 4}, 2.0, 0, flips)
    port = server.server_address[1]
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        page = urllib.request.urlopen(f"http://127.0.0.1:{port}/").read().decode()
        assert "}, 2000));" in page and "im.decode()" in page
        png = urllib.request.urlopen(f"http://127.0.0.1:{port}/tag/B.png").read()
        assert png[:8] == b"\x89PNG\r\n\x1a\n"
        urllib.request.urlopen(urllib.request.Request(f"http://127.0.0.1:{port}/flip?to=B", method="POST"))
        assert len(flips) == 1 and flips[0][1] == 4
    finally:
        server.shutdown()
