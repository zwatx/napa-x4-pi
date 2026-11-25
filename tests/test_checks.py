"""The analysis behind the on-camera checks (latency, stabilization), on made-up traces."""

import cv2
import numpy as np

from x4tags.check_latency import along_track, fit_latency, motion_x
from x4tags.check_stabilization import analyze, rotation_between
from x4tags.equirect import pixel_to_ray, ray_to_pixel
from x4tags.mount import rotation_rpy


def test_latency_fit_finds_a_known_delay_from_a_sliding_tag():
    # a tag sliding across a screen 0.6 m away, 0.2 m above the camera; frames 120 ms late, 0.15 px of corner noise
    rng = np.random.default_rng(0)
    motion = dict(t0=100.0, period_s=3.0, amp_px=1000.0, cx_px=1280.0)
    capture = 100.0 + np.arange(900) / 29.97
    x, _ = motion_x(capture, motion)
    rays = np.stack([(x - 1280) * 0.000233, np.full_like(x, -0.2), np.full_like(x, 0.6)], 1)
    rays += rng.normal(0, 0.15 / 458 * 0.6, rays.shape)
    lat, jitter, n = fit_latency(capture + 0.120, along_track(rays), motion)
    assert abs(lat - 0.120) < 0.001 and jitter < 1.5 and n > 400
    late = capture + 0.120 + rng.uniform(0, 0.025, len(capture))  # uneven delivery: scatter shows, mean delay found
    lat, jitter, _ = fit_latency(late, along_track(rays), motion)
    assert abs(lat - 0.1325) < 0.0015 and jitter > 4  # the mean delay


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
        rw, rh = int(rng.integers(4, 30)), int(rng.integers(4, 30))
        cv2.rectangle(img, (x, y), (x + rw, y + rh), int(rng.integers(0, 256)), -1)
    r_true = rotation_rpy(np.radians(2.0), np.radians(-1.0), np.radians(30.0))
    v, u = np.mgrid[0:h, 0:w].astype(np.float64)
    src = ray_to_pixel(pixel_to_ray(u.ravel(), v.ravel(), w, h) @ r_true, w, h).reshape(h, w, 2).astype(np.float32)
    turned = cv2.remap(img, src[..., 0], src[..., 1], cv2.INTER_LINEAR, borderMode=cv2.BORDER_WRAP)
    r, inliers = rotation_between(img, turned)
    err = np.degrees(np.arccos(np.clip((np.trace(r.T @ r_true) - 1) / 2, -1, 1)))
    assert inliers > 200 and err < 0.15


def test_latency_page_serves_the_tag_and_records_its_motion():
    import json
    import threading
    import time
    import urllib.request

    from x4tags.check_latency import make_server

    motion = {}
    server = make_server(40, 3.0, 0.5, 0, motion)
    port = server.server_address[1]
    threading.Thread(target=server.serve_forever, daemon=True).start()
    url = f"http://127.0.0.1:{port}"
    try:
        page = urllib.request.urlopen(url + "/").read().decode()
        assert "const img" in page and "P = 3000" in page and "Y = 0.5" in page and "requestAnimationFrame" in page
        assert urllib.request.urlopen(url + "/tag.png").read()[:8] == b"\x89PNG\r\n\x1a\n"
        server_ms = float(urllib.request.urlopen(urllib.request.Request(url + "/sync", method="POST")).read())
        assert abs(server_ms - time.time() * 1e3) < 1000
        body = json.dumps(dict(t0_ms=time.time() * 1e3 - 500, rtt_ms=8.5, period_ms=3000, amp_px=1000, cx_px=1280))
        urllib.request.urlopen(urllib.request.Request(url + "/motion", data=body.encode(), method="POST"))
        assert abs(motion["t0"] - (time.monotonic() - 0.5)) < 0.05  # on time.monotonic()'s clock
        assert motion["period_s"] == 3.0 and motion["amp_px"] == 1000 and motion["rtt_ms"] == 8.5
    finally:
        server.shutdown()


def test_latency_fit_needs_both_directions_and_avoids_the_mirror():
    rng = np.random.default_rng(1)
    motion = dict(t0=100.0, period_s=3.0, amp_px=1000.0, cx_px=1280.0)
    capture = 100.0 + np.arange(900) / 29.97
    x, ph = motion_x(capture, motion)
    rays = np.stack([(x - 1280) * 0.000233, np.full_like(x, -0.2), np.full_like(x, 0.6)], 1)
    rays += rng.normal(0, 0.15 / 458 * 0.6, rays.shape)
    one_way = ph < 0.5  # the tag seen moving right only: latency and screen shape can't be told apart
    assert fit_latency(capture[one_way] + 0.12, along_track(rays[one_way]), motion) is None
    fast = dict(motion, period_s=0.5)  # half a period on, the path is mirrored: the search stays short of it
    x, _ = motion_x(capture, fast)
    rays = np.stack([(x - 1280) * 0.000233, np.full_like(x, -0.2), np.full_like(x, 0.6)], 1)
    lat, _, _ = fit_latency(capture + 0.12, along_track(rays), fast)
    assert abs(lat - 0.12) < 0.002
