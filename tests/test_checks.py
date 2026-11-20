"""The analysis behind the on-camera checks (latency, stabilization), on made-up traces."""

import numpy as np

from x4tags.check_latency import latencies
from x4tags.check_stabilization import verdict


def test_latency_matches_each_flip_to_the_first_frame_showing_it():
    flips = [(1.0, 1), (3.0, 0), (5.0, 1)]
    frames = [(t, {1} if (1.12 <= t < 3.12 or t >= 5.12) else {0}) for t in np.arange(0, 7, 1 / 30)]
    ms = latencies(flips, frames)
    assert len(ms) == 3 and all(115 <= m <= 160 for m in ms)


def _trace(kind):
    """Bearing of a tag while the body turns 90 degrees at 3 s: body-fixed, heading-locked or lagging picture."""
    t = np.arange(0, 8, 1 / 30)
    body = np.clip((t - 3.0) / 0.4, 0, 1) * 90.0
    if kind == "fixed":
        seen = 10.0 - body
    elif kind == "locked":
        seen = 10.0 - np.where((t > 3.0) & (t < 3.4), 5.0, 0.0)  # a brief wobble, then back
    else:  # the picture follows the body with a 1.5 s lag
        follow = np.where(t > 3.0, 90.0 * (1 - np.exp(-(t - 3.0) / 1.5)), 0.0)
        seen = 10.0 - follow
    return t, seen


def test_stabilization_verdicts():
    assert verdict(*_trace("fixed"), turned_deg=90)[0].startswith("body-fixed")
    assert verdict(*_trace("locked"), turned_deg=90)[0].startswith("heading-locked")
    assert verdict(*_trace("lagging"), turned_deg=90)[0].startswith("smoothed")


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
        assert "setInterval(flip, 2000)" in page
        png = urllib.request.urlopen(f"http://127.0.0.1:{port}/tag/B.png").read()
        assert png[:8] == b"\x89PNG\r\n\x1a\n"
        urllib.request.urlopen(urllib.request.Request(f"http://127.0.0.1:{port}/flip?to=B", method="POST"))
        assert len(flips) == 1 and flips[0][1] == 4
    finally:
        server.shutdown()
