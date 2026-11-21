"""Measure the X4's latency: how long from something changing in front of it to the frame showing it arriving here.

A browser on this computer shows a big tag whose ID flips every couple of seconds; the page reports each flip once
it has been drawn. Point the X4 at the screen (the tag a metre or so away is plenty). Each flip's latency is the
arrival time of the first frame showing the new ID minus the flip time, which also includes the screen's own delay
(the compositor and panel: typically 1-3 refreshes, 17-50 ms at 60 Hz), so treat the result as an upper bound;
subtract your screen's delay if you know it. Use the median as ``[camera] latency_ms``.

    python -m x4tags.check_latency [--mode dual1080]   # then open http://localhost:8765 here, full screen
"""

from __future__ import annotations

import argparse
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
from napa_vision.tags import png_bytes, tag_image

from .capture import X4Webcam
from .config import MODE_SIZES
from .detector import X4TagDetector

PAGE = """<!doctype html><html><head><meta charset="utf-8"><title>x4tags latency</title>
<style>html,body{margin:0;height:100%;background:#fff}
img{position:fixed;inset:0;margin:auto;height:90vh;image-rendering:pixelated}</style></head><body>
<img id="A" src="/tag/A.png"><img id="B" src="/tag/B.png" style="visibility:hidden">
<script>
// Both pictures are decoded up front and only their visibility is toggled, so a flip shows on the next frame.
const imgs = [document.getElementById("A"), document.getElementById("B")]; let k = 0;
Promise.all(imgs.map(im => im.decode())).then(() => setInterval(() => {
  imgs[k].style.visibility = "hidden"; k = 1 - k; imgs[k].style.visibility = "visible";
  // two animation frames: the change has been handed to the compositor before it's reported
  requestAnimationFrame(() => requestAnimationFrame(() => fetch("/flip?to=" + imgs[k].id, {method: "POST"})));
}, PERIOD_MS));
</script></body></html>"""


def latencies(flips: list[tuple[float, int]], frames: list[tuple[float, set[int]]]) -> list[float]:
    """For each flip (time, new id), the delay until the first frame after it that shows the new id but not the
    old one; flips that never show up before the next flip are skipped. Times in seconds; returns ms."""
    out = []
    for i, (t_flip, new_id) in enumerate(flips):
        t_next = flips[i + 1][0] if i + 1 < len(flips) else float("inf")
        for t, ids in frames:
            if t_flip < t < t_next and new_id in ids and len(ids) == 1:
                out.append((t - t_flip) * 1e3)
                break
    return out


def make_server(ids: dict[str, int], period_s: float, port: int, flips: list) -> ThreadingHTTPServer:
    """The page and tag images on localhost:port; each reported flip is appended to ``flips`` as (time, new id)."""
    pngs = {k: png_bytes(tag_image(v, cell_px=40, quiet_zone_cells=2)) for k, v in ids.items()}  # 480 px
    page = PAGE.replace("PERIOD_MS", str(int(period_s * 1000))).encode()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path.startswith("/tag/") and self.path[5] in pngs:
                body, kind = pngs[self.path[5]], "image/png"
            elif self.path == "/":
                body, kind = page, "text/html"
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", kind)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            t = time.monotonic()
            if self.path.startswith("/flip?to="):
                flips.append((t, ids[self.path.rsplit("=", 1)[1]]))
            self.send_response(204)
            self.end_headers()

        def log_message(self, *args):
            pass

    return ThreadingHTTPServer(("127.0.0.1", port), Handler)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m x4tags.check_latency", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--flips", type=int, default=20)
    ap.add_argument("--period", type=float, default=2.0, help="seconds between flips")
    ap.add_argument("--ids", default="0,1", help="the two tag IDs to show")
    ap.add_argument("--mode", choices=sorted(MODE_SIZES), default="equirect", help="the X4 picture to measure")
    a = ap.parse_args(argv)
    id_a, id_b = (int(x) for x in a.ids.split(","))
    flips: list[tuple[float, int]] = []
    frames: list[tuple[float, set[int]]] = []
    server = make_server({"A": id_a, "B": id_b}, a.period, a.port, flips)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"open http://localhost:{a.port} in a browser on this computer, full screen, and point the X4 at it")
    det = X4TagDetector(near_scale=0, seam_px=0, mode=a.mode)  # a big tag in front: no extra searches
    with X4Webcam(a.device, *MODE_SIZES[a.mode]) as cam:
        try:
            if cam.size != MODE_SIZES[a.mode]:
                print(f"the camera sends {cam.size[0]}x{cam.size[1]}, not mode {a.mode}'s {MODE_SIZES[a.mode]}")
                return 1
            while len(flips) < a.flips + 1:
                t, gray = cam.read(timeout=5.0)
                frames.append((t, {x.tag_id for x in det.detect(gray)} & {id_a, id_b}))
                if len(frames) % 30 == 0:
                    seen = frames[-1][1]
                    print(f"  {len(flips)} flips so far; tag now {sorted(seen) or 'not seen'}", flush=True)
        finally:
            server.shutdown()
    ms = latencies(flips[1:], frames)  # the first flip may come before the camera was running
    if not ms:
        print("no flips seen by the camera: is the tag in view and big enough?")
        return 1
    m = np.array(ms)
    print(f"{len(m)} flips: latency median {np.median(m):.0f} ms, 10-90% {np.percentile(m, 10):.0f}-{np.percentile(m, 90):.0f} ms "
          f"(an upper bound: includes the screen's own delay)")
    print(f"set [camera] latency_ms = {np.median(m):.0f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
