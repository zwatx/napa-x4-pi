"""Measure the X4's latency: from when something happens in front of it to when the frame showing it arrives here.

A browser shows a tag sliding back and forth at a steady speed. Each frame's view of the tag says where the tag was
when the frame was taken, so when that was; the latency is the delay that lines this up with the frame times the
service uses (V4L2 stamps steadied by its frame clock), fitted over every frame. Timing a sudden change instead (a tag
flipping) comes out about half a frame too long, since the change lands anywhere in the camera's frame interval and
only shows from the next frame on. The page times the motion by its own clock, synced to this computer's over the
quickest of 30 round trips, so it can run on another computer (a laptop next to the robot's Pi) over a slow or
uneven network, to within half that round trip.

The result includes the screen's own delay (the browser, compositor and panel: one or two refreshes plus a few ms,
roughly 15-25 ms at 120 Hz and 20-40 ms at 60 Hz); subtract yours and put the rest in ``[camera] latency_ms``. It
also reports how much single frames' times scatter around the fit.

    python -m x4tags.check_latency [--mode dual1080]   # then open http://localhost:8765 here, full screen
    python -m x4tags.check_latency --host 0.0.0.0      # or open http://<this computer>:8765 on another one
"""

from __future__ import annotations

import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
from napa_vision.tags import png_bytes, tag_image

from .capture import X4Webcam
from .config import MODE_SIZES
from .detector import X4TagDetector

PAGE = """<!doctype html><html><head><meta charset="utf-8"><title>x4tags latency</title>
<style>html,body{margin:0;height:100%;background:#fff;overflow:hidden}
img{position:fixed;left:0;top:0;width:400px;height:400px;image-rendering:pixelated;will-change:transform}</style>
</head><body><img id="tag" src="/tag.png"><script>
const img = document.getElementById("tag"), P = PERIOD_MS, Y = Y_FRACTION;
const now = () => performance.timeOrigin + performance.now();  // ms
async function sync() {  // the server's clock minus ours, from the quickest of 30 round trips
  let best = null;
  for (let i = 0; i < 30; i++) {
    const t0 = now(), ts = parseFloat(await (await fetch("/sync", {method: "POST"})).text()), t1 = now();
    if (!best || t1 - t0 < best.rtt) best = {rtt: t1 - t0, off: ts - (t0 + t1) / 2};
  }
  return best;
}
document.body.onclick = () => document.documentElement.requestFullscreen();  // or start the browser full screen
const full = () => document.fullscreenElement || (innerWidth >= screen.width - 2 && innerHeight >= screen.height - 2);
img.decode().then(sync).then(clock => {
  let run = null;  // the current sweep; any resize starts a new one, and only the last one's frames count
  const start = () => {
    const A = Math.min(1000, innerWidth / 2 - 220);
    if (!full() || A < 100) { run = null; return; }
    run = {A, CX: innerWidth / 2, CY: innerHeight * Y, T0: performance.now()};
    fetch("/motion", {method: "POST", body: JSON.stringify({t0_ms: performance.timeOrigin + run.T0 + clock.off,
      rtt_ms: clock.rtt, period_ms: P, amp_px: run.A, cx_px: run.CX})});
  };
  addEventListener("resize", () => setTimeout(start, 300));
  start();
  // Placed by each animation frame's own time, so a late frame still shows the tag where it should be then.
  const step = ts => {
    if (!run) { if (full()) start(); }
    else {
      const ph = (((ts - run.T0) / P) % 1 + 1) % 1, tri = ph < 0.5 ? 4 * ph - 1 : 3 - 4 * ph;
      img.style.transform = `translate(${run.CX + run.A * tri - 200}px, ${run.CY - 200}px)`;
    }
    requestAnimationFrame(step);
  };
  requestAnimationFrame(step);
});
</script></body></html>"""


def motion_x(t, motion: dict) -> tuple[np.ndarray, np.ndarray]:
    """(screen x in CSS px, phase in [0, 1)) of the sliding tag at times ``t`` (seconds, time.monotonic()'s clock)."""
    ph = np.mod((np.asarray(t, float) - motion["t0"]) / motion["period_s"], 1.0)
    return motion["cx_px"] + motion["amp_px"] * np.where(ph < 0.5, 4 * ph - 1, 3 - 4 * ph), ph


def along_track(rays) -> np.ndarray:
    """Angle (radians) of each direction along the great circle through them all (a line on a screen is one)."""
    r = np.asarray(rays, float)
    r = r / np.linalg.norm(r, axis=1, keepdims=True)
    normal = np.linalg.svd(r)[2][2]  # the plane through the camera and the line
    e1 = r.mean(0) - normal * (r.mean(0) @ normal)
    e1 /= np.linalg.norm(e1)
    return np.arctan2(r @ np.cross(normal, e1), r @ e1)


def fit_latency(times, angles, motion: dict, lo_s: float = 0.0, hi_s: float = 0.4):
    """(latency s, robust std of single frames' timing ms, frames used): the mean delay from when each frame was taken
    (where the tag was, through a least-squares cubic from screen x to the tag's angle, the latency picked by the
    median residual) to its time in ``times``. Frames near the turns, where the tag stops, are left out. None with too
    few frames going each way, or when the best fit is at the edge of the range searched."""
    times, angles = np.asarray(times, float), np.asarray(angles, float)
    speed = 4 * motion["amp_px"] / motion["period_s"]  # px/s
    hi_s = min(hi_s, motion["period_s"] / 2 - 0.05)  # half a period on, the tag runs its path mirrored: same fit

    def cost(lat):
        x, ph = motion_x(times - lat, motion)
        right, left = np.abs(ph - 0.25) < 0.2, np.abs(ph - 0.75) < 0.2
        keep = right | left
        # One direction alone can't tell latency from the screen's shape; with both seen, each is about half (a wrong
        # latency can push a few frames of one direction past a turn, so a handful isn't enough).
        if min(right.sum(), left.sum()) < max(15, 0.25 * keep.sum()):
            return None
        c = np.polyfit(x[keep] - motion["cx_px"], angles[keep], 3)
        r = angles[keep] - np.polyval(c, x[keep] - motion["cx_px"])
        return float(np.median(np.abs(r))), c, keep, x - motion["cx_px"]

    best = None
    for grid in (np.arange(lo_s, hi_s, 0.002), None):
        if grid is None:  # refine around the coarse best
            if best is None:
                return None
            grid = np.arange(best[0] - 0.003, best[0] + 0.003, 0.0001)
        for lat in grid:
            f = cost(lat)
            if f is not None and (best is None or f[0] < best[1][0]):
                best = (float(lat), f)
    if best is None or not lo_s + 0.003 < best[0] < hi_s - 0.003:
        return None
    lat, (_, c, keep, x) = best
    _, ph = motion_x(times[keep] - lat, motion)
    rate = np.polyval(np.polyder(c), x[keep]) * np.where(ph < 0.5, speed, -speed)  # rad/s along the track
    taken = (angles[keep] - np.polyval(c, x[keep])) / rate * 1e3  # ms after times - lat that each frame was taken
    near = np.abs(taken - np.median(taken)) < 50  # leave out misreads
    # The robust fit centres on the typical frame; shifting by the mean gives the mean delay, which is what frame
    # times that keep the arrivals' average (the frame clock, or none) need.
    return lat - float(taken[near].mean()) / 1e3, float(1.4826 * np.median(np.abs(taken - np.median(taken)))), int(keep.sum())


def make_server(tag_id: int, period_s: float, y_fraction: float, port: int, motion: dict,
                host: str = "127.0.0.1") -> ThreadingHTTPServer:
    """The page and tag on host:port. When the page starts the tag moving it posts how; that goes into ``motion``,
    with its start time ``t0`` on time.monotonic()'s clock and ``runs``, how many times it has started (again after
    any resize or reload)."""
    png = png_bytes(tag_image(tag_id, cell_px=40, quiet_zone_cells=1))  # 400 px, as on the page
    page = PAGE.replace("PERIOD_MS", str(int(period_s * 1000))).replace("Y_FRACTION", repr(float(y_fraction))).encode()

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"  # keep-alive: a sync round trip that opens a connection would be lopsided

        def do_GET(self):
            if self.path == "/":
                body, kind = page, "text/html"
            elif self.path == "/tag.png":
                body, kind = png, "image/png"
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            body = b""
            if self.path == "/sync":
                body = repr(time.time() * 1e3).encode()
            elif self.path == "/motion":
                m = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))))
                wall_to_mono = time.monotonic() - time.time()
                motion.update(t0=m["t0_ms"] / 1e3 + wall_to_mono, period_s=m["period_ms"] / 1e3, amp_px=m["amp_px"],
                              cx_px=m["cx_px"], rtt_ms=m["rtt_ms"], runs=motion.get("runs", 0) + 1)
            self.send_response(200)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Content-Type", "text/plain")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass

    return ThreadingHTTPServer((host, port), Handler)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m x4tags.check_latency", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--host", default="127.0.0.1", help="address to serve the page on (0.0.0.0: any computer)")
    ap.add_argument("--seconds", type=float, default=30.0, help="how long to watch the tag move")
    ap.add_argument("--period", type=float, default=3.0, help="seconds for the tag to go across and back")
    ap.add_argument("--y", type=float, default=0.5, help="the tag's height on the page, 0 (top) to 1 (bottom)")
    ap.add_argument("--id", type=int, default=40, help="the tag ID to show (not one on your walls)")
    ap.add_argument("--mode", choices=sorted(MODE_SIZES), default="equirect", help="the X4 picture to measure")
    a = ap.parse_args(argv)
    motion: dict = {}
    server = make_server(a.id, a.period, a.y, a.port, motion, a.host)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    where = "this computer" if a.host.startswith("127.") else "any computer that can reach this one"
    print(f"open (or reload) http://{'localhost' if a.host.startswith('127.') else '<this computer>'}:{a.port} in a "
          f"browser on {where}, click it to go full screen, and point the X4 at it")
    # Every near view, every frame: a big tag on a screen above the horizon can come out too bent for the full-frame
    # search, and found only in a near view.
    det = X4TagDetector(seam_px=0, near_per_frame=0, min_tag_px=16, mode=a.mode)
    times, rays = [], []
    with X4Webcam(a.device, *MODE_SIZES[a.mode]) as cam:
        try:
            if cam.size != MODE_SIZES[a.mode]:
                print(f"the camera sends {cam.size[0]}x{cam.size[1]}, not mode {a.mode}'s {MODE_SIZES[a.mode]}")
                return 1
            while "t0" not in motion:
                cam.read(timeout=5.0)  # until the page starts the tag moving, full screen
            run, t_end = 0, 0.0
            while time.monotonic() < t_end or run != motion["runs"]:
                if run != motion["runs"]:  # (re)started: only frames of the latest sweep count
                    run, times, rays = motion["runs"], [], []
                    t_end = time.monotonic() + a.seconds
                    print(f"tag moving (browser clock synced to within {motion['rtt_ms'] / 2:.1f} ms); watching "
                          f"{a.seconds:.0f} s", flush=True)
                t, gray = cam.read(timeout=5.0)
                seen = [x for x in det.detect(gray) if x.tag_id == a.id]
                if seen and t > motion["t0"] + 0.5:
                    times.append(t)
                    rays.append(seen[0].center_ray)
            clock_on = cam.clock.locked
        finally:
            server.shutdown()
    fit = fit_latency(times, along_track(rays), motion) if len(rays) >= 30 else None
    if fit is None:
        print(f"no clear fit from the {len(rays)} frames that saw the tag: is it in view, big enough, and seen moving "
              f"both ways?")
        return 1
    lat, jitter, n = fit
    print(f"latency {lat * 1e3:.0f} ms over {n} frames (includes the screen's own delay); single frames' times scatter "
          f"{jitter:.1f} ms around it (frame clock {'on' if clock_on else 'off'})")
    print(f"set [camera] latency_ms = {lat * 1e3:.0f} minus your screen's delay")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
