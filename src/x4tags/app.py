"""The coprocessor loop: camera -> X4TagDetector -> robot-frame results -> publishers, forever.

    x4tags --config /etc/x4tags.toml             # what the systemd service runs
    x4tags --print                               # defaults, JSON lines on stdout
    x4tags --replay frames.mjpg --print          # recorded MJPEG (or a folder of images) instead of the camera
"""

from __future__ import annotations

import argparse
import glob
import logging
import os
import signal
import time

import cv2
import numpy as np

from . import config as config_mod
from .capture import X4Webcam
from .detector import X4TagDetector
from .publish import make_publishers
from .results import FrameResult, tag_results

log = logging.getLogger("x4tags")


class ReplaySource:
    """Frames from a file of concatenated JPEGs (an MJPEG capture) or a folder of images, at ``fps`` (0: as fast as
    possible), looping if asked. Same read() contract as X4Webcam."""

    def __init__(self, path: str, fps: float = 30.0, loop: bool = False, reduce: int = 1):
        self.flag = {1: cv2.IMREAD_GRAYSCALE, 2: cv2.IMREAD_REDUCED_GRAYSCALE_2, 4: cv2.IMREAD_REDUCED_GRAYSCALE_4}[reduce]
        if os.path.isdir(path):
            files = sorted(f for f in glob.glob(os.path.join(path, "*")) if f.lower().endswith((".jpg", ".jpeg", ".png")))
            self.buffers = [np.fromfile(f, np.uint8) for f in files]
        else:
            self.buffers = list(_split_jpegs(open(path, "rb").read()))
        if not self.buffers:
            raise ValueError(f"no frames in {path}")
        self.period = 1.0 / fps if fps > 0 else 0.0
        self.loop, self.i, self.next_t = loop, 0, time.monotonic()
        self.device, self.size, self.dropped, self.last_decode_ms = path, None, 0, 0.0

    def read(self, timeout: float = 1.0):
        if self.i >= len(self.buffers):
            if not self.loop:
                raise EOFError
            self.i = 0
        if self.period:
            self.next_t = max(self.next_t + self.period, time.monotonic())
            time.sleep(max(0.0, self.next_t - time.monotonic()))
        t = time.monotonic()
        buf = self.buffers[self.i]
        self.i += 1
        t0 = time.perf_counter()
        gray = cv2.imdecode(buf, self.flag)
        self.last_decode_ms = (time.perf_counter() - t0) * 1e3
        return t, gray

    def close(self):
        pass


def _split_jpegs(data: bytes):
    start = 0
    while (a := data.find(b"\xff\xd8\xff", start)) >= 0 and (b := data.find(b"\xff\xd9", a)) >= 0:
        yield np.frombuffer(data[a:b + 2], np.uint8)
        start = b + 2


def cpu_temp_c() -> float | None:
    try:
        return int(open("/sys/class/thermal/thermal_zone0/temp").read()) / 1000
    except (OSError, ValueError):
        return None


class Stats:
    def __init__(self, every_s: float):
        self.every_s, self.t0, self.rows = every_s, time.monotonic(), []

    def add(self, frame: FrameResult, age_ms: float):
        self.rows.append((frame.decode_ms, frame.detect_ms, age_ms, len(frame.tags)))
        now = time.monotonic()
        if self.every_s and now - self.t0 >= self.every_s:
            r = np.array(self.rows)
            temp = cpu_temp_c()
            log.info("%.1f fps | decode %.1f detect %.1f (p95 %.1f) age %.1f (p95 %.1f) ms | tags/frame %.2f%s",
                     len(r) / (now - self.t0), np.median(r[:, 0]), np.median(r[:, 1]), np.percentile(r[:, 1], 95),
                     np.median(r[:, 2]), np.percentile(r[:, 2], 95), r[:, 3].mean(),
                     "" if temp is None else f" | {temp:.0f} C")
            self.t0, self.rows = now, []


def run(cfg: config_mod.Config, replay: str | None = None, seconds: float | None = None, loop: bool = False) -> int:
    det_cfg = cfg.detector
    detector = X4TagDetector(refine_bow_px=det_cfg.refine_bow_px, seam_px=det_cfg.seam_px, min_tag_px=det_cfg.min_tag_px,
                             threads=det_cfg.threads, family=det_cfg.family, **det_cfg.extra)
    publishers = make_publishers(cfg.output)
    stats = Stats(cfg.output.stats_every_s)
    stop = {"now": False}
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.update(now=True))
    deadline = None if seconds is None else time.monotonic() + seconds
    seq = 0
    try:
        while not stop["now"] and (deadline is None or time.monotonic() < deadline):
            try:
                source = (ReplaySource(replay, cfg.camera.fps, loop, cfg.camera.reduce) if replay else
                          X4Webcam(cfg.camera.device, cfg.camera.width, cfg.camera.height, cfg.camera.fps, cfg.camera.reduce))
            except (RuntimeError, OSError) as e:
                log.warning("camera: %s; retrying in %.0f s", e, cfg.camera.reconnect_s)
                for p in publishers:
                    p.camera_connected(False)
                time.sleep(cfg.camera.reconnect_s)
                continue
            log.info("reading %s", source.device)
            for p in publishers:
                p.camera_connected(True)
            try:
                while not stop["now"] and (deadline is None or time.monotonic() < deadline):
                    try:
                        t_cap, gray = source.read(timeout=max(1.0, cfg.camera.reconnect_s))
                    except EOFError:
                        return 0
                    except ValueError as e:  # one corrupt JPEG: skip it
                        log.warning("camera: %s", e)
                        continue
                    except TimeoutError as e:  # stalled or unplugged: reopen it
                        log.warning("camera: %s; reopening", e)
                        break
                    t0 = time.perf_counter()
                    tags = detector.detect(gray)
                    detect_ms = (time.perf_counter() - t0) * 1e3
                    frame = FrameResult(seq, t_cap, gray.shape[1], gray.shape[0], source.last_decode_ms, detect_ms,
                                        tag_results(tags, cfg.mount, gray.shape[0], cfg.tags.size_m,
                                                    cfg.tags.max_pose_error_px, cfg.tags.ids))
                    for p in publishers:
                        p.publish(frame)
                    stats.add(frame, (time.monotonic() - t_cap) * 1e3)
                    seq += 1
            finally:
                source.close()
                for p in publishers:
                    p.camera_connected(False)
    finally:
        for p in publishers:
            p.close()
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="x4tags", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", help="TOML settings (see config.example.toml)")
    ap.add_argument("--print", action="store_true", help="also print one JSON line per frame")
    ap.add_argument("--replay", help="MJPEG file or image folder instead of the camera")
    ap.add_argument("--loop", action="store_true", help="loop the replay")
    ap.add_argument("--seconds", type=float, help="stop after this long")
    ap.add_argument("--log-level", default="INFO")
    a = ap.parse_args(argv)
    logging.basicConfig(level=a.log_level.upper(), format="%(asctime)s %(levelname)s %(message)s")
    cfg = config_mod.load(a.config)
    if a.print:
        cfg.output.print = True
    return run(cfg, a.replay, a.seconds, a.loop)
