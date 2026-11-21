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
import threading
import time

import cv2
import numpy as np

from . import config as config_mod
from .capture import NewestFrame, X4Webcam
from .detector import X4TagDetector
from .field import FieldLayout
from .localize import FieldLocalizer
from .publish import make_publishers
from .results import FrameResult, tag_results

log = logging.getLogger("x4tags")


class ReplaySource:
    """Frames from a file of concatenated JPEGs (an MJPEG capture) or a folder of images; same read() contract as
    X4Webcam. Paced at ``fps``, it behaves like the camera: a thread delivers and decodes frames on time whether or
    not they're collected (uncollected ones are dropped). With fps 0 it hands over every frame, in order, as fast as
    they're asked for."""

    def __init__(self, path: str, fps: float = 30.0, loop: bool = False, reduce: int = 1):
        self.flag = {1: cv2.IMREAD_GRAYSCALE, 2: cv2.IMREAD_REDUCED_GRAYSCALE_2, 4: cv2.IMREAD_REDUCED_GRAYSCALE_4}[reduce]
        if os.path.isdir(path):
            files = sorted(f for f in glob.glob(os.path.join(path, "*")) if f.lower().endswith((".jpg", ".jpeg", ".png")))
            self.buffers = [np.fromfile(f, np.uint8) for f in files]
        else:
            self.buffers = list(_split_jpegs(open(path, "rb").read()))
        if not self.buffers:
            raise ValueError(f"no frames in {path}")
        self.loop, self.i = loop, 0
        self.device, self.size, self.last_decode_ms, self.corrupt = path, None, 0.0, 0
        self._frames = None
        if fps > 0:
            self._frames, self._running = NewestFrame(), True
            self._thread = threading.Thread(target=self._play, args=(1.0 / fps,), name="replay", daemon=True)
            self._thread.start()

    @property
    def dropped(self) -> int:
        return self._frames.dropped if self._frames else 0

    def _next(self):
        """(arrival time, grey or None if it doesn't decode, decode ms) of the next frame; EOFError at the end."""
        if self.i >= len(self.buffers):
            if not self.loop:
                raise EOFError
            self.i = 0
        buf = self.buffers[self.i]
        self.i += 1
        t = time.monotonic()
        t0 = time.perf_counter()
        gray = cv2.imdecode(buf, self.flag)
        return t, gray, (time.perf_counter() - t0) * 1e3

    def _play(self, period):
        next_t = time.monotonic()
        try:
            while self._running:
                next_t = max(next_t + period, time.monotonic())
                time.sleep(max(0.0, next_t - time.monotonic()))
                try:
                    t, gray, ms = self._next()
                except EOFError:
                    raise
                except Exception:  # cv2.error and the like: count it, keep playing
                    log.exception("replay frame %d failed", self.i - 1)
                    gray = None
                if gray is None:
                    self.corrupt += 1
                    continue
                self._frames.put(t, gray, ms)
        except EOFError:
            self._frames.finish()

    def read(self, timeout: float = 1.0):
        if self._frames is not None:
            t, gray, self.last_decode_ms = self._frames.get(timeout, self.device)
            return t, gray
        t, gray, self.last_decode_ms = self._next()
        if gray is None:
            raise ValueError(f"frame {self.i - 1} of {self.device} doesn't decode")
        return t, gray

    def close(self):
        if self._frames is not None:
            self._running = False
            self._thread.join(timeout=2)


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
        self.every_s, self.t0, self.rows, self.errors = every_s, time.monotonic(), [], 0

    def add(self, frame: FrameResult, age_ms: float, source) -> None:
        if not self.every_s:
            return
        self.rows.append((frame.decode_ms, frame.detect_ms, age_ms, len(frame.tags)))
        now = time.monotonic()
        if now - self.t0 >= self.every_s:
            r = np.array(self.rows)
            temp = cpu_temp_c()
            log.info("%.1f fps | decode %.1f detect %.1f (p95 %.1f) age %.1f (p95 %.1f) ms | tags/frame %.2f | "
                     "camera frames skipped %d, corrupt %d | errors %d%s",
                     len(r) / (now - self.t0), np.median(r[:, 0]), np.median(r[:, 1]), np.percentile(r[:, 1], 95),
                     np.median(r[:, 2]), np.percentile(r[:, 2], 95), r[:, 3].mean(), source.dropped, source.corrupt, self.errors,
                     "" if temp is None else f" | {temp:.0f} C")
            self.t0, self.rows = now, []


class RateLimitedLog:
    """Log an error at most every ``every_s`` seconds, with how many were suppressed."""

    def __init__(self, every_s: float = 5.0):
        self.every_s, self.last, self.suppressed = every_s, -1e9, 0

    def __call__(self, msg: str, *args, exc_info=False):
        now = time.monotonic()
        if now - self.last >= self.every_s:
            extra = f" (+{self.suppressed} more since)" if self.suppressed else ""
            log.error(msg + extra, *args, exc_info=exc_info)
            self.last, self.suppressed = now, 0
        else:
            self.suppressed += 1


def run(cfg: config_mod.Config, replay: str | None = None, seconds: float | None = None, loop: bool = False) -> int:
    cv2.setNumThreads(1)  # OpenCV's own pool would compete with NAPA's threads and the grab thread
    det_cfg = cfg.detector
    detector = X4TagDetector(refine_bow_px=det_cfg.refine_bow_px, seam_px=det_cfg.seam_px, near_scale=det_cfg.near_scale,
                             near_per_frame=det_cfg.near_per_frame, min_tag_px=det_cfg.min_tag_px,
                             mode=cfg.camera.mode, threads=det_cfg.threads, family=det_cfg.family, **det_cfg.extra)
    localizer = None
    if cfg.field.layout:
        layout = FieldLayout.load(cfg.field.layout)
        localizer = FieldLocalizer(layout, cfg.tags.size_m, cfg.mount, cfg.field.max_rms_px, cfg.field.min_tags,
                                   corner_noise_px=cfg.field.corner_noise_px, max_xy_std_m=cfg.field.max_xy_std_m)
        log.info("field layout %s: %d tags", cfg.field.layout, len(layout.tags))
    publishers = make_publishers(cfg.output)
    stats = Stats(cfg.output.stats_every_s)
    frame_error = RateLimitedLog()
    latency_s = cfg.camera.latency() / 1e3
    stop = {"now": False}
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.update(now=True))
    deadline = None if seconds is None else time.monotonic() + seconds
    seq = 0
    try:
        while not stop["now"] and (deadline is None or time.monotonic() < deadline):
            try:
                source = (ReplaySource(replay, cfg.camera.fps, loop, cfg.camera.reduce) if replay else
                          X4Webcam(cfg.camera.device, *cfg.camera.size(), cfg.camera.fps, cfg.camera.reduce))
            except (RuntimeError, OSError) as e:
                log.warning("camera: %s; retrying in %.0f s", e, cfg.camera.reconnect_s)
                for p in publishers:
                    p.camera_connected(False)
                time.sleep(cfg.camera.reconnect_s)
                continue
            log.info("reading %s", source.device)
            connected = False
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
                    want = config_mod.MODE_SIZES[cfg.camera.mode]
                    if gray.shape[1] * want[1] != gray.shape[0] * want[0]:
                        log.error("camera sends %dx%d, not the %dx%d picture of [camera] mode = %r: set the mode to "
                                  "match (\"equirect\" for 2880x1440, \"dual1080\" for 1920x1080) or the X4's webcam "
                                  "output; retrying in %.0f s", gray.shape[1], gray.shape[0], *want, cfg.camera.mode,
                                  cfg.camera.reconnect_s)
                        time.sleep(cfg.camera.reconnect_s)
                        break
                    if not connected:
                        connected = True
                        for p in publishers:
                            p.camera_connected(True)
                    t_cap -= latency_s  # when the camera took it, as well as we know (camera.latency_ms)
                    try:
                        px_per_rad = detector.px_per_rad(gray.shape[1], gray.shape[0])
                        t0 = time.perf_counter()
                        tags = detector.detect(gray)
                        detect_ms = (time.perf_counter() - t0) * 1e3
                        frame = FrameResult(seq, t_cap, gray.shape[1], gray.shape[0], source.last_decode_ms, detect_ms,
                                            tag_results(tags, cfg.mount, gray.shape[0], cfg.tags.size_m,
                                                        cfg.tags.max_pose_error_px, cfg.tags.ids, px_per_rad))
                        if localizer is not None:
                            try:  # a failed field pose mustn't cost the frame its tags
                                frame.field_pose = localizer.solve(tags, gray.shape[0], px_per_rad)
                            except Exception:
                                stats.errors += 1
                                frame_error("field pose for frame %d failed", seq, exc_info=True)
                            frame.detect_ms = (time.perf_counter() - t0) * 1e3
                        for p in publishers:
                            p.publish(frame)
                    except Exception:  # one bad frame must not take the robot's vision down
                        stats.errors += 1
                        frame_error("frame %d failed", seq, exc_info=True)
                        continue
                    finally:
                        seq += 1
                    stats.add(frame, (time.monotonic() - t_cap) * 1e3, source)
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
