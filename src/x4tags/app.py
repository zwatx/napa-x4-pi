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
import socket
import threading
import time
import tomllib
from typing import ClassVar

import cv2
import numpy as np

from . import __version__
from . import config as config_mod
from .capture import DECODE_FLAGS, NewestFrame, X4Webcam
from .detector import X4TagDetector
from .field import FieldLayout
from .localize import FieldLocalizer
from .publish import Guarded, make_publishers
from .results import FrameResult, tag_results

log = logging.getLogger("x4tags")


class ReplaySource:
    """Frames from a file of concatenated JPEGs (an MJPEG capture) or a folder of images; same read() contract as
    X4Webcam. Paced at ``fps``, it behaves like the camera: a thread delivers and decodes frames on time whether or
    not they're collected (uncollected ones are dropped). With fps 0 it hands over every frame, in order, as fast as
    they're asked for."""

    def __init__(self, path: str, fps: float = 30.0, loop: bool = False, reduce: int = 1):
        self.flag = DECODE_FLAGS[reduce]
        if os.path.isdir(path):
            files = sorted(f for f in glob.glob(os.path.join(path, "*"))
                           if f.lower().endswith((".jpg", ".jpeg", ".png")))
            self.buffers = [np.fromfile(f, np.uint8) for f in files]
        else:
            with open(path, "rb") as f:
                self.buffers = list(_split_jpegs(f.read()))
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
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            return int(f.read()) / 1000
    except (OSError, ValueError):
        return None


class StartupError(Exception):
    """A problem the service can't fix by retrying (bad settings, a missing file): reported in one line."""


class Systemd:
    """Readiness and watchdog pings for systemd (``Type=notify``, ``WatchdogSec``); does nothing outside it."""

    def __init__(self):
        addr = os.environ.get("NOTIFY_SOCKET")
        self.addr = None if not addr else ("\0" + addr[1:] if addr.startswith("@") else addr)
        self._last = -1e9

    def _send(self, msg: str) -> None:
        if self.addr is None:
            return
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as s:
                s.connect(self.addr)
                s.sendall(msg.encode())
        except OSError:
            pass

    def ready(self) -> None:
        self._send("READY=1")

    def alive(self) -> None:
        """Call often: pings at most once a second."""
        now = time.monotonic()
        if now - self._last >= 1.0:
            self._last = now
            self._send("WATCHDOG=1")

    def stopping(self) -> None:
        self._send("STOPPING=1")


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
            clock = getattr(source, "clock", None)
            log.info("%.1f fps | decode %.1f detect %.1f (p95 %.1f) age %.1f (p95 %.1f) ms | tags/frame %.2f | "
                     "camera frames skipped %d, corrupt %d | errors %d%s%s",
                     len(r) / (now - self.t0), np.median(r[:, 0]), np.median(r[:, 1]), np.percentile(r[:, 1], 95),
                     np.median(r[:, 2]), np.percentile(r[:, 2], 95), r[:, 3].mean(), source.dropped, source.corrupt,
                     self.errors,
                     "" if clock is None else f" | frame clock {'on' if clock.locked else 'off (arrival times)'}",
                     "" if temp is None else f" | {temp:.0f} C")
            self.t0, self.rows = now, []


class RateLimitedLog:
    """Log at most every ``every_s`` seconds, with how many were suppressed."""

    def __init__(self, every_s: float = 5.0, level: int = logging.ERROR):
        self.every_s, self.level, self.last, self.suppressed = every_s, level, -1e9, 0

    def __call__(self, msg: str, *args, exc_info=False):
        now = time.monotonic()
        if now - self.last >= self.every_s:
            extra = f" (+{self.suppressed} more since)" if self.suppressed else ""
            log.log(self.level, msg + extra, *args, exc_info=exc_info)
            self.last, self.suppressed = now, 0
        else:
            self.suppressed += 1


class Status:
    """The service's state for the robot (NetworkTables ``status``) and the log: told when it changes, and a
    problem that lasts is repeated once a minute rather than on every retry."""

    def __init__(self, publishers):
        self.publishers, self.text = publishers, None
        self._repeat = RateLimitedLog(60.0, logging.WARNING)

    def __call__(self, text: str) -> None:
        if text == self.text:
            if text != "ok":
                self._repeat("still: %s", text)
            return
        (log.info if text == "ok" else log.warning)("status: %s", text)
        self._repeat.last = time.monotonic()
        self.text = text
        for p in self.publishers:
            p.status(text)


def _build(cfg: config_mod.Config):
    """(detector, localizer or None, publishers) from the settings; StartupError for what retrying can't fix."""
    det_cfg = cfg.detector
    try:
        detector = X4TagDetector(refine_bow_px=det_cfg.refine_bow_px, seam_px=det_cfg.seam_px,
                                 near_scale=det_cfg.near_scale, near_per_frame=det_cfg.near_per_frame,
                                 min_tag_px=det_cfg.min_tag_px, mode=cfg.camera.mode, threads=det_cfg.threads,
                                 family=det_cfg.family, **{"camera_gamma": det_cfg.camera_gamma, **det_cfg.extra})
    except (TypeError, ValueError) as e:
        raise StartupError(f"[detector]: {e}") from e
    localizer = None
    if cfg.field.layout:
        try:
            layout = FieldLayout.load(cfg.field.layout)
        except (OSError, ValueError) as e:
            raise StartupError(f"[field] layout {cfg.field.layout}: {e}") from e
        localizer = FieldLocalizer(layout, cfg.tags.size_m, cfg.mount, cfg.field.max_rms_px, cfg.field.min_tags,
                                   corner_noise_px=cfg.field.corner_noise_px, max_xy_std_m=cfg.field.max_xy_std_m)
        log.info("field layout %s: %d tags", cfg.field.layout, len(layout.tags))
    try:
        publishers = [Guarded(p) for p in make_publishers(cfg.output)]
    except ImportError as e:
        detector.close()
        raise StartupError(f"[output] nt_server needs pyntcore: pip install 'napa-x4-pi[nt]' ({e})") from e
    except ValueError as e:
        detector.close()
        raise StartupError(f"[output]: {e}") from e
    return detector, localizer, publishers


def run(cfg: config_mod.Config, replay: str | None = None, seconds: float | None = None, loop: bool = False) -> int:
    """Run until stopped (SIGINT/SIGTERM), ``seconds`` pass, or a replay ends. StartupError for settings or files
    that won't work; everything else (camera missing or unplugged, bad frames, a failing output) it rides out."""
    cv2.setNumThreads(1)  # OpenCV's own pool would compete with NAPA's threads and the grab thread
    detector, localizer, publishers = _build(cfg)
    systemd = Systemd()
    status = Status(publishers)
    stats = Stats(cfg.output.stats_every_s)
    frame_error = RateLimitedLog()
    latency_s = cfg.camera.latency() / 1e3
    stop = {"now": False}
    deadline = None if seconds is None else time.monotonic() + seconds
    running = lambda: not stop["now"] and (deadline is None or time.monotonic() < deadline)  # noqa: E731

    def wait(s):  # sleep, still pinging the watchdog and stopping promptly
        end = time.monotonic() + s
        while running() and time.monotonic() < end:
            systemd.alive()
            time.sleep(min(0.5, max(0.0, end - time.monotonic())))

    old_handlers = {}
    if threading.current_thread() is threading.main_thread():
        for sig in (signal.SIGINT, signal.SIGTERM):
            old_handlers[sig] = signal.signal(sig, lambda *_: stop.update(now=True))
    seq = 0
    systemd.ready()
    try:
        while running():
            systemd.alive()
            try:
                source = (ReplaySource(replay, cfg.camera.fps, loop, cfg.camera.reduce) if replay else
                          X4Webcam(cfg.camera.device, *cfg.camera.size(), cfg.camera.fps, cfg.camera.reduce))
            except (RuntimeError, OSError, ValueError) as e:
                if replay:  # a file won't appear by waiting
                    raise StartupError(f"replay {replay}: {e}") from e
                status(f"no camera: {e}")
                for p in publishers:
                    p.camera_connected(False)
                wait(cfg.camera.reconnect_s)
                continue
            log.info("reading %s", source.device)
            connected = False
            try:
                while running():
                    systemd.alive()
                    try:
                        t_cap, gray = source.read(timeout=min(5.0, max(1.0, cfg.camera.reconnect_s)))
                    except EOFError:
                        return 0
                    except ValueError as e:  # one corrupt JPEG: skip it
                        log.warning("camera: %s", e)
                        continue
                    except TimeoutError as e:  # stalled or unplugged: reopen it
                        status(f"camera stalled: {e}")
                        break
                    want = config_mod.MODE_SIZES[cfg.camera.mode]
                    if gray.shape[1] * want[1] != gray.shape[0] * want[0]:
                        status(f"camera sends {gray.shape[1]}x{gray.shape[0]}, not the {want[0]}x{want[1]} picture "
                               f"of [camera] mode = {cfg.camera.mode!r}: set the mode to match (\"equirect\" for "
                               f"2880x1440, \"dual1080\" for 1920x1080) or the X4's webcam output")
                        wait(cfg.camera.reconnect_s)
                        break
                    if not connected:
                        connected = True
                        status("ok")
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
                            frame.detect_ms = (time.perf_counter() - t0) * 1e3  # detection plus the field pose
                    except Exception:  # one bad frame must not take the robot's vision down
                        stats.errors += 1
                        frame_error("frame %d failed", seq, exc_info=True)
                        continue
                    finally:
                        seq += 1
                    for p in publishers:  # each guarded: one failing output can't stop the others
                        p.publish(frame)
                    stats.add(frame, (time.monotonic() - t_cap) * 1e3, source)
            finally:
                source.close()
                for p in publishers:
                    p.camera_connected(False)
    finally:
        systemd.stopping()
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
        for p in publishers:
            p.close()
        detector.close()
    return 0


class _JournalFormatter(logging.Formatter):
    """Under systemd: a syslog priority prefix (so ``journalctl -p warning`` works) and no time (journald adds it)."""

    PRIORITY: ClassVar[dict[int, int]] = {logging.DEBUG: 7, logging.INFO: 6, logging.WARNING: 4, logging.ERROR: 3,
                                          logging.CRITICAL: 2}

    def format(self, record):
        return f"<{self.PRIORITY.get(record.levelno, 6)}>{super().format(record)}"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="x4tags", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", help="TOML settings (see config.example.toml)")
    ap.add_argument("--print", action="store_true", help="also print one JSON line per frame")
    ap.add_argument("--replay", help="MJPEG file or image folder instead of the camera")
    ap.add_argument("--loop", action="store_true", help="loop the replay")
    ap.add_argument("--seconds", type=float, help="stop after this long")
    ap.add_argument("--log-level", default="INFO")
    ap.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    a = ap.parse_args(argv)
    handler = logging.StreamHandler()
    if os.environ.get("JOURNAL_STREAM"):
        handler.setFormatter(_JournalFormatter("%(levelname)s %(message)s"))
    else:
        handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    logging.basicConfig(level=a.log_level.upper(), handlers=[handler])
    try:
        cfg = config_mod.load(a.config)
    except (OSError, ValueError, tomllib.TOMLDecodeError) as e:
        log.error("settings %s: %s", a.config, e)
        return 78  # EX_CONFIG: systemd retries with a growing delay, so a fixed file is picked up
    if a.print:
        cfg.output.print = True
    out = cfg.output
    log.info("x4tags %s (napa-vision %s, OpenCV %s); settings %s; camera %s, %s mode; outputs: %s", __version__,
             _version("napa-vision"), cv2.__version__, a.config or "defaults", cfg.camera.device or "auto",
             cfg.camera.mode, ", ".join(x for x in (out.nt_server and f"NetworkTables {out.nt_server}",
                                                    out.udp and f"UDP {out.udp}", out.print and "stdout") if x)
             or "none")
    if not (out.nt_server or out.udp or out.print):
        log.warning("no outputs set ([output] nt_server, udp or print): results go nowhere")
    if out.print and os.environ.get("JOURNAL_STREAM"):
        log.warning("[output] print under systemd writes a JSON line per frame into the journal")
    try:
        return run(cfg, a.replay, a.seconds, a.loop)
    except StartupError as e:
        log.error("%s", e)
        return 78


def _version(dist: str) -> str:
    from importlib.metadata import PackageNotFoundError, version

    try:
        return version(dist)
    except PackageNotFoundError:
        return "?"
