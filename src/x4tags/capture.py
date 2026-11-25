"""The X4 in webcam (UVC) mode: MJPEG frames, decoded straight to grey.

OpenCV's usual ``VideoCapture.read()`` decodes every MJPEG frame to colour, and a detector then converts it back to
grey. Asking V4L2 for the undecoded JPEG and decoding it as greyscale (libjpeg skips the colour planes) is about half
the cost: on a Pi 5, 25 -> 11 ms for a 2880x1440 frame from the X4, 34 -> 18 ms for busier quality-90 test frames.
A background thread reads and decodes each frame as it arrives, on another core than the detector, and keeps only
the newest, so a slow consumer always gets the latest picture instead of a growing backlog.
"""

from __future__ import annotations

import glob
import logging
import os
import threading
import time
from collections import deque

import cv2
import numpy as np

X4_USB_VENDOR = "2e1a"
# libjpeg decodes straight to grey, at full, half or quarter size (``reduce``)
DECODE_FLAGS = {1: cv2.IMREAD_GRAYSCALE, 2: cv2.IMREAD_REDUCED_GRAYSCALE_2, 4: cv2.IMREAD_REDUCED_GRAYSCALE_4}
log = logging.getLogger(__name__)


# How early (in frame periods) a frame may seem to arrive and still count as on time: room for the clock's own
# estimate to wander between refits.
EARLY = 0.06


class FrameClock:
    """Steady frame times from late, uneven arrival times.

    The X4 captures on a steady clock (29.97 fps on one camera) but delivers its frames late by an uneven amount,
    in a repeating pattern of three: on a Pi 5, about 16, 9 and 0 ms late at 2880x1440 and 26, 3 and 0 ms at
    1920x1080, plus noise. A frame can't arrive early, so the lower edge of arrival time against frame number is the
    camera's clock. This finds that clock (period and phase), numbers each frame by its slot on it, refits the line
    by least squares over the last ``window_s`` seconds, and returns each frame's slot time plus the window's mean
    delay: on average the same as the arrival times, so a latency measured from arrivals still applies, but without
    the frame-to-frame jitter.

    Until it has locked on (``lock_frames``, ~3 s) it returns arrival times unchanged. A frame late enough to land
    in the previous frame's slot (over a period late) takes the next slot, since the camera seldom drops frames,
    unless that slot starts after the frame arrived (the frame before it was the late one); the times returned always
    increase. When frames that had to be moved, or came later than any frame did while locking on, plus half the
    slots nothing arrived for, add up to more than ``max_misfit`` of the last 90 frames, the camera no longer runs on
    this clock (a new frame rate, say) and it starts over. It doesn't lock on at all when frames arrive 80% of a
    period late or more (the X4's 1920x1080 mode on a Pi 5): a frame's slot would be a guess.
    """

    def __init__(self, fps: float = 30.0, window_s: float = 20.0, lock_frames: int = 90, refit_every: int = 10,
                 max_misfit: float = 0.15):
        self.nominal = 1.0 / fps
        self.window_s, self.lock_frames, self.refit_every = window_s, lock_frames, refit_every
        self.max_misfit = max_misfit
        self._prev = -np.inf  # last time returned
        self._told_unused = False  # said once why it doesn't lock on (the 1080 mode tries again every 10 s)
        self._retry = refit_every  # frames between tries to lock on; doubles after each failure, up to 10 s
        self.reset()

    def reset(self) -> None:
        self._t: deque[float] = deque()
        self._n: deque[int] = deque()
        self._events: deque[float] = deque(maxlen=90)  # 1 for a frame that didn't fit, 0.5 for a slot left empty
        self.period: float | None = None  # seconds per frame, once locked
        self.delay = 0.0  # mean delay above the clock's edge, seconds
        self._a, self._last_n, self._late_max, self._since_fit = 0.0, 0, 0.0, 0

    @property
    def locked(self) -> bool:
        return self.period is not None

    def __call__(self, t: float) -> float:
        out = self._stamp(t)
        if out <= self._prev:  # e.g. back to arrival times after losing the clock
            out = self._prev + 1e-4
        self._prev = out
        return out

    def _stamp(self, t: float) -> float:
        if self._t and t - self._t[-1] > 1.0:  # a stall: the camera may have restarted its clock
            self.reset()
        if self.period is None:
            self._t.append(t)
            if len(self._t) > self.lock_frames:
                self._t.popleft()
            self._since_fit += 1
            if len(self._t) < self.lock_frames or self._since_fit < self._retry:
                return t
            if not self._lock():
                self._retry = min(2 * self._retry, 300)
                return t
            self._retry = self.refit_every
            return self._a + self._last_n * self.period + self.delay
        x = (t - self._a) / self.period
        n = int(np.floor(x + EARLY))
        fits = n > self._last_n and x - n <= self._late_max
        if fits:  # a slot nothing arrived for counts half: frames do get dropped, but not often
            self._events.extend([0.5] * min(n - self._last_n - 1, 10))
        self._events.append(0.0 if fits else 1.0)
        if sum(self._events) > self.max_misfit * self._events.maxlen:
            log.info("frame clock lost: arrivals no longer fit a steady %.3f fps", 1 / self.period)
            self.reset()
            self._t.append(t)
            return t
        if not fits:
            # Late enough to land in the last frame's slot (or later than any frame while locking on): frames are
            # seldom dropped, so it takes the next slot, unless that slot only starts after the frame arrived; then
            # the frame before was a very late one that took this frame's slot, and this one keeps its arrival time.
            # Either way it stays out of the fit.
            m = max(n, self._last_n + 1)
            if x < m - EARLY:
                self._last_n = n
                return t
            self._last_n = m
            return self._a + m * self.period + self.delay
        self._last_n = n
        self._t.append(t)
        self._n.append(n)
        while self._t[0] < t - self.window_s:
            self._t.popleft()
            self._n.popleft()
        self._since_fit += 1
        if self._since_fit >= self.refit_every and not self._fit():
            self.reset()
            return t
        return self._a + n * self.period + self.delay

    def _lock(self) -> bool:
        """Find the period (within 1.5% of nominal) that bunches the arrivals' phases tightest, then the edge: the
        frame taken as on time that puts every frame in a slot of its own and leaves the fewest slots empty."""
        self._since_fit = 0
        t = np.asarray(self._t)
        rel = t - t[0]
        periods = self.nominal * np.linspace(0.985, 1.015, 601)
        ph = np.mod(rel[None, :], periods[:, None]) / periods[:, None]
        s = np.sort(ph, axis=1)
        gaps = np.diff(np.concatenate([s, s[:, :1] + 1.0], axis=1), axis=1)  # empty arcs between phases, circular
        best = int(gaps.max(1).argmax())
        if gaps[best].max() < 0.04:  # phases all over the period: no steady clock to find
            return False
        period = float(periods[best])
        x = rel / period
        edges = np.unique(np.round(ph[best], 4))  # each phase seen, as the on-time one
        n = np.floor(x[None, :] - edges[:, None] + EARLY)
        steps = np.diff(n, axis=1)
        shared = (steps <= 0).sum(1)
        empty = np.where(steps > 1, steps - 1, 0).sum(1)
        k = int(np.argmin(shared * 1000 + empty))
        if shared[k] or empty[k] > self.max_misfit * len(t):
            return False
        self.period, self._a = period, float(t[0] + edges[k] * period)
        self._n = deque(int(v) for v in n[k])
        self._last_n = self._n[-1]
        if not self._fit():
            self.reset()
            return False
        late = float(((t - self._a) / self.period - np.asarray(self._n)).max())
        if late > 0.8:  # some frames nearly a period late: which slot a frame belongs to would be a guess
            (log.debug if self._told_unused else log.info)(
                "frame clock not used: frames arrive up to %.0f ms late, too close to a frame period",
                late * self.period * 1e3)
            self._told_unused = True
            self.reset()
            return False
        self._late_max = late + 0.1
        log.info("frame clock locked: %.3f fps, frames %.1f ms late on average", 1 / self.period, self.delay * 1e3)
        return True

    def _fit(self) -> bool:
        """Least-squares line through the window's arrivals against slot number: the delays have a steady mean, so
        this is the camera's clock plus the mean delay (what ``__call__`` returns). Its lowest point is the clock's
        edge, which numbers the slots."""
        self._since_fit = 0
        t, n = np.asarray(self._t), np.asarray(self._n, float)
        n0 = n[0]  # fit about the window's first frame, for precision
        slope, icept = np.polyfit(n - n0, t, 1)
        if abs(slope / self.nominal - 1) > 0.02:
            return False
        low = float((t - (icept + (n - n0) * slope)).min())
        self._a, self.period = float(icept + low - n0 * slope), float(slope)
        self.delay = -low
        return True


def _read(path: str) -> str:
    with open(path) as f:
        return f.read().strip()


def find_x4_device() -> str | None:
    """/dev/videoN of the X4's capture node, found by USB vendor ID in sysfs (``lsusb`` isn't needed)."""
    for node in sorted(glob.glob("/sys/class/video4linux/video*"), key=lambda p: int(p.rsplit("video", 1)[1])):
        try:
            if _read(os.path.join(node, "index")) != "0":
                continue  # metadata nodes share the device; index 0 is the picture
        except OSError:
            continue
        path = os.path.realpath(os.path.join(node, "device"))
        while path != "/":
            vid = os.path.join(path, "idVendor")
            if os.path.exists(vid):
                if _read(vid) == X4_USB_VENDOR:
                    return "/dev/" + os.path.basename(node)
                break
            path = os.path.dirname(path)
    return None


def x4_on_usb() -> bool:
    """Whether an Insta360 camera is on USB at all (with or without a video node: in USB-drive mode it has none)."""
    for vid in glob.glob("/sys/bus/usb/devices/*/idVendor"):
        try:
            if _read(vid) == X4_USB_VENDOR:
                return True
        except OSError:
            continue
    return False


class NewestFrame:
    """Hands the newest decoded frame from a producer thread to the consumer; older uncollected frames are dropped
    (and counted), so the consumer never works through a backlog."""

    def __init__(self):
        self._cond = threading.Condition()
        self._latest = None  # (sequence, capture time, grey, decode ms)
        self._seq = self._taken = 0
        self.dropped = 0
        self.finished = False

    def put(self, t: float, gray: np.ndarray, decode_ms: float) -> None:
        with self._cond:
            if self._latest is not None and self._latest[0] > self._taken:
                self.dropped += 1
            self._seq += 1
            self._latest = (self._seq, t, gray, decode_ms)
            self._cond.notify_all()

    def finish(self) -> None:
        with self._cond:
            self.finished = True
            self._cond.notify_all()

    def get(self, timeout: float, what: str):
        with self._cond:
            ready = lambda: (self._latest is not None and self._latest[0] > self._taken) or self.finished  # noqa: E731
            if not self._cond.wait_for(ready, timeout):
                raise TimeoutError(f"no frame from {what} in {timeout} s")
            if self._latest is None or self._latest[0] <= self._taken:
                raise EOFError
            seq, t, gray, decode_ms = self._latest
            self._taken = seq
            return t, gray, decode_ms


class X4Webcam:
    """Newest-frame grey capture from the X4 webcam stream.

    A background thread reads each JPEG and decodes it as it arrives, on its own core, while the caller works on
    the previous frame. ``reduce``: 1 for full size, 2 or 4 for libjpeg's built-in half- or quarter-size decode
    (1440x720 or 720x360 in the 360-degree mode, 960x540 or 480x270 in the 1080 mode).
    Frame times are arrival times (V4L2's own, when it gives them), steadied by a ``FrameClock``.
    """

    def __init__(self, device: str | None = None, width: int = 2880, height: int = 1440, fps: int = 30,
                 reduce: int = 1):
        self.device = device or find_x4_device()
        if self.device is None:
            if x4_on_usb():
                raise RuntimeError("the X4 is on USB but isn't a webcam: choose Webcam mode on the camera's USB menu "
                                   "(it may have come up as a USB drive)")
            raise RuntimeError("no Insta360 X4 on USB: check its cable and power, and plug it straight into this "
                               "computer (not through a dock)")
        self.flag = DECODE_FLAGS[reduce]
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if hasattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC"):  # don't sit in a stalled read for OpenCV's default ~10 s
            cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 1000)
        self.raw = bool(cap.set(cv2.CAP_PROP_CONVERT_RGB, 0))  # hand back the JPEG itself
        if not cap.isOpened():
            cap.release()
            raise RuntimeError(f"could not open {self.device}")
        self.size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        if not self.raw:
            log.warning("this OpenCV build won't hand back undecoded JPEGs; decoding colour then converting (slower)")
        self._cap = cap  # owned by the grab thread from here on: VideoCapture isn't thread-safe
        self._frames = NewestFrame()
        self._running = True
        self.corrupt = 0
        self.last_decode_ms = 0.0
        self.clock = FrameClock(fps if fps > 0 else 30)
        self._thread = threading.Thread(target=self._grab, name="x4-grab", daemon=True)
        self._thread.start()

    @property
    def dropped(self) -> int:
        return self._frames.dropped

    def _grab(self):
        failed = 0  # reads in a row that gave nothing: back off, up to 0.2 s, rather than spin on an unplugged camera
        try:
            while self._running:
                try:
                    ok, buf = self._cap.read()
                except Exception:  # cv2.error from the driver: the app sees no frames and reopens the camera
                    log.exception("reading %s failed", self.device)
                    return
                t = time.monotonic()
                if not ok:
                    failed += 1
                    time.sleep(min(0.005 * 2 ** min(failed, 6), 0.2))
                    continue
                failed = 0
                # V4L2's buffer time, on the same clock: when the frame's USB transfer began (3-6 ms before read()
                # returns on a Pi 5), and not held up by this thread being busy decoding the previous frame.
                kernel = self._cap.get(cv2.CAP_PROP_POS_MSEC) / 1e3
                if t - 0.5 < kernel <= t:
                    t = kernel
                t0 = time.perf_counter()
                try:
                    gray = self.decode(buf)
                except Exception:  # cv2.error on a mangled frame must not end the thread
                    gray = None
                if gray is None:
                    self.corrupt += 1
                    if self.corrupt in (1, 10, 100) or self.corrupt % 1000 == 0:
                        log.warning("%d corrupt frames from %s so far", self.corrupt, self.device)
                    continue
                try:
                    t_frame = self.clock(t)
                except Exception:  # a bug in the clock mustn't stop the frames: arrival times instead
                    log.exception("frame clock failed; using arrival times")
                    self.clock.reset()
                    t_frame = t
                self._frames.put(t_frame, gray, (time.perf_counter() - t0) * 1e3)
        finally:
            self._cap.release()  # only here, so it never happens in the middle of a read

    def read(self, timeout: float = 1.0):
        """(arrival time on time.monotonic()'s clock, steadied; grey frame) for the newest frame not yet returned."""
        t, gray, self.last_decode_ms = self._frames.get(timeout, self.device)
        return t, gray

    def decode(self, buf: np.ndarray) -> np.ndarray | None:
        """Grey frame from what VideoCapture returned, or None for a corrupt JPEG."""
        if buf.ndim == 3:  # this OpenCV build decoded it anyway
            return cv2.cvtColor(buf, cv2.COLOR_BGR2GRAY)
        return cv2.imdecode(buf.reshape(-1), self.flag)

    def close(self):
        """Stop the grab thread; it releases the camera itself as it exits. A read stuck in the driver keeps the
        device open until it returns, so reopening may fail for a moment (the app retries)."""
        self._running = False
        self._thread.join(timeout=3)
        if self._thread.is_alive():
            log.warning("camera read still blocked; the device is released when it returns")

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
