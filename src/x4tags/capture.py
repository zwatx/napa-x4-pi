"""The X4 in webcam (UVC) mode: MJPEG frames, decoded straight to grey.

OpenCV's usual ``VideoCapture.read()`` decodes every MJPEG frame to colour, and a detector then converts it back to
grey. Asking V4L2 for the undecoded JPEG and decoding it as greyscale (libjpeg skips the colour planes) is about half
the cost: on a Pi 5, 25 -> 11 ms for a 2880x1440 frame from the X4, 34 -> 18 ms for busier quality-90 test frames. A background thread keeps only the newest frame, so a slow
consumer always gets the latest picture instead of a growing backlog.
"""

from __future__ import annotations

import glob
import logging
import os
import threading
import time

import cv2
import numpy as np

X4_USB_VENDOR = "2e1a"
log = logging.getLogger(__name__)


def find_x4_device() -> str | None:
    """/dev/videoN of the X4's capture node, found by USB vendor ID in sysfs (``lsusb`` isn't needed)."""
    for node in sorted(glob.glob("/sys/class/video4linux/video*"), key=lambda p: int(p.rsplit("video", 1)[1])):
        try:
            if open(os.path.join(node, "index")).read().strip() != "0":
                continue  # metadata nodes share the device; index 0 is the picture
        except OSError:
            continue
        path = os.path.realpath(os.path.join(node, "device"))
        while path != "/":
            vid = os.path.join(path, "idVendor")
            if os.path.exists(vid):
                if open(vid).read().strip() == X4_USB_VENDOR:
                    return "/dev/" + os.path.basename(node)
                break
            path = os.path.dirname(path)
    return None


class X4Webcam:
    """Newest-frame grey capture from the X4 webcam stream.

    ``reduce``: 1 for full 2880x1440, 2 for libjpeg's built-in half-size decode (1440x720, cheaper still).
    """

    def __init__(self, device: str | None = None, width: int = 2880, height: int = 1440, fps: int = 30,
                 reduce: int = 1):
        self.device = device or find_x4_device()
        if self.device is None:
            raise RuntimeError("no Insta360 X4 found; is it in webcam mode and plugged straight into this computer?")
        self.flag = {1: cv2.IMREAD_GRAYSCALE, 2: cv2.IMREAD_REDUCED_GRAYSCALE_2, 4: cv2.IMREAD_REDUCED_GRAYSCALE_4}[reduce]
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
        self._cond = threading.Condition()
        self._latest = None  # (sequence, capture time, buffer)
        self._seq = 0
        self._taken = 0
        self._running = True
        self.dropped = 0
        self.last_decode_ms = 0.0
        self._thread = threading.Thread(target=self._grab, name="x4-grab", daemon=True)
        self._thread.start()

    def _grab(self):
        try:
            while self._running:
                ok, buf = self._cap.read()
                t = time.monotonic()
                if not ok:
                    time.sleep(0.005)
                    continue
                with self._cond:
                    if self._latest is not None and self._latest[0] > self._taken:
                        self.dropped += 1  # never collected: the consumer is slower than the camera
                    self._seq += 1
                    self._latest = (self._seq, t, buf)
                    self._cond.notify_all()
        finally:
            self._cap.release()  # only here, so it never happens in the middle of a read

    def read(self, timeout: float = 1.0):
        """(capture time from time.monotonic(), grey frame) for the newest frame not yet returned."""
        with self._cond:
            if not self._cond.wait_for(lambda: self._latest is not None and self._latest[0] > self._taken, timeout):
                raise TimeoutError(f"no frame from {self.device} in {timeout} s")
            seq, t, buf = self._latest
            self._taken = seq
        t0 = time.perf_counter()
        gray = self.decode(buf)
        self.last_decode_ms = (time.perf_counter() - t0) * 1e3
        return t, gray

    def decode(self, buf: np.ndarray) -> np.ndarray:
        if buf.ndim == 3:  # this OpenCV build decoded it anyway
            return cv2.cvtColor(buf, cv2.COLOR_BGR2GRAY)
        gray = cv2.imdecode(buf.reshape(-1), self.flag)
        if gray is None:
            raise ValueError("corrupt JPEG from the camera")
        return gray

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
