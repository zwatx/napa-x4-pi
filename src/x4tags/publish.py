"""Where results go: stdout (JSON lines), UDP (one JSON datagram per frame), NetworkTables 4 (needs pyntcore)."""

from __future__ import annotations

import json
import logging
import socket
import sys
import threading
import time

import numpy as np

from . import __version__
from .results import FrameResult

log = logging.getLogger(__name__)


class Publisher:
    def publish(self, frame: FrameResult) -> None:
        raise NotImplementedError

    def camera_connected(self, connected: bool) -> None:
        pass

    def status(self, text: str) -> None:
        """The service's state in a few words ("ok", "no camera: ...", ...), whenever it changes."""

    def close(self) -> None:
        pass


class Guarded(Publisher):
    """One publisher whose failures are logged (at most every 10 s) and kept away from the others: a broken
    stdout pipe mustn't stop NetworkTables."""

    def __init__(self, inner: Publisher):
        self.inner, self.name, self._last, self._missed = inner, type(inner).__name__, -1e9, 0

    def _call(self, method, *args):
        try:
            getattr(self.inner, method)(*args)
        except Exception:
            now = time.monotonic()
            if now - self._last >= 10.0:
                log.warning("%s.%s failed%s", self.name, method,
                            f" (+{self._missed} more since)" if self._missed else "", exc_info=True)
                self._last, self._missed = now, 0
            else:
                self._missed += 1

    def publish(self, frame):
        self._call("publish", frame)

    def camera_connected(self, connected):
        self._call("camera_connected", connected)

    def status(self, text):
        self._call("status", text)

    def close(self):
        self._call("close")


class StdoutPublisher(Publisher):
    def publish(self, frame):
        sys.stdout.write(json.dumps(frame.to_dict(time.monotonic())) + "\n")
        sys.stdout.flush()


class UdpPublisher(Publisher):
    """One JSON datagram per frame to "host:port" (IPv4). The name is looked up on a thread of its own, now and every
    ``resolve_every_s``, so a robot or laptop that comes up late, or changes address, is found, and a slow lookup
    (a DNS server that doesn't answer can take 10 s or more) never holds up a frame."""

    def __init__(self, target: str, resolve_every_s: float = 10.0):
        host, sep, port = target.rpartition(":")
        if not sep or not host or not port.isdigit():
            raise ValueError(f"udp = {target!r}: expected \"host:port\"")
        self.host, self.port, self.resolve_every_s = host, int(port), resolve_every_s
        self.addr, self._failing = None, False
        self._stop = threading.Event()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._thread = threading.Thread(target=self._resolve_forever, name="udp-resolve", daemon=True)
        self._thread.start()

    def _resolve_forever(self):
        while True:
            try:
                self.addr = socket.getaddrinfo(self.host, self.port, socket.AF_INET, socket.SOCK_DGRAM)[0][4]
                if self._failing:
                    log.info("udp: %s:%d is %s again", self.host, self.port, self.addr[0])
                self._failing = False
            except OSError as e:
                if not self._failing:
                    log.warning("udp: can't find %s (%s); trying again every %.0f s", self.host, e,
                                self.resolve_every_s)
                self._failing = True
            if self._stop.wait(self.resolve_every_s):
                return

    def publish(self, frame):
        if self.addr is None:
            return
        data = json.dumps(frame.to_dict(time.monotonic())).encode()
        try:
            self.sock.sendto(data, self.addr)
        except OSError as e:  # no route yet (robot not up): keep running
            log.debug("udp send failed: %s", e)

    def close(self):
        self._stop.set()
        self.sock.close()


TAG_FIELDS = ("id", "yaw_deg", "pitch_deg", "distance_m", "x_m", "y_m", "z_m")
FIELD_POSE_FIELDS = ("robot_x_m", "robot_y_m", "robot_z_m", "roll_deg", "pitch_deg", "yaw_deg",
                     "camera_x_m", "camera_y_m", "camera_z_m", "rms_px", "tag_count", "xy_std_m", "yaw_std_deg",
                     *(f"robot_cov_{a}_{b}" for a in ("x", "y", "yaw") for b in ("x", "y", "yaw")),
                     *(f"camera_cov_{a}_{b}" for a in ("x", "y") for b in ("x", "y")),
                     "gyro_std_forward_m", "gyro_std_left_m")


class NetworkTablesPublisher(Publisher):
    """Publishes under /<table>/ (default /x4tags/):

    - ``tags`` (double[]): every tag of a frame packed in one array, ``len(TAG_FIELDS)`` = 7 values per tag:
      id, yaw_deg, pitch_deg, distance_m, x_m, y_m, z_m (robot frame; distance and position NaN without a pose).
      One array, so a reader never mixes one frame's IDs with another frame's bearings.
    - ``field_pose`` (double[]): with a field layout, the pose from all tags together, 28 values
      (``FIELD_POSE_FIELDS``):
      0-2 robot x, y, z (m); 3-5 roll, pitch, yaw (deg); 6-8 camera x, y, z (m); 9 rms_px; 10 tag count;
      11 xy_std_m, 12 yaw_std_deg (expected position and heading error); 13-21 the robot (x, y, yaw) covariance,
      3x3 row by row, field axes, m and rad; 22-25 the camera (x, y) covariance, 2x2 row by row, m; 26-27 the
      camera position's standard deviations along the robot's forward and left axes (m), for WPILib's
      ``addVisionMeasurement`` with the gyro's heading. Empty when there's no solution. The robot values use the
      picture's heading, which lags the body by a few degrees after quick turns; the camera position holds either
      way (use it with the gyro's heading).
    - ``frame`` (int), ``age_ms``, ``decode_ms``, ``detect_ms`` (double), ``camera_connected`` (boolean).

    Every value of a frame carries the frame's capture time (``camera.latency_ms`` before it arrived) as its NT
    timestamp, for latency compensation with ``getAtomic().serverTime``. While the camera is away, ``tags`` is empty.
    """

    def __init__(self, server: str, table: str = "x4tags", client_name: str = "x4tags"):
        """``server``: a team number ("1234"), or a host ("10.12.34.2", "localhost"), optionally ":port"."""
        import ntcore  # optional dependency: pip install "napa-x4-pi[nt]"

        self._nt, self._server, self._table, self._client_name = ntcore, str(server), table, client_name
        self._now = ntcore._now  # NT's local clock, microseconds (the time base set(value, time) expects)
        self._connected, self._status = False, "starting"
        self._start()

    def _start(self):
        nt = self._nt
        self.inst = nt.NetworkTableInstance.create()
        self.inst.addConnectionListener(False, self._on_connection)
        self.inst.startClient4(self._client_name)
        host, _, port = self._server.partition(":")
        port = int(port) if port else 0  # 0: NT4's default port
        if host.isdigit():
            self.inst.setServerTeam(int(host), port)
        else:
            self.inst.setServer(host, port)
        t = self.inst.getTable(self._table)
        self.tags = t.getDoubleArrayTopic("tags").publish()
        self.field_pose = t.getDoubleArrayTopic("field_pose").publish()
        self.frame = t.getIntegerTopic("frame").publish()
        self.timings = {k: t.getDoubleTopic(k).publish() for k in ("age_ms", "decode_ms", "detect_ms")}
        self.connected = t.getBooleanTopic("camera_connected").publish()
        self.status_pub = t.getStringTopic("status").publish()
        self.version_pub = t.getStringTopic("version").publish()  # kept: a dropped publisher unpublishes
        self.version_pub.set(__version__)
        self.connected.set(self._connected)
        self.status_pub.set(self._status)
        # NT's clock is the wall clock: if that steps back (NTP at boot, say), NT drops every value stamped before its
        # last one until the clock catches up. Watch how it moves against the monotonic clock to notice.
        self._offset = self._now() / 1e6 - time.monotonic()

    def _on_connection(self, event):  # on ntcore's listener thread: logging only
        info = event.data
        what = "connected to" if event.is_(self._nt.EventFlags.kConnected) else "disconnected from"
        log.info("NetworkTables: %s %s:%d", what, info.remote_ip, info.remote_port)

    def _check_clock(self):
        offset = self._now() / 1e6 - time.monotonic()
        if abs(offset - self._offset) > 0.5:
            log.warning("the clock jumped %+.1f s: restarting NetworkTables so values aren't dropped",
                        offset - self._offset)
            self._offset = offset  # once per jump, even if the restart fails
            self._stop()
            self._start()

    def publish(self, frame):
        self._check_clock()
        nan = float("nan")
        now = time.monotonic()
        stamp = int(self._now() - (now - frame.capture_time) * 1e6)
        packed = []
        for t in frame.tags:
            pos = t.position_m if t.position_m is not None else (nan, nan, nan)
            packed += [float(t.tag_id), t.yaw_deg, t.pitch_deg, nan if t.distance_m is None else t.distance_m,
                       float(pos[0]), float(pos[1]), float(pos[2])]
        self.tags.set(packed, stamp)
        fp = frame.field_pose
        if fp is None:
            self.field_pose.set([], stamp)
        else:
            self.field_pose.set(field_pose_values(fp), stamp)
        self.frame.set(frame.seq, stamp)
        self.timings["age_ms"].set((now - frame.capture_time) * 1e3, stamp)
        self.timings["decode_ms"].set(frame.decode_ms, stamp)
        self.timings["detect_ms"].set(frame.detect_ms, stamp)
        self.inst.flush()  # send now rather than at NT's next periodic update

    def camera_connected(self, connected):
        self._connected = connected
        self.connected.set(connected)
        if not connected:
            self.tags.set([])  # no stale tags or poses while the camera is away
            self.field_pose.set([])
        self.inst.flush()

    def status(self, text):
        self._status = text
        self.status_pub.set(text)
        self.inst.flush()

    def _stop(self):
        # Close every publisher first: one left for the garbage collector unpublishes the restarted instance's
        # publishers of the same topics (ntcore reuses the handles), and the robot would see nothing again.
        for pub in (self.tags, self.field_pose, self.frame, *self.timings.values(), self.connected, self.status_pub,
                    self.version_pub):
            pub.close()
        self.inst.stopClient()
        type(self.inst).destroy(self.inst)

    def close(self):
        self._stop()


def field_pose_values(fp) -> list[float]:
    """``fp`` packed for NetworkTables, in ``FIELD_POSE_FIELDS`` order (NaN for covariances it doesn't have)."""
    nan = float("nan")
    robot_cov = np.full(9, nan) if fp.robot_covariance is None else np.ravel(fp.robot_covariance)
    camera_cov = np.full(4, nan) if fp.camera_xy_covariance is None else np.ravel(fp.camera_xy_covariance)
    return [*map(float, fp.robot_position), *fp.robot_rpy_deg, *map(float, fp.camera_position), fp.rms_px,
            float(len(fp.tag_ids)), fp.xy_std_m, fp.yaw_std_deg, *map(float, robot_cov), *map(float, camera_cov),
            *fp.gyro_std_devs()]


def make_publishers(out) -> list[Publisher]:
    pubs: list[Publisher] = []
    if out.print:
        pubs.append(StdoutPublisher())
    if out.udp:
        pubs.append(UdpPublisher(out.udp))
    if out.nt_server:
        pubs.append(NetworkTablesPublisher(out.nt_server, out.nt_table, out.nt_client_name))
    return pubs
