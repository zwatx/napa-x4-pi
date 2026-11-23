"""Where results go: stdout (JSON lines), UDP (one JSON datagram per frame), NetworkTables 4 (needs pyntcore)."""

from __future__ import annotations

import json
import logging
import socket
import sys
import time

import numpy as np

from .results import FrameResult

log = logging.getLogger(__name__)


class Publisher:
    def publish(self, frame: FrameResult) -> None:
        raise NotImplementedError

    def camera_connected(self, connected: bool) -> None:
        pass

    def close(self) -> None:
        pass


class StdoutPublisher(Publisher):
    def publish(self, frame):
        sys.stdout.write(json.dumps(frame.to_dict(time.monotonic())) + "\n")
        sys.stdout.flush()


class UdpPublisher(Publisher):
    def __init__(self, target: str):
        host, port = target.rsplit(":", 1)
        # Resolve once: a name lookup per frame (mDNS .local names especially) could stall the loop.
        self.addr = socket.getaddrinfo(host, int(port), socket.AF_INET, socket.SOCK_DGRAM)[0][4]
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def publish(self, frame):
        data = json.dumps(frame.to_dict(time.monotonic())).encode()
        try:
            self.sock.sendto(data, self.addr)
        except OSError as e:  # no route yet (robot not up): keep running
            log.debug("udp send failed: %s", e)

    def close(self):
        self.sock.close()


TAG_FIELDS = ("id", "yaw_deg", "pitch_deg", "distance_m", "x_m", "y_m", "z_m")
FIELD_POSE_FIELDS = ("robot_x_m", "robot_y_m", "robot_z_m", "roll_deg", "pitch_deg", "yaw_deg", "camera_x_m", "camera_y_m",
                     "camera_z_m", "rms_px", "tag_count", "xy_std_m", "yaw_std_deg",
                     *(f"robot_cov_{a}_{b}" for a in ("x", "y", "yaw") for b in ("x", "y", "yaw")),
                     *(f"camera_cov_{a}_{b}" for a in ("x", "y") for b in ("x", "y")),
                     "gyro_std_forward_m", "gyro_std_left_m")


class NetworkTablesPublisher(Publisher):
    """Publishes under /<table>/ (default /x4tags/):

    - ``tags`` (double[]): every tag of a frame packed in one array, ``len(TAG_FIELDS)`` = 7 values per tag:
      id, yaw_deg, pitch_deg, distance_m, x_m, y_m, z_m (robot frame; distance and position NaN without a pose).
      One array, so a reader never mixes one frame's IDs with another frame's bearings.
    - ``field_pose`` (double[]): with a field layout, the pose from all tags together, 28 values (``FIELD_POSE_FIELDS``):
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

        self._now = ntcore._now  # NT's local clock, microseconds (the time base set(value, time) expects)
        self.inst = ntcore.NetworkTableInstance.create()
        self.inst.startClient4(client_name)
        host, _, port = str(server).partition(":")
        port = int(port) if port else 0  # 0: NT4's default port
        if host.isdigit():
            self.inst.setServerTeam(int(host), port)
        else:
            self.inst.setServer(host, port)
        t = self.inst.getTable(table)
        self.tags = t.getDoubleArrayTopic("tags").publish()
        self.field_pose = t.getDoubleArrayTopic("field_pose").publish()
        self.frame = t.getIntegerTopic("frame").publish()
        self.timings = {k: t.getDoubleTopic(k).publish() for k in ("age_ms", "decode_ms", "detect_ms")}
        self.connected = t.getBooleanTopic("camera_connected").publish()

    def publish(self, frame):
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
        self.connected.set(connected)
        if not connected:
            self.tags.set([])  # no stale tags or poses while the camera is away
            self.field_pose.set([])
        self.inst.flush()

    def close(self):
        self.inst.stopClient()
        type(self.inst).destroy(self.inst)


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
