"""Where results go: stdout (JSON lines), UDP (one JSON datagram per frame), NetworkTables 4 (needs pyntcore)."""

from __future__ import annotations

import json
import logging
import socket
import sys
import time

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
        self.addr = (host, int(port))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def publish(self, frame):
        data = json.dumps(frame.to_dict(time.monotonic())).encode()
        try:
            self.sock.sendto(data, self.addr)
        except OSError as e:  # no route yet (robot not up): keep running
            log.debug("udp send failed: %s", e)

    def close(self):
        self.sock.close()


class NetworkTablesPublisher(Publisher):
    """Publishes under /<table>/: per-tag arrays in the same order (ids, yaw_deg, pitch_deg, distance_m, x_m, y_m,
    z_m; NaN where no pose), plus frame, age_ms, decode_ms, detect_ms and camera_connected.

    Every value of a frame carries the frame's capture time as its NT timestamp, so robot code can read
    ``getAtomic().serverTime`` for latency compensation instead of trusting age_ms."""

    def __init__(self, server: str, table: str = "x4tags", client_name: str = "x4tags"):
        """``server``: a team number ("1234"), or a host ("10.12.34.2", "localhost"), optionally ":port"."""
        import ntcore  # optional dependency: pip install "napa-x4-pi[nt]"

        self._now = ntcore._now  # NT's local clock, microseconds
        self.inst = ntcore.NetworkTableInstance.create()
        self.inst.startClient4(client_name)
        host, _, port = server.partition(":")
        port = int(port) if port else 0  # 0: NT4's default port
        if host.isdigit():
            self.inst.setServerTeam(int(host), port)
        else:
            self.inst.setServer(host, port)
        t = self.inst.getTable(table)
        self.ids = t.getIntegerArrayTopic("ids").publish()
        self.arrays = {k: t.getDoubleArrayTopic(k).publish() for k in ("yaw_deg", "pitch_deg", "distance_m", "x_m", "y_m", "z_m")}
        self.frame = t.getIntegerTopic("frame").publish()
        self.timings = {k: t.getDoubleTopic(k).publish() for k in ("age_ms", "decode_ms", "detect_ms")}
        self.connected = t.getBooleanTopic("camera_connected").publish()

    def publish(self, frame):
        nan = float("nan")
        now = time.monotonic()
        stamp = int(self._now() - (now - frame.capture_time) * 1e6)
        tags = frame.tags
        self.ids.set([t.tag_id for t in tags], stamp)
        cols = {
            "yaw_deg": [t.yaw_deg for t in tags],
            "pitch_deg": [t.pitch_deg for t in tags],
            "distance_m": [t.distance_m if t.distance_m is not None else nan for t in tags],
            "x_m": [float(t.position_m[0]) if t.position_m is not None else nan for t in tags],
            "y_m": [float(t.position_m[1]) if t.position_m is not None else nan for t in tags],
            "z_m": [float(t.position_m[2]) if t.position_m is not None else nan for t in tags],
        }
        for k, v in cols.items():
            self.arrays[k].set(v, stamp)
        self.frame.set(frame.seq, stamp)
        self.timings["age_ms"].set((now - frame.capture_time) * 1e3, stamp)
        self.timings["decode_ms"].set(frame.decode_ms, stamp)
        self.timings["detect_ms"].set(frame.detect_ms, stamp)
        self.inst.flush()  # send now rather than at NT's next periodic update

    def camera_connected(self, connected):
        self.connected.set(connected)
        self.inst.flush()

    def close(self):
        self.inst.stopClient()
        type(self.inst).destroy(self.inst)


def make_publishers(out) -> list[Publisher]:
    pubs: list[Publisher] = []
    if out.print:
        pubs.append(StdoutPublisher())
    if out.udp:
        pubs.append(UdpPublisher(out.udp))
    if out.nt_server:
        pubs.append(NetworkTablesPublisher(out.nt_server, out.nt_table, out.nt_client_name))
    return pubs
