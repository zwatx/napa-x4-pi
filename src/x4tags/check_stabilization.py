"""Check whether the X4's webcam picture is fixed to the camera body (what the robot needs) or stabilized.

Insta360 says the X4 stitches and stabilizes its 360-degree webcam output in the camera, without saying whether the
picture's heading follows the body. Keep one tag in view, start this, then when asked turn the camera by hand about
its vertical axis by roughly 90 degrees, quickly (under a second), and hold it still. The tag's bearing in the
picture should jump by the same angle and stay there if the picture is body-fixed. If it barely moves, the picture
is held to the world (heading lock); if it barely moves at first and then creeps over, the picture follows the body
with smoothing (lag).

    python -m x4tags.check_stabilization
"""

from __future__ import annotations

import argparse
import time

import numpy as np

from .capture import X4Webcam
from .detector import X4TagDetector


def verdict(times: np.ndarray, azimuth_deg: np.ndarray, turned_deg: float | None = None) -> tuple[str, dict]:
    """Classify a bearing trace around one quick turn: compare the bearing before the turn, just after it settles
    (0.3-0.8 s after the fastest change) and at the end (last second). Returns (verdict, numbers)."""
    az = np.unwrap(np.radians(azimuth_deg))
    az = np.degrees(az)
    rate = np.gradient(az, times)
    i = int(np.argmax(np.abs(rate)))
    t0 = times[i]
    before = np.median(az[times < t0 - 0.5]) if np.any(times < t0 - 0.5) else az[0]
    early = az[(times > t0 + 0.3) & (times < t0 + 0.8)]
    late = az[times > times[-1] - 1.0]
    if early.size == 0 or late.size == 0:
        return "not enough data after the turn", {}
    jump, final = float(np.median(early) - before), float(np.median(late) - before)
    turned = abs(turned_deg) if turned_deg else max(abs(jump), abs(final))
    numbers = {"jump_deg": round(jump, 1), "final_deg": round(final, 1), "turn_at_s": round(float(t0 - times[0]), 2)}
    if turned < 20:
        return "the turn was too small to judge: turn the camera ~90 degrees", numbers
    if abs(final) < 0.25 * turned:
        return "heading-locked: the picture keeps pointing the same way as the body turns (not usable as a fixed mount)", numbers
    if abs(final - jump) > 0.1 * turned:
        return "smoothed: the picture follows the body but lags; bearings are wrong while turning", numbers
    return "body-fixed: the picture turns with the camera (a fixed mount transform is valid)", numbers


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m x4tags.check_stabilization", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--turn-deg", type=float, default=90.0, help="how far you turn the camera (roughly)")
    a = ap.parse_args(argv)
    det = X4TagDetector(near_per_frame=0)
    times, az, el = [], [], []
    with X4Webcam(a.device) as cam:
        print("hold the camera still with one tag in view...")
        start = time.monotonic()
        told = False
        while time.monotonic() - start < a.seconds:
            t, gray = cam.read(timeout=5.0)
            tags = det.detect(gray)
            if tags:
                tag = max(tags, key=lambda x: np.linalg.norm(x.corners[0] - x.corners[2]))
                b = tag.bearing_deg()
                times.append(t)
                az.append(b[0])
                el.append(b[1])
            if not told and time.monotonic() - start > 2.0:
                print("now turn the camera ~90 degrees about its vertical axis, quickly, and hold it still")
                told = True
    if len(times) < 20:
        print("the tag was seen in too few frames")
        return 1
    text, numbers = verdict(np.array(times), np.array(az), a.turn_deg)
    print(text)
    print(numbers, f"(tag elevation moved {np.ptp(el):.1f} deg over the run)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
