"""Check whether the X4's webcam picture is fixed to the camera body (what the robot needs) or stabilized.

Insta360 says the X4 stitches and stabilizes its 360-degree webcam output in the camera, without saying whether the
picture's heading or horizon follow the body. Keep one tag in view (about a metre away, roughly in front), start
this, and when asked turn the camera by hand about its vertical axis by roughly 90 degrees, quickly (well under a
second), and hold it still. The tag's bearing in the picture should change by the same angle at once and stay
there if the picture is body-fixed. With ``--tilt`` it then asks for a ~30 degree forward tilt, to check the
horizon the same way (a levelled picture would make ``[mount]`` pitch/roll count twice).

    python -m x4tags.check_stabilization [--tilt]
"""

from __future__ import annotations

import argparse
import time

import numpy as np

from .capture import X4Webcam
from .detector import X4TagDetector


def _median_filter(x: np.ndarray, k: int = 5) -> np.ndarray:
    pad = np.pad(x, k // 2, mode="edge")
    return np.median(np.lib.stride_tricks.sliding_window_view(pad, k), axis=1)


def verdict(times: np.ndarray, angle_deg: np.ndarray, moved_deg: float, what: str = "turn") -> tuple[str, dict]:
    """Classify a bearing (or elevation) trace around one quick hand movement of about ``moved_deg``.

    Compares the angle before (first 1.5 s) and after (last 1 s), and how long the change took (10% to 90%): a
    picture fixed to the body changes as fast as the hand; a smoothed one lags behind it."""
    times = np.asarray(times, float)
    a = _median_filter(np.degrees(np.unwrap(np.radians(np.asarray(angle_deg, float)))))
    before = np.median(a[times < times[0] + 1.5])
    after = np.median(a[times > times[-1] - 1.0])
    change = float(after - before)
    numbers = {"change_deg": round(change, 1), "expected_deg": moved_deg}
    lock = "heading" if what == "turn" else "horizon"
    if abs(change) < 0.25 * abs(moved_deg):
        return (f"{lock}-locked, or the camera wasn't {'turned' if what == 'turn' else 'tilted'}: the picture didn't "
                f"follow the body (check you moved it about {moved_deg:.0f} degrees)"), numbers
    frac = (a - before) / change
    t10 = times[np.argmax(frac >= 0.1)]
    t90 = times[np.argmax(frac >= 0.9)]
    numbers["rise_s"] = round(float(t90 - t10), 2)
    if t90 - t10 > 1.0:
        return (f"smoothed: the picture took {t90 - t10:.1f} s to follow (or the {what} itself was that slow: do it "
                f"in well under a second and run again)"), numbers
    if not 0.5 * abs(moved_deg) <= abs(change) <= 1.5 * abs(moved_deg):
        return (f"follows the body at once, but by {abs(change):.0f} degrees: if that's how far you moved it, "
                f"it's body-fixed"), numbers
    return f"body-fixed for this {what}: the picture moves with the camera", numbers


def _record(cam, det, seconds, prompt, tag_id=None):
    times, az, el = [], [], []
    start = time.monotonic()
    told = False
    while time.monotonic() - start < seconds:
        t, gray = cam.read(timeout=5.0)
        tags = det.detect(gray)
        if tag_id is None and tags:
            tag_id = max(tags, key=lambda x: float(np.arccos(np.clip(x.rays[0] @ x.rays[2], -1, 1)))).tag_id
            print(f"  following tag {tag_id}")
        tag = next((x for x in tags if x.tag_id == tag_id), None)
        if tag is not None:
            b = tag.bearing_deg()
            times.append(t)
            az.append(b[0])
            el.append(b[1])
        if not told and time.monotonic() - start > 2.0:
            print(prompt)
            told = True
    return tag_id, np.array(times), np.array(az), np.array(el)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m x4tags.check_stabilization", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device")
    ap.add_argument("--seconds", type=float, default=10.0, help="per step")
    ap.add_argument("--turn-deg", type=float, default=90.0, help="how far you'll turn the camera (roughly)")
    ap.add_argument("--tilt", action="store_true", help="also check the horizon with a forward tilt")
    ap.add_argument("--tilt-deg", type=float, default=30.0)
    a = ap.parse_args(argv)
    det = X4TagDetector(near_per_frame=0)
    ok = True
    with X4Webcam(a.device) as cam:
        print("hold the camera still with a tag in view...")
        tag_id, t, az, _ = _record(cam, det, a.seconds,
                                   f"now turn the camera ~{a.turn_deg:.0f} degrees about its vertical axis, quickly, and hold it")
        if len(t) < 20:
            print("the tag was seen in too few frames")
            return 1
        text, numbers = verdict(t, az, a.turn_deg, "turn")
        print("heading:", text, numbers)
        ok = text.startswith("body-fixed")
        if a.tilt:
            print("turn back so the tag is in front, hold still...")
            time.sleep(3.0)
            _, t, _, el = _record(cam, det, a.seconds,
                                  f"now tip the camera's front down ~{a.tilt_deg:.0f} degrees, quickly, and hold it", tag_id)
            if len(t) < 20:
                print("the tag was seen in too few frames during the tilt")
                return 1
            text, numbers = verdict(t, el, a.tilt_deg, "tilt")
            print("horizon:", text, numbers)
            ok = ok and text.startswith("body-fixed")
    return 0 if ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
