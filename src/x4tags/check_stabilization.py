"""Check how the X4's webcam picture follows the camera body. No tag needed.

Insta360 says the X4 stitches and stabilizes its webcam output in the camera, and webcam mode has no settings to
turn that off. Measured on one X4 (firmware of 2026-11), camera in hand, turned quickly and let go:

- the picture follows quick turns within about half a second (a ~110 degree turn showed up in ~0.5 s),
- but the last few degrees (3.7 and 8 in two tries) are taken back slowly, as an exponential with a ~5 s time
  constant, so the picture's heading lags the body's by up to that much for 10-20 s after a sharp turn,
- a camera at rest gives a still picture (< 0.002 degrees of drift in 15 s).

So treat the picture's heading as a few degrees off after the robot turns: take heading from the gyro and use the
camera *position* from the field pose (barely affected: the picture turns about the lenses, 28 mm apart).

How it measures: frames are kept at quarter size while you move the camera, then each frame's rotation relative to
the last one (camera at rest) is fitted from matched ORB features, as directions on the sphere (RANSAC + Kabsch).
That's good to ~0.05 degrees, needs no tag, and doesn't mind people moving in view.

    python -m x4tags.check_stabilization [--tilt]

Put the camera on a table with the cable slack. When told, turn it ~90 degrees about its vertical axis, quickly,
let go, and don't touch it (or the table) until the end. ``--tilt`` repeats with a ~15 degree tilt (prop one side up),
to see whether the horizon follows the body too (if it's levelled, leave ``[mount]`` pitch and roll at 0).
"""

from __future__ import annotations

import argparse
import time

import cv2
import numpy as np

from .capture import X4Webcam
from .equirect import pixel_to_ray


def _features(gray: np.ndarray):
    """ORB keypoints away from the poles (where the picture is most stretched), as rays, with descriptors."""
    h, w = gray.shape
    mask = np.zeros_like(gray)
    mask[int(h * 0.15):int(h * 0.85)] = 255
    kp, des = cv2.ORB_create(4000, scaleFactor=1.3, nlevels=5).detectAndCompute(gray, mask)
    if des is None or len(kp) < 3:
        return np.zeros((0, 3)), None
    pts = np.array([k.pt for k in kp])
    return pixel_to_ray(pts[:, 0], pts[:, 1], w, h), des


def _kabsch(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Rotation R minimising sum |R a_i - b_i|^2."""
    u, _, vt = np.linalg.svd(b.T @ a)
    return u @ np.diag([1.0, 1.0, np.sign(np.linalg.det(u @ vt))]) @ vt


def rotation_between(gray_a: np.ndarray, gray_b: np.ndarray, threshold_deg: float = 0.5, iterations: int = 500,
                     seed: int = 0) -> tuple[np.ndarray, int] | None:
    """(R, inliers) with ray_in_b ~= R @ ray_in_a, for two equirect frames from (nearly) the same place; None if
    too few features match. Things moving in view (people) and parallax fall outside the 0.5 degree threshold."""
    fa, fb = _features(gray_a), _features(gray_b)
    if fa[1] is None or fb[1] is None:
        return None
    matches = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True).match(fa[1], fb[1])
    if len(matches) < 20:
        return None
    a = fa[0][[m.queryIdx for m in matches]]
    b = fb[0][[m.trainIdx for m in matches]]
    rng = np.random.default_rng(seed)
    cos_t = np.cos(np.radians(threshold_deg))
    best = None
    for _ in range(iterations):
        pick = rng.choice(len(a), 3, replace=False)
        r = _kabsch(a[pick], b[pick])
        inliers = np.sum((a @ r.T) * b, axis=1) > cos_t
        if best is None or inliers.sum() > best.sum():
            best = inliers
    if best.sum() < 20:
        return None
    return _kabsch(a[best], b[best]), int(best.sum())


def rotation_trace(frames: list[np.ndarray]) -> np.ndarray:
    """(n, 3) rotation vectors, degrees, camera axes (x right, y down, z forward): how each frame's picture is
    turned relative to the last frame's. NaN where a frame couldn't be matched."""
    out = np.full((len(frames), 3), np.nan)
    for k, frame in enumerate(frames):
        fit = rotation_between(frames[-1], frame)
        if fit is not None:
            out[k] = np.degrees(cv2.Rodrigues(fit[0])[0].ravel())
    return out


def analyze(times, rotvecs, moved_deg: float, what: str = "turn", moving_deg_s: float = 20.0) -> tuple[str, dict]:
    """Verdict on a trace from ``rotation_trace`` around one quick move of about ``moved_deg``.

    The picture's change is its rotation at the start relative to the end (camera at rest); for a tilt only the part
    about horizontal axes counts. The hand lets go after the last moment the picture turned faster than
    ``moving_deg_s``; whatever rotation is left after that is the camera's own settling (the creep)."""
    t = np.asarray(times, float)
    w = np.asarray(rotvecs, float)
    ok = np.all(np.isfinite(w), axis=1)
    t, w = t[ok], w[ok]
    if len(t) < 10:
        return "too few frames could be matched: is the camera's view blocked?", {}
    start = np.median(w[t < t[0] + 1.5], axis=0)
    change = float(np.hypot(start[0], start[2]) if what == "tilt" else np.linalg.norm(start))
    numbers = {"change_deg": round(change, 2), "expected_deg": moved_deg}
    lock = "heading" if what == "turn" else "horizon"
    if change < 0.25 * moved_deg:
        return (f"{lock}-locked, or the camera wasn't {'turned' if what == 'turn' else 'tilted'} that far: the "
                f"picture changed by {change:.1f} degrees"), numbers
    angle = np.linalg.norm(w, axis=1)
    speed = np.linalg.norm(np.diff(w, axis=0), axis=1) / np.maximum(np.diff(t), 1e-3)
    fast = np.nonzero(speed > moving_deg_s)[0]
    if len(fast) == 0:
        return "the picture never turned quickly: move it faster (well under a second)", numbers
    after = t > t[fast[-1] + 1] + 0.2  # from just after the hand let go
    if after.sum() < 5 or t[-1] - t[fast[-1] + 1] < 8.0:
        return "the camera was still moving near the end: let go earlier and leave it until the end", numbers
    creep = float(angle[after][0])
    numbers["creep_deg"] = round(creep, 2)
    late = t > t[-1] - 3.0
    numbers["end_drift_deg_s"] = round(float(np.median(speed[late[1:]])), 3)
    if creep < 0.5:
        return f"body-fixed for this {what}: the picture moves with the camera and stays put", numbers
    fit = after & (angle > 0.05)
    if fit.sum() >= 5:
        slope = np.polyfit(t[fit], np.log(angle[fit]), 1)[0]
        if slope < 0:
            numbers["time_constant_s"] = round(float(-1 / slope), 1)
    tau = numbers.get("time_constant_s")
    how = f"with a {tau:.0f} s time constant" if tau else "slowly"
    return (f"follows the body at once but creeps the last {creep:.1f} degrees {how}: the picture's {lock} is that "
            f"far off for a while after each quick {what}"), numbers


def _record(cam, seconds: float, prompt: str, keep_every: int = 2):
    """Quarter-size frames (every ``keep_every``-th) and their times; prints ``prompt`` after 3 s."""
    frames, times = [], []
    start = time.monotonic()
    told = False
    k = 0
    while time.monotonic() - start < seconds:
        t, gray = cam.read(timeout=5.0)
        if k % keep_every == 0:
            frames.append(gray)
            times.append(t)
        k += 1
        if not told and time.monotonic() - start > 3.0:
            print(prompt, flush=True)
            told = True
    print("  stop: analysing...", flush=True)
    return np.array(times), frames


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m x4tags.check_stabilization", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device")
    ap.add_argument("--seconds", type=float, default=30.0, help="per step")
    ap.add_argument("--turn-deg", type=float, default=90.0, help="how far you'll turn the camera (roughly)")
    ap.add_argument("--tilt", action="store_true", help="also check the horizon with a tilt")
    ap.add_argument("--tilt-deg", type=float, default=15.0)
    a = ap.parse_args(argv)
    steps = [("turn", a.turn_deg, f"now turn the camera ~{a.turn_deg:.0f} degrees about its vertical axis, quickly, "
                                  f"let go, and don't touch it until told")]
    if a.tilt:
        steps.append(("tilt", a.tilt_deg, f"now tilt the camera ~{a.tilt_deg:.0f} degrees (prop one side up), "
                                          f"quickly, let go, and don't touch it until told"))
    ok = True
    with X4Webcam(a.device, reduce=4) as cam:
        try:
            _, gray = cam.read(timeout=5.0)
        except TimeoutError as e:
            print(f"no frames from the camera: {e}", flush=True)
            return 1
        if gray.shape[1] != 2 * gray.shape[0]:
            print(f"the camera sends a {gray.shape[1] * 4}x{gray.shape[0] * 4} picture: this check needs the X4's "
                  f"360-degree (2880x1440) webcam output", flush=True)
            return 1
        for what, deg, prompt in steps:
            print(f"{what}: hold the camera still, hands off...", flush=True)
            times, frames = _record(cam, a.seconds, prompt)
            text, numbers = analyze(times, rotation_trace(frames), deg, what)
            print(f"{what}: {text} {numbers}", flush=True)
            ok = ok and text.startswith("body-fixed")
    return 0 if ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
