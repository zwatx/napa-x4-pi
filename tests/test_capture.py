"""FrameClock: steady frame times from late, uneven arrivals (no camera needed)."""

import numpy as np

from x4tags.capture import FrameClock

PERIOD = 1 / 29.976


def arrivals(seconds=30.0, period=PERIOD, max_late=0.025, drop=0.03, noise=0.0002, seed=0, start=1234.5):
    """(capture times, arrival times) for a steady camera whose frames arrive 0..max_late late; some never arrive."""
    rng = np.random.default_rng(seed)
    cap = start + np.arange(int(seconds / period)) * period
    keep = rng.random(len(cap)) >= drop
    cap = cap[keep]
    late = rng.uniform(0, max_late, len(cap)) + rng.normal(0, noise, len(cap))
    return cap, cap + late


def test_steady_times_lose_the_jitter():
    cap, arr = arrivals()
    clock = FrameClock()
    out = np.array([clock(t) for t in arr])
    assert clock.locked
    assert abs(1 / clock.period - 29.976) < 0.005
    settled = slice(120, None)  # after ~4 s
    raw = arr[settled] - cap[settled]
    steady = out[settled] - cap[settled]
    assert raw.std() > 0.006  # ~7 ms jitter in arrivals
    assert steady.std() < 0.0012  # ~0.7 ms after (6x+ less); 0.4 ms on a real X4 on a Pi 5
    assert abs(steady.mean() - raw.mean()) < 0.002  # same mean as the arrivals, so latency_ms still applies


def test_not_locked_returns_arrivals_unchanged():
    cap, arr = arrivals(seconds=2.0)
    clock = FrameClock()
    assert [clock(t) for t in arr] == list(arr)
    assert not clock.locked


def test_a_camera_without_a_steady_clock_is_left_alone():
    rng = np.random.default_rng(1)
    arr = 50.0 + np.cumsum(rng.uniform(0.010, 0.060, 600))  # frames whenever
    clock = FrameClock()
    out = np.array([clock(t) for t in arr])
    assert np.mean(out == arr) > 0.9


def test_follows_a_slow_drift_and_relocks_after_a_stall():
    cap, arr = arrivals(seconds=20.0, period=PERIOD * (1 + 150e-6))
    cap2, arr2 = arrivals(seconds=20.0, seed=3, start=arr[-1] + 2.0)  # a 2 s stall, then a new phase
    clock = FrameClock()
    out = np.array([clock(t) for t in np.concatenate([arr, arr2])])
    err = out - np.concatenate([cap, cap2])
    first, second = err[300:len(cap)], err[len(cap) + 120:]
    assert first.std() < 0.0012 and second.std() < 0.0012


def cyclic(pattern_ms, seconds=30.0, noise_ms=0.5, drop=0.02, seed=0, period=PERIOD, start=500.0):
    """Arrivals late by a repeating pattern (the X4 delivers frames in threes), plus noise; some frames never come."""
    rng = np.random.default_rng(seed)
    k = np.arange(int(seconds / period))
    cap = start + k * period
    late = np.asarray(pattern_ms)[k % len(pattern_ms)] / 1e3 + np.abs(rng.normal(0, noise_ms / 1e3, len(k)))
    keep = rng.random(len(k)) >= drop
    return cap[keep], (cap + late)[keep]


def test_the_x4s_pattern_in_threes_locks_and_keeps_every_frame_in_its_own_slot():
    cap, arr = cyclic([16, 9, 0])  # 2880x1440 on a Pi 5
    clock = FrameClock()
    out = np.array([clock(t) for t in arr])
    assert clock.locked
    assert np.all(np.diff(out) > 0.5 * PERIOD)  # one frame per slot: never the same or an earlier time
    err = (out - cap)[120:]
    assert err.std() < 0.001 < (arr - cap)[120:].std()


def test_delays_spanning_more_than_a_frame_fall_back_to_arrivals():
    # 1920x1080 on a Pi 5: about 1, 30 and 7 ms late, wandering by a few ms, so some frames are over a period late
    cap, arr = cyclic([1, 30, 7], noise_ms=2.0)
    clock = FrameClock()
    out = np.array([clock(t) for t in arr])
    assert np.all(np.diff(out) > 0)
    assert (out - cap)[120:].std() <= (arr - cap)[120:].std() + 1e-4  # never worse than the arrivals


def test_a_new_frame_rate_drops_the_lock_within_seconds():
    cap, arr = arrivals(seconds=20.0)
    cap2 = arr[-1] + 0.04 + np.arange(400) * 0.040  # the camera switches to 25 fps
    arr2 = cap2 + np.random.default_rng(5).uniform(0, 0.02, len(cap2))
    clock = FrameClock()
    out = np.array([clock(t) for t in np.concatenate([arr, arr2])])
    assert np.mean(out[len(arr) + 100:] == arr2[100:]) > 0.95  # back to arrival times after 4 s


def test_without_a_steady_clock_it_tries_to_lock_only_now_and_then():
    rng = np.random.default_rng(2)
    arr = 10.0 + np.cumsum(rng.uniform(0.010, 0.060, 900))
    clock = FrameClock()
    calls = []
    lock = clock._lock
    clock._lock = lambda: calls.append(1) or lock()
    for t in arr:
        clock(t)
    assert len(calls) <= 10  # backs off: 10, 20, 40 ... frames apart, up to 300


def test_one_very_late_frame_doesnt_push_the_next_ones_a_slot_late():
    for spike_ms in (25, 40, 60):
        cap, arr = cyclic([16, 9, 0], seconds=20.0, drop=0.0)
        k = 300 + int(np.argmax(np.round((arr[300:303] - cap[300:303]) * 1e3) == 16))  # a 16 ms frame
        arr[k] += spike_ms / 1e3  # a USB hiccup: the frames queued behind it come no sooner
        arr[k + 1:] = np.maximum(arr[k + 1:], arr[k] + 0.0005 * np.arange(1, len(arr) - k))
        clock = FrameClock()
        out = np.array([clock(t) for t in arr])
        assert clock.locked
        assert np.all(out[120:] - arr[120:] < 0.020)  # never later than its arrival plus about the mean delay
        off = (out - cap - np.mean(out[120:] - cap[120:]))[120:]
        assert np.sum(np.abs(off) > 0.008) <= 3  # the hiccup's frames, not the next 40
