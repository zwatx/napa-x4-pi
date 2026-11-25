# Changelog

## 0.2.0 (2026-11-24)

First release tested with a live X4 on a Raspberry Pi 5, against tags at known places on a monitor.

**Accuracy**
- The X4 is two cameras back to back: each half of the picture is now seen from its own lens, 28 mm apart. With
  tags on both sides of the camera, one centre left them 11-13 px apart at 1 m and gave no field pose at all.
- Corners near the stitch between the lenses are trusted less, and a pose no longer fails because of them.
- Undo a gamma of 2.0 before fitting tag edges (`[detector] camera_gamma`): corner error down about a third on
  rendered frames, 10% against a screen's exact geometry.
- `min_tag_px` defaults to 16: far field tags are 16-22 px in the 360-degree picture.

**Timing**
- Frame times follow the camera's steady clock instead of uneven USB arrival: about 6 ms of jitter becomes
  0.2-0.5 ms in the 360-degree mode. Times come from V4L2's buffer stamps.
- Latency defaults are 100 ms (360-degree mode) and 75 ms (1080 mode), from `check_latency`'s new moving-tag
  measurement. The old 135 and 112 ms read about half a frame plus half the exposure too long.
- `check_latency` can serve its page to another computer (`--host`), syncing that browser's clock.

**Speed** (Pi 5)
- The seam and close-tag searches run on a second core beside the full-frame search: detect 12.3 -> 8.4 ms.
- A tag that never fits no longer costs a full search every frame (46 -> 22 ms).

**The service**
- One broken output (stdout, UDP, NetworkTables) no longer stops the others; a UDP name that doesn't resolve yet is
  retried instead of crashing the service.
- Settings and files that can't work (bad values, a missing layout or replay) end with one line and exit code 78.
- systemd: `Type=notify` with a 15 s watchdog, restart back-off, hardening, and journald log priorities.
- NetworkTables: `status` and `version` topics, connection messages in the log, and a restart when the system
  clock steps back (NT would otherwise drop values until it caught up).
- With more than 8 tags in view, one tag that doesn't fit no longer stops the field pose.
- Clearer messages when the X4 is missing or in USB-drive mode; the stats line says whether the frame clock is on.
- The installer restarts the service after an update, rebuilds a mismatched `.venv`, needs no compiler, and can
  `--uninstall`.

**Changed**
- `X4TagDetector()` defaults to `min_tag_px=16` and `camera_gamma=2.0`, matching the service.
- Positions are from the middle of the camera body, between the lenses: measure `[mount]` there.

## 0.1.0 (2026-11-20)

First version: grey MJPEG capture, the seam strip, refinement of bent tags, close-tag views, robot-frame bearings
and poses, the field pose from all tags with covariances, the 1080 mode's calibrated geometry, NetworkTables / UDP /
stdout output, and the systemd service.
