# napa-x4-pi

AprilTags from an Insta360 X4's 360-degree webcam stream on a Raspberry Pi, published to the robot.
A starting point for a coprocessor: one camera that sees all the way around, one Pi, results on NetworkTables.

Detection is [NAPA](https://pypi.org/project/napa-vision/) (`napa-vision`), used as it is. This repository adds
what the X4's picture needs and the robot plumbing:

- **Grey capture.** The X4 in webcam mode sends MJPEG. `X4Webcam` takes the JPEG undecoded and decodes it straight
  to grey, instead of OpenCV's default colour decode and a grey conversion afterwards: about half the time on a Pi 5
  (25 -> 11 ms for a 2880x1440 frame from the X4, 34 -> 18 ms for busier test frames). It decodes in its own thread,
  on another core than detection, and keeps only the newest frame, so the loop never falls behind.
- **The seam.** The stitched picture is equirectangular: its left and right edges are the same direction, behind the
  camera. A tag there is cut in two and NAPA can't see it; a strip of both edges side by side is searched too.
- **Bent tags.** The 360-degree picture bends straight lines, more for big tags and away from the horizon. Tags bent
  by more than 0.2 px are looked at again through a small flat (pinhole) view centred on the tag, where their sides
  are straight again, and the corners are mapped back.
- **Close tags.** Tags closer than about half a metre come out so bent (sides bowed 5-40 px) that NAPA can't find
  them in the 360-degree picture at all. Twelve low-resolution flat views all around (centred on an icosahedron's
  corners) are searched for them, two per frame in rotation plus every view that found one on the previous frame.
- **Robot frame.** Each tag's yaw and pitch from the robot, and with the tag size its distance and pose, in WPILib's
  robot frame (x forward, y left, z up) through the camera's mount position.
- **Field pose.** With a WPILib AprilTag field layout, one pose fitted to every visible tag's corners at once. A
  360-degree camera sees tags all around, which pins its position down far better than any single tag. It needs two
  or more tags (one tag's pose is ambiguous), leaves out tags that don't fit (a misread ID, say), and comes with its
  expected position and heading error for a pose estimator.
- **Outputs.** NetworkTables 4, JSON over UDP, or JSON lines on stdout. NT values are timestamped with when the frame
  was taken (arrival minus a configured camera latency).
- **Service.** A systemd unit that starts at boot and restarts on failure; the loop reconnects when the camera drops
  out, skips frames that fail, and says so in the log.
- **Checks for the camera.** Tools that measure the X4's latency and whether its picture is fixed to the camera body
  (see "Before relying on it").

## Measured

On rendered 360-degree frames with exact corners (64 frames, 481 tags at 10-360 px, real HDR backgrounds, sensor
noise, motion blur, defocus, glare, shadows):

| | NAPA alone | x4tags |
|---|---|---|
| Tags found | 179 | 208 (the 29 more are close, strongly bent tags) |
| Worst-corner error, median / 95th percentile | 0.50 / 5.5 px | 0.26 / 1.9 px |
| Same, on tags both find that are bent over 1.5 px | 1.60 / 9.5 px | 0.23 / 1.7 px |
| Tags found when each is moved across the seam | 0 | 200 |

Field pose, in a rendered 6 m x 4 m room with 12 tags (7-11 in view), 40 robot poses, blur and noise: camera
position error 0.2 mm median, 0.3 mm 95th percentile, against 5.6 mm and 4.3 cm from the biggest single tag. That is
with perfect geometry (exact stitch and tag positions); a real camera and field will do worse.

Range: the stitched picture has 458 px per radian (0.125 degrees per pixel). A face-on FRC tag (0.1651 m) is 22 px,
NAPA's default minimum, at 3.4 m, and 16 px at 4.7 m. On the rendered frames, NAPA found 32% of tags at 22-26 px,
54% at 40-56 px and 74% at 80-120 px, through all the blur, glare and tilt those frames include.

Raspberry Pi 5, the service on those frames as quality-90 JPEGs paced at 30 fps, one detector thread: 30.0 fps;
decode 17.8 ms; detection 14.7 ms (95th percentile 22.9); frame age when results are published 34 ms (95th
percentile 46); 59 C, no throttling. In the room with 11 tags in view and the field pose on: detection plus field
pose 15.7 ms, frame age 29 ms. The X4's own delay (stitching, encoding, USB) comes on top: about 140 ms (below).

The live X4 (on a laptop): 29.3 fps from the driver, unevenly (about 5 gaps of over 50 ms a second, whatever the
buffer count); about 216 kB per frame, baseline 4:2:0 JPEG with no restart markers (so no decoding part of a
frame). Grey decode 8.6 ms, results published 11 ms after a frame arrives. From something changing in front of
the camera to the frame arriving: 154-163 ms median in three runs of 20-30 flips (10-90%: 146-176 ms), including the
laptop screen's own delay, so about 140 ms for the camera; the default `latency_ms` is 140.

## Hardware

- Raspberry Pi 5 (a Pi 4 should work, slower), 64-bit Raspberry Pi OS or Debian 12+.
- Insta360 X4 set to **webcam mode** (USB menu on the camera), plugged **straight into the Pi**, not through a hub
  or dock: some docks power it but never pass data. It then shows up as a UVC camera; `x4tags` finds it by its USB
  vendor ID. It may fall back to USB-drive mode; choose webcam mode again on the camera.
- Use the 2880x1440 360-degree output. (The 1920x1080 mode is two stacked flattened half-panoramas, not this.)
  Feed the X4 from a supply that can give 5 V 3 A; it heats up when charging and streaming at once.

## Install on the Pi

```sh
git clone <this repository> napa-x4-pi && cd napa-x4-pi
deploy/install.sh            # venv, /etc/x4tags.toml, systemd service (asks for sudo for those)
sudo nano /etc/x4tags.toml   # set [mount], [tags], [field] and [output], then:
sudo systemctl restart x4tags && journalctl -u x4tags -f
```

`deploy/install.sh --no-service` only builds `.venv` (no sudo if a C compiler and python3-venv are installed).
Try it by hand first:

```sh
.venv/bin/x4tags --print                         # camera -> JSON lines
.venv/bin/x4tags --replay some.mjpg --print      # recorded MJPEG, or a folder of images
```

## Before relying on it

Two things about the X4 aren't published anywhere and decide how far to trust the output. Check both once, with the
X4 plugged into a computer with a screen (any Linux machine with this installed):

1. **Does the picture follow the camera body?** Insta360 says the X4 stitches and *stabilizes* its 360-degree
   webcam output in the camera, and webcam mode has no settings to turn that off. Measured on one X4 (2026-11): the
   picture follows quick turns within about half a second, but the last few degrees (3-8 after a quick hand turn)
   come back slowly, with a ~5 s time constant. A camera at rest gives a still picture (under 0.002 degrees of drift
   in 15 s). So for 10-20 s after the robot turns, everything that depends on the picture's heading is a few
   degrees off: bearings to tags, the robot heading from `field_pose`, and (through the mount offset) its robot
   position. The *camera's* position isn't affected: the picture turns about the lens.

   Take heading from the robot's gyro and use `field_pose`'s camera position with it
   (`x4tags.localize.robot_position_from_heading` shows the arithmetic); to aim at a tag, work from field positions
   rather than the raw bearing. To check your camera (firmware may change this), no tag needed:

   ```sh
   .venv/bin/python -m x4tags.check_stabilization --tilt   # turn it, let go; then tilt it, let go, when told
   ```

   It reports *body-fixed*, *follows ... but creeps* (with how far and how slowly), or *heading-* /
   *horizon-locked*. If the horizon is locked, leave `[mount]` pitch and roll at 0.

2. **How late are frames?** Open the page it serves in a browser on the same computer, full screen, and point the X4
   at the screen; it flips a tag's ID and times how long the change takes to arrive. That includes the screen's own
   delay (typically 17-50 ms), so it's an upper bound: put the median, less your screen's delay, in
   `[camera] latency_ms` (one X4 measured ~140 ms, the default). Dismiss any browser or system pop-ups first:
   one over the tag stops it being seen.

   ```sh
   .venv/bin/python -m x4tags.check_latency          # then open http://localhost:8765
   ```

## Settings

All in `/etc/x4tags.toml`; `config.example.toml` lists every key. The ones to set:

- `[mount]`: where the camera is on the robot and how it's turned (WPILib conventions: + pitch tips the view down,
  + yaw turns it left; zero means the middle of the picture looks along the robot's +x).
- `[tags] size_m`: black-square side (FRC: 0.1651). Without it you get bearings only.
- `[field] layout`: a WPILib AprilTag field layout JSON (the season's file from WPILib) for `field_pose`. Keep
  `min_tags` at 2 or more: a single tag's pose has two nearly equally good fits, metres apart.
- `[output] nt_server`: your team number (or a host, or `localhost:5810` for a simulator).
- `[camera] latency_ms`: from `check_latency`. Timestamps are moved back by this much; at 4 m/s every 100 ms
  unaccounted for is 0.4 m of error.
- `[detector] min_tag_px`: 22 by default; 16 finds more distant tags for more time. `near_scale = 0` turns the
  close-tag views off (saves ~5 ms per frame on a Pi 5).

## What the robot sees

NetworkTables 4, table `/x4tags` (rename with `nt_table`):

| Topic | Type | |
|---|---|---|
| `tags` | double[] | 7 values per tag: id, yaw_deg, pitch_deg, distance_m, x_m, y_m, z_m |
| `field_pose` | double[] | with a layout: robot x, y, z (m), roll, pitch, yaw (deg), camera x, y, z (m), rms_px, tag count, xy_std_m, yaw_std_deg; empty when there's no fit |
| `frame`, `age_ms`, `decode_ms`, `detect_ms` | int, double | per frame |
| `camera_connected` | boolean | true while frames arrive; `tags` and `field_pose` are emptied when it goes false |

Yaw (left of robot forward) and pitch (above horizontal) are the direction of each tag's centre from the camera, in
robot axes. `x_m, y_m, z_m` is the tag's centre in the robot frame; distance and position are NaN without
`tags.size_m` or when the pose fits poorly. Bearings go through the picture's heading, so they too lag a few
degrees after quick turns. All of a frame's tags are in one array so they can't be mixed with another frame's, and
every value is timestamped with when the frame was taken. In Java:

```java
DoubleArraySubscriber sub = NetworkTableInstance.getDefault().getTable("x4tags")
    .getDoubleArrayTopic("tags").subscribe(new double[0]);

TimestampedDoubleArray latest = sub.getAtomic();
double takenSeconds = latest.serverTime / 1e6;            // when the frame was taken (FPGA time base)
for (int i = 0; i + 7 <= latest.value.length; i += 7) {
    int id = (int) latest.value[i];
    double yawDeg = latest.value[i + 1], distanceM = latest.value[i + 3];
    // ... ignore it if Timer.getFPGATimestamp() - takenSeconds is too old
}
```

`field_pose` reads the same way (13 values). The X4's picture lags its body by a few degrees after quick turns
(see *Before relying on it*), so don't use its yaw as a heading measurement: take the camera's x, y (values 6 and
7), and the robot's position from them with the gyro's heading at `takenSeconds`:

```java
TimestampedDoubleArray pose = poseSub.getAtomic();          // poseSub: the field_pose topic, subscribed like tags
double[] v = pose.value;
double takenSeconds = pose.serverTime / 1e6;
if (v.length == 13) {
    Rotation2d heading = poseEstimator.sampleAt(takenSeconds)   // gyro-based heading when the frame was taken
        .map(Pose2d::getRotation).orElse(poseEstimator.getEstimatedPosition().getRotation());
    Translation2d mount = new Translation2d(MOUNT_X_M, MOUNT_Y_M);   // the [mount] x_m, y_m
    Translation2d robot = new Translation2d(v[6], v[7]).minus(mount.rotateBy(heading));
    poseEstimator.addVisionMeasurement(new Pose2d(robot, heading), takenSeconds,
        VecBuilder.fill(v[11], v[11], Double.POSITIVE_INFINITY));   // xy_std_m; heading from the gyro only
}
```

`xy_std_m` and `yaw_std_deg` come from the fit's geometry and an assumed corner error (`corner_noise_px`): two
small, distant tags in nearly the same direction can fit well and still be 10-20 cm out, and they say so (typical
well-spread poses: 2-3 cm). The robot x, y, z in values 0-2 go through the picture's heading: with the camera
0.3 m from the robot's centre, a 5 degree lag moves them 2.6 cm.

UDP and stdout carry one JSON object per frame with all of that plus each tag's corners (pixels in the stitched
frame), full pose rotation (columns: the tag's axes towards its printed left edge, its printed bottom, and out of
its face, in robot axes), hamming distance and decision margin.

## Layout

| Path | |
|---|---|
| `src/x4tags/capture.py` | `X4Webcam`: find the X4, newest-frame grey capture with decoding in its own thread |
| `src/x4tags/equirect.py` | pixel <-> direction, flat views of the sphere, how much a tag is bent |
| `src/x4tags/detector.py` | `X4TagDetector`: NAPA + seam + close-tag views + bent-tag refinement + single-tag pose |
| `src/x4tags/field.py`, `localize.py` | WPILib field layouts; the all-tags field pose |
| `src/x4tags/mount.py`, `results.py` | camera -> robot frame, per-frame results |
| `src/x4tags/publish.py` | NetworkTables, UDP, stdout |
| `src/x4tags/app.py`, `config.py` | the service loop and its settings |
| `src/x4tags/check_latency.py`, `check_stabilization.py` | the camera checks |
| `deploy/` | install script and systemd unit |
| `tests/` | renders its own 360-degree frames and rooms of tags with exact truth; `pip install -e ".[test,nt]" && pytest` |

## Using the pieces directly

```python
from x4tags import X4Webcam, X4TagDetector
from x4tags.field import FieldLayout
from x4tags.localize import FieldLocalizer

det = X4TagDetector(min_tag_px=22)                # other NapaDetector options pass through
loc = FieldLocalizer(FieldLayout.load("field.json"), tag_size_m=0.1651)
with X4Webcam() as cam:
    t, gray = cam.read()                          # newest frame, grey
    tags = det.detect(gray)
    for tag in tags:
        print(tag.tag_id, tag.bearing_deg(), tag.refined, tag.seam, tag.near)
        pose = det.pose(tag, 0.1651)              # (R, t, error px) in the camera frame, or None
    field_pose = loc.solve(tags)                  # FieldPose or None
```

Camera frame: x right, y down, z forward (the middle of the picture). Corners are in NAPA's order: corner 0 is the
printed bottom-right, then bottom-left, top-left, top-right; in WPILib's tag frame (x out of the face, y to a
viewer's right, z up) those are (0, s/2, -s/2), (0, -s/2, -s/2), (0, -s/2, s/2), (0, s/2, s/2). A tag across the
seam keeps its quad whole, so a corner's column can be below 0 or past the width.

## Not done yet

- Run against a live X4 on a laptop (capture, timing, stabilization, latency), not yet on a Pi. Values sent before NetworkTables has synchronised its clock with the robot may carry the Pi's own time
  base; check timestamps on a real robot.
- Range is short (see Measured); distant tags need a narrower camera.
- The stitch is from two lenses about 3 cm apart, and the X4 stitches in the camera; close tags on a stitch line can
  be slightly misplaced, which the rendered tests don't model.
- Single-tag poses don't report the IPPE ambiguity; a face-on tag's rotation can be a few degrees off.

## License

GPL-3.0-only, like NAPA.
