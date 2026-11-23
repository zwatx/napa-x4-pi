# napa-x4-pi

Reads AprilTags from an Insta360 X4's 360-degree webcam stream on a Raspberry Pi and sends them to the robot over
NetworkTables. It's a starting point for a vision coprocessor built around one camera that sees all the way around.

Detection is [NAPA](https://pypi.org/project/napa-vision/) (`napa-vision`), unchanged. This repository adds what the
X4's picture needs, and the robot plumbing.

## What it adds

- **Grey capture.** In webcam mode the X4 sends MJPEG. `X4Webcam` decodes each JPEG straight to grey instead of
  decoding colour and converting it afterwards. On a Pi 5 that halves the decode time: 25 to 11 ms for a 2880x1440
  frame from the X4, 34 to 18 ms for busier test frames. Decoding runs in its own thread on another core, and only
  the newest frame is kept, so the loop never falls behind.
- **The seam.** The 360-degree picture is equirectangular, so its left and right edges both look straight back. A tag
  on that line is cut in two and NAPA can't read it, so x4tags also searches a strip made of both edges side by side.
- **Bent tags.** The 360-degree picture bends straight lines, more so for big tags and for tags far above or below the
  horizon. When a tag's sides bow by more than 0.2 px, x4tags looks at it again through a small flat (pinhole) view
  centred on the tag, where the sides are straight, and maps the corners back.
- **Close tags.** Closer than about half a metre, a tag's sides bow by 5-40 px and NAPA can't find it in the
  360-degree picture at all. Twelve low-resolution flat views, centred on the corners of an icosahedron, cover the
  sphere for these. Each frame searches two of them in rotation, plus any view that found a tag on the previous frame.
- **Robot frame.** Each tag's yaw and pitch from the robot, and, given the tag size, its distance and pose. All of it
  is in WPILib's robot frame (x forward, y left, z up), through the camera's mount position.
- **Field pose.** Given a WPILib AprilTag field layout, one pose fitted to the corners of every visible tag at once. A
  camera that sees tags all around pins its position down far better than any single tag can. The fit needs at least
  two tags, because one tag's pose is ambiguous, and it leaves out tags that don't fit, such as a misread ID. Each
  pose comes with covariances for a pose estimator.
- **Outputs.** NetworkTables 4, JSON over UDP, or JSON lines on stdout. NetworkTables values carry the time the frame
  was taken: its arrival time minus the configured camera latency.
- **Service.** A systemd unit starts it at boot and restarts it if it fails. The loop reconnects when the camera drops
  out and skips frames that fail, and logs both.
- **Camera checks.** Two tools measure the X4's latency and whether its picture stays fixed to the camera body. See
  [Before relying on it](#before-relying-on-it).

## Measured

On rendered 360-degree frames with exact corners: 64 frames and 481 tags from 10 to 360 px, with real HDR
backgrounds, sensor noise, motion blur, defocus, glare and shadows.

| | NAPA alone | x4tags |
|---|---|---|
| Tags found | 179 | 208 (the extra 29 are close, strongly bent tags) |
| Worst-corner error, median / 95th percentile | 0.50 / 5.5 px | 0.26 / 1.9 px |
| The same, for tags both find that are bent by over 1.5 px | 1.60 / 9.5 px | 0.23 / 1.7 px |
| Tags found when each is moved onto the seam | 0 | 200 |

**Field pose.** In a rendered 6 m x 4 m room with 12 tags (7-11 in view) and 40 robot poses with blur and noise, the
camera position was off by 0.2 mm median and 0.3 mm at the 95th percentile. The biggest single tag alone gave 5.6 mm
and 4.3 cm. That's with perfect geometry, an exact stitch and exactly placed tags. A real camera on a real field will
do worse.

**Range.** The 360-degree picture has 458 px per radian, or 0.125 degrees per pixel. A face-on FRC tag (0.1651 m) is
22 px wide at 3.4 m, which is NAPA's default minimum, and 16 px at 4.7 m. On the rendered frames NAPA found 32% of
tags at 22-26 px, 54% at 40-56 px and 74% at 80-120 px, through all the blur, glare and tilt in those frames.

**Raspberry Pi 5.** The service ran on those frames as quality-90 JPEGs paced at 30 fps, with one detector thread, and
kept up at 30.0 fps. Decoding took 17.8 ms and detection 14.7 ms (22.9 at the 95th percentile). Results went out
34 ms after a frame arrived (46 at the 95th percentile), and the Pi held 59 C without throttling. In the room with 11
tags in view and the field pose on, detection plus the field pose took 15.7 ms, and results went out after 29 ms.
The X4's own delay for stitching, encoding and USB comes on top of that: about 140 ms, measured below.

**The live X4, on a laptop.** The driver delivers 29.3 fps, unevenly: about five gaps of over 50 ms every second,
whatever the buffer count. Frames are about 216 kB of baseline 4:2:0 JPEG with no restart markers, so a frame can't
be split up and decoded in parallel. Grey decoding took 8.6 ms, and results went out 11 ms after a frame arrived.
From a change in front of the camera to its frame arriving took 154-163 ms median, over three runs of 20-30 screen
flips (146-176 ms from the 10th to the 90th percentile). That includes the laptop screen's own delay, which leaves
about 140 ms for the camera. The default `latency_ms` is 140.

**The 1920x1080 mode** (`[camera] mode = "dual1080"`, see [Hardware](#hardware)). The same screen flips took 133 ms,
about 115 ms for the camera, which is that mode's default. Its picture has 615 px per radian at the middle of each
view and about 820 at 60 degrees to either side, against 458 in the 360-degree mode. A tag stays above the 22 px
minimum out to 4.5-6 m instead of 3.4 m. With 13 tags on a screen in view, decoding took 2.7 ms against 4.7,
detection took about 5.4 ms in both, and corner directions agreed with the 360-degree mode's to 0.18 px median and
0.57 px at worst. On the Pi 5, with the rendered room frames resampled into this layout and the field pose on at
30 fps:

| | 1080 mode | 360-degree mode |
|---|---|---|
| Decoding | 5.9 ms | 13.3 ms |
| Detection | 18.1 ms (more tags need straightening) | 16.5 ms |
| Tags per frame | 8.8 (the blind gaps) | 11.1 |
| Results out after a frame arrives | 24 ms | 30 ms |

## Hardware

- A Raspberry Pi 5 with 64-bit Raspberry Pi OS or Debian 12 or later. A Pi 4 should work, more slowly.
- An Insta360 X4 in **webcam mode** (from the USB menu on the camera), plugged **straight into the Pi**. Don't go
  through a hub or dock, because some docks power the camera but never pass data. In webcam mode it's an ordinary UVC
  camera, and `x4tags` finds it by its USB vendor ID. If it falls back to USB-drive mode, choose webcam mode again on
  the camera.
- A supply that can give 5 V at 3 A. The X4 heats up when it charges and streams at the same time.

The default is the X4's 2880x1440 360-degree output. Its other webcam size, 1920x1080, works too
(`[camera] mode = "dual1080"`). That picture is two views stacked, the back above the front, and each covers 76
degrees to either side and 24 degrees up and down at its middle. It's sharper and faster. It's also blind for 28
degrees on each side of the camera and to anything high or low, so a tag 1.4 m above the camera drops out of view
closer than about 3 m. Insta360 doesn't publish this mode's geometry. `x4tags/dual.py` holds a calibration of one
X4, good to about 0.3 px on measurements the fit didn't see. Another X4, or new firmware, may differ.

## Install on the Pi

```sh
git clone https://github.com/zwatx/napa-x4-pi && cd napa-x4-pi
deploy/install.sh            # venv, /etc/x4tags.toml, systemd service (asks for sudo for those)
sudo nano /etc/x4tags.toml   # set [mount], [tags], [field] and [output], then:
sudo systemctl restart x4tags && journalctl -u x4tags -f
```

`deploy/install.sh --no-service` builds `.venv` and nothing else. It doesn't need sudo if a C compiler and
python3-venv are already installed. Try it by hand first:

```sh
.venv/bin/x4tags --print                         # camera -> JSON lines
.venv/bin/x4tags --replay some.mjpg --print      # a recorded MJPEG, or a folder of images
```

## Before relying on it

Two things about the X4 aren't documented anywhere, and they decide how far to trust the output. Check both once,
with the X4 plugged into any Linux computer that has a screen and this package installed.

1. **Does the picture follow the camera body?** Insta360 says the X4 stitches and *stabilizes* its 360-degree webcam
   output in the camera, and webcam mode has no setting to turn that off. On one X4 (2026-11), the picture followed
   quick turns within about half a second. The last few degrees, 3-8 after a quick turn by hand, came back slowly,
   with a time constant of about 5 s. A camera at rest gave a still picture, drifting less than 0.002 degrees in 15 s.

   So for 10-20 s after the robot turns, anything that depends on the picture's heading is a few degrees off. That
   means bearings to tags, the robot heading in `field_pose`, and through the mount offset the robot position there
   too. The *camera's* position isn't affected, because the picture turns about the lens.

   Take the heading from the robot's gyro and combine it with `field_pose`'s camera position. The Java example under
   [What the robot sees](#what-the-robot-sees) does this, as does `x4tags.localize.robot_position_from_heading`. To
   aim at a tag, work from field positions rather than the raw bearing. Firmware may change this, so check your own
   camera. No tag is needed:

   ```sh
   .venv/bin/python -m x4tags.check_stabilization --tilt   # turn it and let go, then tilt it and let go, when told
   ```

   It reports *body-fixed*, *follows ... but creeps* (with how far and how slowly), *heading-locked* or
   *horizon-locked*. If the horizon is locked, leave `[mount]` pitch and roll at 0.

2. **How late are frames?** Run the check below, open the page it serves in a browser on the same computer, full
   screen, and point the X4 at the screen. The page flips a tag's ID, and the check times how long the change takes
   to arrive. That time includes the screen's own delay, typically 17-50 ms, so it's an upper bound. Put the median,
   less your screen's delay, in `[camera] latency_ms`. One X4 measured about 140 ms, which is the default. Close any
   browser or system pop-ups first, because one covering the tag hides it.

   ```sh
   .venv/bin/python -m x4tags.check_latency          # then open http://localhost:8765
   ```

## Settings

Everything lives in `/etc/x4tags.toml`, and `config.example.toml` lists every key. These are the ones to set:

- `[mount]` is where the camera sits on the robot and how it's turned, in WPILib conventions. Positive pitch tips the
  view down and positive yaw turns it left. At zero, the middle of the picture looks along the robot's +x.
- `[tags] size_m` is the side of the black square (0.1651 for FRC). Without it you get bearings only.
- `[field] layout` is a WPILib AprilTag field layout JSON, such as the season's file from WPILib, and turns on
  `field_pose`. Keep `min_tags` at 2 or more, because a single tag's pose has two nearly equally good fits, metres
  apart.
- `[field] corner_noise_px` is the corner error the covariances assume, 0.5 by default. See
  [Pose uncertainty](#pose-uncertainty).
- `[output] nt_server` is your team number, a host, or `localhost:5810` for a simulator.
- `[camera] mode` is `"equirect"` (360 degrees, the default) or `"dual1080"` (see [Hardware](#hardware)). Turning
  the camera 45 degrees on the robot puts the 1080 mode's blind gaps on the diagonals.
- `[camera] latency_ms` comes from `check_latency`, run with `--mode dual1080` for that mode. The defaults are one
  X4's. Timestamps are moved back by this much, and at 4 m/s every 100 ms not accounted for is 0.4 m of error.
- `[detector] min_tag_px` is 22 by default. 16 finds more distant tags and takes longer. `near_scale = 0` turns off
  the close-tag views, which saves about 5 ms per frame on a Pi 5.

## What the robot sees

NetworkTables 4, under the table `/x4tags` (rename it with `nt_table`):

| Topic | Type | |
|---|---|---|
| `tags` | double[] | 7 values per tag: id, yaw_deg, pitch_deg, distance_m, x_m, y_m, z_m |
| `field_pose` | double[] | with a field layout, 28 values (below); empty when there's no fit |
| `frame`, `age_ms`, `decode_ms`, `detect_ms` | int, double | per frame |
| `camera_connected` | boolean | true while frames arrive; `tags` and `field_pose` are emptied when it goes false |

Yaw (left of the robot's forward) and pitch (above horizontal) give the direction of each tag's centre from the
camera, in robot axes. `x_m, y_m, z_m` is the tag's centre in the robot frame. Distance and position are NaN without
`tags.size_m`, or when the tag's pose fits poorly. Bearings go through the picture's heading, so they lag a few
degrees after quick turns too. A frame's tags all share one array, so they can't get mixed up with another frame's,
and every value carries the time its frame was taken. In Java:

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

`field_pose` reads the same way. `x4tags.publish.FIELD_POSE_FIELDS` names its 28 values in order:

| Index | Value |
|---|---|
| 0-2 | robot x, y, z (m) |
| 3-5 | robot roll, pitch, yaw (deg) |
| 6-8 | camera x, y, z (m) |
| 9 | rms_px, how closely the corners fit |
| 10 | the number of tags used |
| 11 | xy_std_m, the camera position's standard deviation in its least certain direction |
| 12 | yaw_std_deg |
| 13-21 | the robot (x, y, yaw) covariance, 3x3 row by row, field axes, in m and rad |
| 22-25 | the camera (x, y) covariance, 2x2 row by row, field axes, in m |
| 26-27 | the camera position's standard deviations along the robot's forward and left axes (m) |

The X4's picture lags its body by a few degrees after quick turns (see
[Before relying on it](#before-relying-on-it)), so don't use its yaw as a heading measurement. Take the camera's x
and y (values 6 and 7) and work out the robot's position from them with the gyro's heading at `takenSeconds`:

```java
TimestampedDoubleArray pose = poseSub.getAtomic();          // poseSub: the field_pose topic, subscribed like tags
double[] v = pose.value;
double takenSeconds = pose.serverTime / 1e6;
if (v.length >= 28) {
    Rotation2d heading = poseEstimator.sampleAt(takenSeconds)   // the estimator's heading when the frame was taken
        .map(Pose2d::getRotation).orElse(poseEstimator.getEstimatedPosition().getRotation());
    Translation2d mount = new Translation2d(MOUNT_X_M, MOUNT_Y_M);   // the [mount] x_m, y_m
    Translation2d robot = new Translation2d(v[6], v[7]).minus(mount.rotateBy(heading));
    double floorM = 0.03;                                       // for what the covariance leaves out (below)
    poseEstimator.addVisionMeasurement(new Pose2d(robot, heading), takenSeconds,
        VecBuilder.fill(Math.max(v[26], floorM), Math.max(v[27], floorM), Double.POSITIVE_INFINITY));
}
```

Values 26-27 are along the robot's forward and left axes because WPILib's estimators apply vision standard deviations
along the robot's axes, not the field's. x4tags turns them into robot axes with the picture's heading, and a few
degrees of lag there changes them very little. WPILib takes one standard deviation per axis, so it can't use the
correlation between x and y. For a weak two-tag pose that leaves the per-axis figures up to 1.4 times too small along
the ellipse's long axis. Pass `xy_std_m` (value 11) for both axes if you'd rather err on the cautious side.

The robot x, y, z (values 0-2), its heading and the robot covariance (values 13-21) all go through the picture's
heading, so use them only if `check_stabilization` finds the picture body-fixed. With the camera 0.3 m from the
robot's centre, a 5-degree lag moves the robot position by 2.6 cm.

### Pose uncertainty

The covariances come from the fit's geometry and an assumed random error in each corner, `[field] corner_noise_px`
(0.5 px by default). They say how well the visible tags pin the pose down. Two small, distant tags in nearly the
same direction can fit well and still be weak, and the covariance says so: in an earlier test, such pairs came with an
`xy_std_m` of 8-20 cm. In 40 other poses of the rendered room, with 4-12 tags found and the default 0.5 px:

| Tags used | xy_std_m, median / 90th percentile |
|---|---|
| every tag found | 0.7 / 0.8 mm |
| the three largest | 1.6 / 3.0 mm |
| the two largest | 9 / 24 mm |

From two tags, a linearized covariance is too sure of some combinations of x, y and heading. x4tags caps its
correlations with NAPA's `cap_correlations`. In a Monte Carlo of the test room with Gaussian corner noise of the assumed
size, 3.5-5.5% of robot (x, y, yaw) errors then fell outside the 95% ellipse, with two tags or with all of them.
Without the cap, two tags left 11-28% outside. A test fails if the cap goes.

The covariances cover random corner error only. They leave out how far the real tags are from the layout, the
camera's stitch, timing errors and the picture's heading lag. Those don't shrink as more tags come into view, so a
robot won't see the millimetre figures above. Put a floor under the standard deviations on the robot, a few
centimetres to start with, and tune it on the field. The real X4's corners move far less than 0.5 px from frame to
frame: over 20 frames of a screen at half a metre, a corner's largest move from its average was 0.09 px median. The
0.5 px default is there to stand in for slower errors too.

UDP and stdout carry one JSON object per frame with all of this. Their `field_pose` has `robot_covariance`,
`camera_xy_covariance` and `gyro_std_devs_m` (values 26-27). For each tag they add its corners (pixels in the
stitched frame), its hamming distance, its decision margin and its full pose rotation. The rotation's columns are
the tag's axes towards its printed left edge, towards its printed bottom and out of its face, in robot axes.

## Layout

| Path | |
|---|---|
| `src/x4tags/capture.py` | `X4Webcam`: finds the X4 and captures the newest frame in grey, decoding in its own thread |
| `src/x4tags/equirect.py` | pixels to directions and back, flat views of the sphere, how much a tag is bent |
| `src/x4tags/dual.py` | the same for the 1920x1080 mode, from its calibration |
| `src/x4tags/detector.py` | `X4TagDetector`: NAPA, plus the seam, close-tag views, bent-tag refinement and single-tag poses |
| `src/x4tags/field.py`, `localize.py` | WPILib field layouts, and the field pose from all tags |
| `src/x4tags/mount.py`, `results.py` | camera to robot frame, and each frame's results |
| `src/x4tags/publish.py` | NetworkTables, UDP and stdout |
| `src/x4tags/app.py`, `config.py` | the service loop and its settings |
| `src/x4tags/check_latency.py`, `check_stabilization.py` | the camera checks |
| `deploy/` | the install script and systemd unit |
| `tests/` | rendered 360-degree frames and rooms of tags with exact truth; `pip install -e ".[test,nt]" && pytest` |

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

The camera frame has x to the right, y down and z forward, towards the middle of the picture. Corners come in NAPA's
order: the printed bottom-right first, then bottom-left, top-left and top-right. In WPILib's tag frame (x out of the
face, y to a viewer's right, z up) those are (0, s/2, -s/2), (0, -s/2, -s/2), (0, -s/2, s/2) and (0, s/2, s/2). A
tag across the seam keeps its quad whole, so a corner's column can be below 0 or past the width.

## Not done yet

- It hasn't run on a Pi with a live X4. The live tests so far (capture, timing, stabilization and latency) were on a
  laptop.
- Values sent before NetworkTables has synced its clock with the robot may carry the Pi's own time base. Check the
  timestamps on a real robot.
- Range is short (see [Measured](#measured)). Distant tags need a narrower camera.
- The X4 stitches its two lenses, about 3 cm apart, in the camera. A close tag on a stitch line can come out slightly
  misplaced, and the rendered tests don't model that.
- Single-tag poses don't report the IPPE ambiguity, so a face-on tag's rotation can be a few degrees off.
- The 1080 mode's calibration comes from one X4.

## License

GPL-3.0-only, like NAPA.
