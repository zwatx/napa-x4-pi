# napa-x4-pi

AprilTags from an Insta360 X4's 360-degree webcam stream on a Raspberry Pi, published to the robot.
A starting point for a coprocessor: one camera that sees all the way around, one Pi, results on NetworkTables.

Detection is [NAPA](https://pypi.org/project/napa-vision/) (`napa-vision`), used as it is. This repository adds
what the X4's picture needs and the robot plumbing:

- **Grey capture.** The X4 in webcam mode sends MJPEG. `X4Webcam` takes the JPEG undecoded and decodes it straight
  to grey, instead of OpenCV's default colour decode and a grey conversion afterwards: 18 ms instead of 34 ms per
  2880x1440 frame on a Pi 5. A background thread keeps only the newest frame, so the loop never falls behind.
- **The seam.** The stitched picture is equirectangular: its left and right edges are the same direction, behind the
  camera. A tag there is cut in two and NAPA can't see it; a strip of both edges side by side is searched too.
- **Bent tags.** The 360-degree picture bends straight lines, more for big tags and away from the horizon. Tags bent
  by more than 0.2 px are looked at again through a small flat (pinhole) view centred on the tag, where their sides
  are straight again, and the corners are mapped back.
- **Robot frame.** Each tag's yaw and pitch from the robot, and with the tag size its distance and pose, in WPILib's
  robot frame (x forward, y left, z up) through the camera's mount position.
- **Outputs.** NetworkTables 4, JSON over UDP, or JSON lines on stdout. NT values carry the frame's capture time as
  their timestamp.
- **Service.** A systemd unit that starts at boot, restarts on failure and reconnects when the camera drops out.

## Measured

On rendered 360-degree frames with exact corners (64 frames, 481 tags, real HDR backgrounds, sensor noise, blur):

| | NAPA alone | x4tags |
|---|---|---|
| Worst-corner error, median / 95th percentile | 0.50 / 5.5 px | 0.25 / 1.5 px |
| Same, on the most bent tags (bow over 1.5 px) | 1.60 / 9.5 px | 0.23 / 1.7 px |
| Tags found when placed across the seam | 0 of 481 | 170 of 481 (179 elsewhere) |

Raspberry Pi 5, the service's loop on those frames as quality-90 JPEGs paced at 30 fps, one detector thread:
29.3 fps, decode 17.9 ms, detection 9.1 ms (95th percentile 15.8), frame age when results are published 28 ms
(95th percentile 37), 52 C, no throttling. More detector threads barely help (`threads = 4`: 27 ms); the JPEG
decode is the largest part. Camera and USB latency come on top and haven't been measured yet.

## Hardware

- Raspberry Pi 5 (a Pi 4 should work, slower), 64-bit Raspberry Pi OS or Debian 12+.
- Insta360 X4 set to **webcam mode** (USB menu on the camera), plugged **straight into the Pi**, not through a hub
  or dock: some docks power it but never pass data. It then shows up as a UVC camera; `x4tags` finds it by its USB
  vendor ID. It may fall back to USB-drive mode; choose webcam mode again on the camera.
- The X4 sends 2880x1440 (stitched equirectangular) at 30 fps. Feed it from a supply that can give 5 V 3 A;
  it heats up when charging and streaming at once.

## Install on the Pi

```sh
git clone <this repository> napa-x4-pi && cd napa-x4-pi
deploy/install.sh            # venv, /etc/x4tags.toml, systemd service (asks for sudo for those)
sudo nano /etc/x4tags.toml   # set [mount] and [output], then:
sudo systemctl restart x4tags && journalctl -u x4tags -f
```

`deploy/install.sh --no-service` only builds `.venv` (no sudo if a C compiler and python3-venv are installed).
Try it by hand first:

```sh
.venv/bin/x4tags --print                         # camera -> JSON lines
.venv/bin/x4tags --replay some.mjpg --print      # recorded MJPEG, or a folder of images
```

## Settings

All in `/etc/x4tags.toml`; `config.example.toml` lists every key. The ones to set:

- `[mount]`: where the camera is on the robot and how it's turned (WPILib conventions: + pitch tips the view down,
  + yaw turns it left; zero means the middle of the picture looks along the robot's +x).
- `[tags] size_m`: black-square side (FRC: 0.1651). Without it you get bearings only.
- `[output] nt_server`: your team number (or a host, or `localhost:5810` for a simulator).
- `[detector] min_tag_px`: 22 by default; 16 finds more distant tags for more time.

## What the robot sees

NetworkTables 4, table `/x4tags` (rename with `nt_table`). Per-tag arrays share one order:

| Topic | Type | |
|---|---|---|
| `ids` | int[] | tag IDs |
| `yaw_deg`, `pitch_deg` | double[] | direction of each tag's centre from the camera, in robot axes |
| `distance_m` | double[] | camera to tag centre (NaN without `size_m` or with a poor pose) |
| `x_m`, `y_m`, `z_m` | double[] | tag centre in the robot frame |
| `frame`, `age_ms`, `decode_ms`, `detect_ms` | int, double | per frame |
| `camera_connected` | boolean | false while the camera is missing |

Every value of a frame is stamped with the time the frame arrived from the camera, so use the timestamp for latency
compensation, for example in Java:

```java
var ids = NetworkTableInstance.getDefault().getTable("x4tags").getIntegerArrayTopic("ids").subscribe(new long[0]);
TimestampedIntegerArray latest = ids.getAtomic();   // latest.serverTime: capture time, microseconds
```

UDP and stdout carry one JSON object per frame with everything above plus each tag's corners (pixels in the stitched
frame), full pose rotation, hamming distance and decision margin.

## Layout

| Path | |
|---|---|
| `src/x4tags/capture.py` | `X4Webcam`: find the X4, newest-frame grey capture |
| `src/x4tags/equirect.py` | pixel <-> direction, flat views of the sphere, how much a tag is bent |
| `src/x4tags/detector.py` | `X4TagDetector`: NAPA + seam + bent-tag refinement + pose |
| `src/x4tags/mount.py`, `results.py` | camera -> robot frame, per-frame results |
| `src/x4tags/publish.py` | NetworkTables, UDP, stdout |
| `src/x4tags/app.py`, `config.py` | the service loop and its settings |
| `deploy/` | install script and systemd unit |
| `tests/` | renders its own 360-degree frames with exact corners; `pip install -e ".[test,nt]" && pytest` |

## Using the pieces directly

```python
from x4tags import X4Webcam, X4TagDetector

det = X4TagDetector(min_tag_px=22)                # other NapaDetector options pass through
with X4Webcam() as cam:
    t, gray = cam.read()                          # newest frame, grey
    for tag in det.detect(gray):
        print(tag.tag_id, tag.bearing_deg(), tag.refined, tag.seam)
        pose = det.pose(tag, 0.1651)              # (R, t, error px) in the camera frame, or None
```

Camera frame: x right, y down, z forward (the middle of the picture). Corners are in NAPA's (OpenCV's) order;
a tag across the seam keeps its quad whole, so one corner's column can be below 0 or past the width.

## Not done yet

- Not yet run against a live X4 on a Pi: capture, frame age and the X4's UVC controls (exposure) need a check.
- Single-tag poses only; combining tags into a field position (with an AprilTag field layout) is the robot's job here.
- The stitch is from two lenses about 3 cm apart; near the stitch line, close tags can be slightly misplaced.

## License

GPL-3.0-only, like NAPA.
