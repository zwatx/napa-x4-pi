# Measurements

What the numbers in the README rest on. Each section says what was measured, on what, and with which settings.
"Rendered" frames are synthetic 360-degree frames with exact truth; everything else is a real Insta360 X4 (one
camera, firmware as of November 2026). Unless a section says otherwise, the Pi is a Raspberry Pi 5 (Debian 13,
Python 3.13, NAPA 0.6.1) with one NAPA thread.

## Corners and tags found (rendered frames)

64 rendered 360-degree frames, 481 tags from 10 to 360 px, with real HDR backgrounds, sensor noise, motion blur,
defocus, glare and shadows. Worst-corner error is the largest of a tag's four corner errors, in pixels.

With the current defaults (`min_tag_px` 16, `camera_gamma` 2.0, every close-tag view searched):

| | tags found | worst-corner error, median / 95th percentile |
|---|---|---|
| x4tags | 261 | 0.19 / 1.09 px |
| the same without undoing the gamma | 259 | 0.30 / 1.77 px |

Earlier, at NAPA's own defaults (`min_tag_px` 22, no gamma), against NAPA alone:

| | NAPA alone | x4tags |
|---|---|---|
| Tags found | 179 | 208 (the extra 29 are close, strongly bent tags) |
| Worst-corner error, median / 95th percentile | 0.50 / 5.5 px | 0.26 / 1.9 px |
| The same, for tags both find that are bent by over 1.5 px | 1.60 / 9.5 px | 0.23 / 1.7 px |
| Tags found when each tag in turn is moved onto the seam (481 tries) | 0 | 200 |

What's left of the error comes mostly from heavy motion blur (6-13 px of it: 67% of those tags are over 1 px off),
tags tilted more than 65 degrees and shadows across tags.

**Range.** The 360 mode has 458 px per radian (0.125 degrees per pixel). A face-on FRC tag (0.1651 m) is 22 px wide
at 3.4 m (NAPA's own minimum) and 16 px at 4.7 m (this service's). On the rendered frames NAPA found 32% of tags at
22-26 px, 54% at 40-56 px and 74% at 80-120 px. In phone screenshots of the X4 on an FRC practice field, far tags
were 16-23 px; at 22 px one shot showed 1 of 9 tags, at 16 px all 9, with no false IDs.

## Field pose (rendered)

A rendered 6 m x 4 m room with 12 tags (7-11 in view) and 40 robot poses with blur and noise: the camera position was
off by 0.2 mm median and 0.3 mm at the 95th percentile; the biggest single tag alone gave 5.6 mm and 4.3 cm. That's
with perfect geometry and exactly placed tags.

The covariance's `xy_std_m` in 40 poses of that room (4-12 tags found, `corner_noise_px` 0.5):

| Tags used | xy_std_m, median / 90th percentile |
|---|---|
| every tag found | 0.7 / 0.8 mm |
| the three largest | 1.6 / 3.0 mm |
| the two largest | 9 / 24 mm |

In a Monte Carlo of that room with Gaussian corner noise of the assumed size, 3.5-5.5% of robot (x, y, yaw) errors
fell outside the 95% ellipse with the correlation cap, with two tags or with all of them. Without the cap, two tags
left 11-28% outside. On the real X4, a corner's largest move from its average over 20 frames of a screen at half a
metre was 0.09 px median: the 0.5 px default stands in for slower errors too.

## Tags at known places (live X4)

Tags shown on a 27-inch, 120 Hz monitor at known positions (from the panel's pixel pitch: 0.233 mm per CSS pixel),
recorded from the Pi.

**Facing a lens** (eight 75 mm tags, 0.5-0.8 m away, up to 28 degrees above the horizon). The field pose fitted all
seven tags it saw (one wasn't found) to within 0.25 px (0.13-0.35 px per tag): no error beyond the corners' own. Its
camera position wandered by under 0.1 mm. Without refining bent tags: 0.37 px. Undoing a gamma of 2.0: 0.22 px.
Spacings from single tags' own poses: -1.5 to +3.9 mm off over 150-478 mm.

**Near the stitch** (the X4 turned so the tags sat 93-115 degrees to its side, 1 m away). Tags more than 10 degrees
from the stitch fitted to 0.13-0.24 px; the two 3.5 degrees from it were shifted about 3 px and bent by about 1.7 px.
With one lens centre for both halves, those two tags made the field pose fail outright.

**Across the stitch** (21 tags of 56 mm, the stitch through the middle of the monitor, at 1 m and 0.6 m, both stitch
lines). Against a pose fitted to the tags on one side, the tags on the other were 11-13 px off at 1 m (about 20 px
at 0.6 m), constant beyond a blending band from about 5 degrees before the stitch to 3 after. Fitting the lens
centres (on the forward axis, sideways offsets about 0.2 mm):

| | one centre | two centres, 14 mm either side of the body's middle |
|---|---|---|
| 1 m, fit residual | 1.57 px | 0.38 px |
| 0.6 m, fit residual | 5.8 px | 0.54 px |
| 0.6 m, the other stitch | 5.9 px | 0.51 px |

With the two-centre model and the stitch weighting, the service's field pose used every tag in view: all 31 frames
at 1 m near the stitch (1.3 mm from the pose of the far tags alone), 20 of 20 across it at 1 m (19 tags, 0.39 px),
and 18 of 20 at 0.6 m (17 tags, 0.57 px).

**On a practice field** (a phone screenshot of the 1080 mode): three tags on a board, centres marked 17 1/4 in apart,
seen from 1.3 m, came out 17.45 and 17.25 in apart.

## Latency and timing (live X4)

**Latency, from a moving tag.** On the 120 Hz monitor a tag slid back and forth at a known speed, timed by the
browser's clock synced to the Pi's over Wi-Fi (to within 7-9 ms). The fitted latency, against the service's frame
times, was 120 and 121 ms in the 360 mode (the tag at two heights) and 98 ms in the 1080 mode. Each includes the
monitor's own delay: one or two refreshes, the scan down to the tag and the pixels' response, about 20 ms (23 ms for
the 1080 run's lower tag). So the X4's own latency is about 100 and 75 ms, give or take 8: the defaults.

Timing a tag that flips instead read 144 ms on this monitor, and 154-163 ms (360 mode) and 133 ms (1080 mode) on a
60 Hz laptop screen earlier: a flip lands anywhere in the camera's frame interval and shows only from the next frame,
so that way reads about half a frame (17 ms) plus half the exposure too long.

**Frame times.** The X4 takes frames on a steady 29.97 fps clock; its frames reach the Pi late in a repeating pattern
of three: in the 360 mode about 16, 9 and 0 ms plus noise, in the 1080 mode about 26, 3 and 0 ms with some frames
more than a frame period late. Timed by arrival, frame times jitter by about 6 ms (standard deviation) in the 360 mode
and 13 ms in the 1080 mode. The frame clock leaves 0.2-0.5 ms (1.7 ms at worst) on three recorded minutes in the
360 mode. Against the moving tag, single frames' capture times scattered 1.3-1.7 ms around the fit with the frame
clock, 7.5 ms with V4L2's stamps as they come; much of what's left is the monitor's 8 ms refresh steps.

**Exposure.** The moving tag's edges widened by 17.5 px per 1000 px/s of motion across the picture: the X4 exposed
for about 23 ms in that dim room. The webcam stream has no exposure setting, and the one UVC control it offers
(brightness) changes nothing.

## Speed (Pi 5)

**Live X4, 360 mode** (`min_tag_px` 16, four printed tags around a room 1-4 m away, field pose on): 28-29 fps,
decoding 9.5-10 ms on the capture thread, detection plus the field pose 17-19 ms, about 24% of the four cores, 50 C,
no throttling. Results went out about 30 ms after a frame arrived. Over 4,200 frames of the room with no tags,
nothing was reported. The process uses about 100 MB.

**Live X4, 1080 mode**: 29.7 fps, decoding 6.5 ms, detection plus the field pose 10.9 ms, 13% of the cores.

**Where the time goes** (360 mode, a recorded clip, one core each): NAPA on the frame 3.0 ms (5.8 at `min_tag_px`
16), the seam strip 0.9 ms and two close-tag views 2.8 ms on a second core alongside it, refining a bent tag about
1.6 ms, the field pose 1.7 ms from four tags. Running the seam and close-tag searches beside the full-frame search
took detection from 12.3 to 8.4 ms (15.4 to 9.6 at 16 px), with identical results.

**Rendered frames replayed** as quality-90 JPEGs at 30 fps (an earlier version, `min_tag_px` 22): 30.0 fps, decoding
17.8 ms and detection 14.7 ms. In the rendered room with the field pose, both modes at 30 fps:

| | 1080 mode | 360 mode |
|---|---|---|
| Decoding | 5.9 ms | 13.3 ms |
| Detection | 18.1 ms (more tags need straightening) | 16.5 ms |
| Tags per frame | 8.8 (the blind gaps) | 11.1 |

## Heading lag (live X4, laptop)

Turned quickly by hand, the X4's picture followed within about half a second, then the last 3-8 degrees came back
with a time constant of about 5 s. At rest it drifted less than 0.002 degrees in 15 s. Tilted, the picture drifted
about 0.35 px/s for a long time. Insta360 says webcam output is stabilized in the camera, with no setting to turn it
off.
