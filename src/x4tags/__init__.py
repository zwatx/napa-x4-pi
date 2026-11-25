"""AprilTags from an Insta360 X4's 360-degree webcam stream on a Raspberry Pi, with NAPA.

NAPA (napa-vision) does the detecting, unchanged; this package adds what the X4's picture needs (grey MJPEG capture
with steady frame times, the seam, bent and close tags, its two lens centres) and the robot plumbing (robot-frame
results, a field pose from every tag in view, NetworkTables). See docs/HOW-IT-WORKS.md.
"""

from importlib.metadata import PackageNotFoundError, version

from .capture import FrameClock, X4Webcam, find_x4_device
from .detector import X4Tag, X4TagDetector
from .dual import DualView
from .equirect import pixel_to_ray, ray_to_pixel
from .field import FieldLayout
from .localize import FieldLocalizer, FieldPose
from .mount import Mount

__all__ = ["DualView", "FieldLayout", "FieldLocalizer", "FieldPose", "FrameClock", "Mount", "X4Tag",
           "X4TagDetector", "X4Webcam", "find_x4_device", "pixel_to_ray", "ray_to_pixel"]
try:
    __version__ = version("napa-x4-pi")
except PackageNotFoundError:  # running from a source tree that isn't installed
    __version__ = "0+unknown"
