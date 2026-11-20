"""AprilTags from an Insta360 X4's 360-degree webcam stream on a Raspberry Pi, with NAPA.

NAPA (napa-vision) does the detecting, unchanged; this package adds what the X4's stitched equirectangular frame
needs (grey MJPEG capture, the left/right seam, corners of tags whose sides the frame bends) and robot plumbing.
"""

from .capture import X4Webcam, find_x4_device
from .detector import X4Tag, X4TagDetector
from .equirect import pixel_to_ray, ray_to_pixel
from .mount import Mount

__all__ = ["X4Webcam", "find_x4_device", "X4Tag", "X4TagDetector", "Mount", "pixel_to_ray", "ray_to_pixel"]
__version__ = "0.1.0"
