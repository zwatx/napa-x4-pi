"""AprilTags in the X4's stitched 360-degree webcam frame, using NAPA as it is (public API only).

The frame is equirectangular, so a tag's straight sides come out curved, more so for big tags and tags away from the
horizon, and a tag straight behind the camera is cut in two by the left/right edge. This wraps a ``NapaDetector``:

1. NAPA runs on the frame as usual.
2. A strip made of the frame's right and left edges side by side is searched too, for tags cut by the seam.
3. Each tag whose sides the frame bends by more than ``refine_bow_px`` is looked at again through a small pinhole
   view centred on it (where its sides are straight again), and its corners are taken from there.

Results carry corners in the frame's pixels and the direction of each corner, which is what pose and bearing need.

``mode="dual1080"`` takes the X4's 1920x1080 mode instead (front and back views stacked; see ``x4tags.dual``): NAPA
runs on the frame, tags across the line between the halves are dropped, and bent tags are refined the same way.
"""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor, wait
from dataclasses import dataclass, field

import cv2
import numpy as np
from napa_vision import NapaDetector

from .dual import DualView
from .equirect import TangentView, angular_radius, pixel_to_ray, ray_to_pixel, side_bow_px, tangent_basis, unwrap_near


# The X4 is two cameras back to back: the front half of its picture (z > 0) is seen from its front lens's centre and
# the back half from its back lens's, this far ahead of and behind the middle of the body (28 mm apart; sideways
# offsets fitted at under 0.2 mm). Measured with tags on a screen across the stitch at 0.6 and 1 m: taking one centre
# for both halves left 11-13 px between them at 1 m and ~20 px at 0.6 m, about 4 px at 3 m.
LENS_OFFSET_M = 0.014


def lens_centre(rays) -> np.ndarray:
    """(..., 3) where in the camera frame the lens that sees each ray is (by the half of the picture it's in)."""
    rays = np.asarray(rays, float)
    out = np.zeros(rays.shape)
    out[..., 2] = np.where(rays[..., 2] >= 0, LENS_OFFSET_M, -LENS_OFFSET_M)
    return out


_PHI = (1 + 5 ** 0.5) / 2
_ICOSAHEDRON = ([(0, a, b * _PHI) for a in (1, -1) for b in (1, -1)]
                + [(a, b * _PHI, 0) for a in (1, -1) for b in (1, -1)]
                + [(a * _PHI, 0, b) for a in (1, -1) for b in (1, -1)])


@dataclass
class X4Tag:
    tag_id: int
    corners: np.ndarray  # (4, 2) frame pixels, NAPA's (OpenCV) corner order; u unwrapped to keep the quad whole
    rays: np.ndarray  # (4, 3) unit directions of the corners, camera frame (x right, y down, z forward)
    hamming: int | None
    decision_margin: float | None
    bow_px: float  # how far the frame bends this tag's sides, pixels
    refined: bool  # corners came from the straight-sided view
    seam: bool  # found in the seam strip
    near: bool = False  # found in the low-resolution straight views (close tags the frame bends too much)
    extra: dict = field(default_factory=dict)

    @property
    def center_ray(self) -> np.ndarray:
        """Direction of the tag's true centre: where its diagonals cross, in a flat view of the sphere around the tag
        (straight lines stay straight there, so this is exact for a flat tag; the mean of the corner rays is not, by
        up to a degree or so for close, turned tags)."""
        m = self.rays.sum(0)
        basis = tangent_basis(m)
        local = self.rays @ basis.T
        q = local[:, :2] / local[:, 2:3]
        d1, d2 = q[2] - q[0], q[3] - q[1]
        a = np.array([[d1[0], -d2[0]], [d1[1], -d2[1]]])
        if abs(np.linalg.det(a)) < 1e-12:
            return m / np.linalg.norm(m)
        s = np.linalg.solve(a, q[1] - q[0])[0]
        c = q[0] + s * d1
        r = np.array([c[0], c[1], 1.0]) @ basis
        return r / np.linalg.norm(r)

    def bearing_deg(self) -> tuple[float, float]:
        """(azimuth right of straight ahead, elevation above the horizon) of the tag's middle, degrees."""
        x, y, z = self.center_ray
        return float(np.degrees(np.arctan2(x, z))), float(np.degrees(np.arcsin(-y)))


class X4TagDetector:
    """``detect(gray)`` on a stitched X4 frame (2:1). Extra keyword arguments go to ``NapaDetector``. Use one per
    thread: it keeps state between frames, and in "equirect" mode runs the seam and near searches on a worker thread
    of its own with a second NapaDetector (so ``threads`` applies to each).

    ``mode``: "equirect" for the 360-degree 2880x1440 picture (or a reduced decode of it), "dual1080" for the
    1920x1080 front/back picture (or a reduced decode); the seam and near-view searches apply to "equirect" only.

    ``refine_bow_px``: refine tags whose sides the frame bends by more than this (None: never refine).
    ``seam_px``: width searched on each side of the left/right seam (0: don't search it); should exceed half the
    widest tag you need there.
    ``near_scale``: also search fixed straight (pinhole) views all around the camera at this fraction of the frame's
    resolution (0: don't). Close tags come out so bent in the 360-degree frame (sides bowed by 5-40 px) that NAPA
    can't fit them there; in a straight view their sides are straight again. Low resolution suffices because only
    big tags need it, and the full-resolution refinement then fixes their corners.
    ``near_per_frame``: how many of the 12 straight views to search per call (0: all). Searching all of them costs
    several ms, so a live loop searches a few in rotation plus every view that found a close tag on the previous
    call: a new close tag is found within 12 / near_per_frame frames, then kept every frame.
    """

    MODES = ("equirect", "dual1080")

    def __init__(self, refine_bow_px: float | None = 0.2, seam_px: int = 192, view_margin: float = 1.7,
                 max_view_px: int = 400, min_view_px: int = 256, near_scale: float = 1 / 3, near_per_frame: int = 2,
                 mode: str = "equirect", **napa_kwargs):
        if mode not in self.MODES:
            raise ValueError(f"mode {mode!r}: expected one of {', '.join(self.MODES)}")
        self.mode = mode
        self._dual: DualView | None = None
        self.napa = NapaDetector(**napa_kwargs)
        self._napa_kwargs = napa_kwargs
        self._side_napa: NapaDetector | None = None  # the seam and near searches' own detector, on a worker thread
        self._pool: ThreadPoolExecutor | None = None
        self.refine_bow_px, self.seam_px = refine_bow_px, int(seam_px)
        self.view_margin, self.max_view_px, self.min_view_px = view_margin, int(max_view_px), int(min_view_px)
        self.near_scale, self.near_per_frame = float(near_scale), int(near_per_frame)
        self._near_views = {}  # (width, height) -> [(TangentView, map1, map2)]
        self._near_next = 0  # next view in the rotation
        self._near_hits: set[int] = set()  # views that found a close tag on the previous call

    def projection(self, width: int, height: int) -> DualView | None:
        """The frame's geometry for "dual1080" (None for the equirect frame, handled by ``x4tags.equirect``)."""
        if self.mode != "dual1080":
            return None
        if self._dual is None or (self._dual.width, self._dual.height) != (width, height):
            self._dual = DualView(width, height)
        return self._dual

    def px_per_rad(self, width: int, height: int) -> float:
        """The frame's pixels per radian (at the middle of each half for "dual1080"), for errors in pixels."""
        proj = self.projection(width, height)
        return height / np.pi if proj is None else proj.px_per_rad

    def detect(self, gray: np.ndarray) -> list[X4Tag]:
        height, width = gray.shape[:2]
        if self.mode == "dual1080":
            return self._detect_dual(gray, width, height)
        if width != 2 * height:
            raise ValueError(f"expected a 2:1 equirectangular frame, got {width}x{height}")
        side = None
        if self.seam_px > 0 or self.near_scale > 0:
            # The seam and near searches don't need the full-frame tags until they are compared with them, so they run
            # on another core meanwhile, with their own NapaDetector (one per thread; its native call frees the GIL).
            if self._pool is None:
                self._side_napa = NapaDetector(**self._napa_kwargs)
                self._pool = ThreadPoolExecutor(1, thread_name_prefix="x4tags-side")
            side = self._pool.submit(self._side_search, gray, width, height)
        try:
            tags = [self._tag(o, width, height, seam=False) for o in self.napa.detect(gray)]
        except BaseException:
            if side is not None:  # don't leave it running into the next frame
                wait([side])
            raise
        if side is not None:
            seam, near = side.result()
            tags += [t for t in seam if not _seen(t, tags, 1.0)]
            tags += self._keep_near(near, tags)
        if self.refine_bow_px is not None:
            for t in tags:
                if t.bow_px > self.refine_bow_px:
                    self._refine(gray, t, width, height)
        return tags

    def _detect_dual(self, gray, width, height):
        proj = self.projection(width, height)  # raises unless 16:9
        tags = []
        for o in self.napa.detect(gray):
            c = np.asarray(o.corners, float).reshape(4, 2)
            front = proj.half_of(c[:, 1])
            if front.any() != front.all():
                continue  # across the line between the halves: neither view holds the whole tag
            tags.append(self._tag(o, width, height, seam=False, projection=proj))
        if self.refine_bow_px is not None:
            for t in tags:
                if t.bow_px > self.refine_bow_px:
                    self._refine(gray, t, width, height, proj)
        return tags

    # -- steps
    def _tag(self, obs, width, height, seam, corners=None, projection=None):
        if projection is not None:
            c = np.asarray(obs.corners if corners is None else corners, float).reshape(4, 2).copy()
            rays = projection.pixel_to_ray(c[:, 0], c[:, 1])
            return X4Tag(int(obs.tag_id), c, rays, obs.hamming, obs.decision_margin, _bow_px(c, rays, projection),
                         False, seam)
        c = np.asarray(obs.corners if corners is None else corners, float).reshape(4, 2).copy()
        c[:, 0] = unwrap_near(c[:, 0], c[0, 0], width)
        mid = np.mod(c[:, 0].mean() + 0.5, width) - 0.5  # keep the middle in [-0.5, W - 0.5)
        c[:, 0] += mid - c[:, 0].mean()
        rays = pixel_to_ray(c[:, 0], c[:, 1], width, height)
        return X4Tag(int(obs.tag_id), c, rays, obs.hamming, obs.decision_margin, side_bow_px(c, width, height),
                     False, seam)

    def _side_search(self, gray, width, height):
        """The seam strip's and the near views' tags, before they are compared with the full-frame search's."""
        seam = self._seam(gray, width, height) if self.seam_px > 0 else []
        near = self._near(gray, width, height) if self.near_scale > 0 else []
        return seam, near

    def _seam(self, gray, width, height):
        s = min(self.seam_px, width // 4)
        strip = np.ascontiguousarray(np.concatenate([gray[:, width - s:], gray[:, :s]], axis=1))
        out = []
        for o in self._side_napa.detect(strip):
            c = np.asarray(o.corners, float).reshape(4, 2)
            if not (c[:, 0].min() < s - 0.5 < c[:, 0].max()):
                continue  # wholly on one side: the full-frame search has it
            c = c.copy()
            c[:, 0] += width - s  # strip column x is frame column x + W - s (mod W)
            out.append(self._tag(o, width, height, seam=True, corners=c))
        return out

    # Straight views for close tags, centred on the 12 vertices of an icosahedron: no direction is more than 37.4
    # degrees from a view's centre, so with views 55 degrees in radius any tag whose corners lie within 17.6 degrees
    # of its centre (a face-on FRC tag 0.37 m away or farther) fits whole in at least one view. (A cube's 6 faces
    # leave only 5 degrees of margin towards its corners.)
    NEAR_DIRECTIONS = _ICOSAHEDRON
    NEAR_HALF_ANGLE = np.radians(55.0)

    def _views(self, width, height):
        views = self._near_views.get((width, height))
        if views is None:
            focal = height / np.pi * self.near_scale
            size = 2 * int(np.ceil(focal * np.tan(self.NEAR_HALF_ANGLE))) + 1
            views = []
            for d in self.NEAR_DIRECTIONS:
                view = TangentView(np.array(d), focal, size, width, height)
                g = np.arange(size, dtype=float)
                uv = ray_to_pixel(view.view_to_ray(np.stack(np.meshgrid(g, g), -1)), width, height)
                mx = (np.mod(uv[..., 0] + 0.5, width) - 0.5).astype(np.float32)
                m1, m2 = cv2.convertMaps(mx, uv[..., 1].astype(np.float32), cv2.CV_16SC2)  # fast fixed-point remap
                views.append((view, m1, m2))
            self._near_views[(width, height)] = views
        return views

    def _near(self, gray, width, height):
        """(view index, tag) for each tag found in this call's near views, in view order."""
        views = self._views(width, height)
        if self.near_per_frame <= 0 or self.near_per_frame >= len(views):
            chosen = list(range(len(views)))
        else:
            chosen = sorted(self._near_hits)
            for k in range(self.near_per_frame):
                i = (self._near_next + k) % len(views)
                if i not in chosen:
                    chosen.append(i)
            self._near_next = (self._near_next + self.near_per_frame) % len(views)
        out = []
        for i in chosen:
            view, m1, m2 = views[i]
            # The maps' columns are already wrapped into the frame, so the border only touches the half pixel at each
            # edge; replicating it is ~0.6 ms a view faster than BORDER_WRAP on a Pi 5, and these views only find tags
            # (bent ones are then refined in the full frame, which does wrap).
            patch = cv2.remap(gray, m1, m2, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
            for o in self._side_napa.detect(patch):
                rays = view.view_to_ray(np.asarray(o.corners, float).reshape(4, 2))
                rays /= np.linalg.norm(rays, axis=1, keepdims=True)
                t = self._tag(o, width, height, seam=False, corners=ray_to_pixel(rays, width, height))
                t.near = True
                out.append((i, t))
        return out

    def _keep_near(self, near, found):
        """The near views' tags that no other search (or earlier near view) found; remembers which views had one."""
        out, hits = [], set()
        for i, t in near:
            if _seen(t, found + out, 2.0):
                continue
            out.append(t)
            hits.add(i)
        self._near_hits = hits
        return out

    def _refine(self, gray, tag, width, height, projection=None):
        centre = tag.rays.sum(0)
        half_angle = min(np.radians(75.0), angular_radius(tag.rays, centre) * self.view_margin)
        lat = np.arcsin(np.clip(-centre[1] / np.linalg.norm(centre), -1, 1))
        if projection is None and abs(lat) + np.sqrt(2) * half_angle > np.radians(88.0):  # the view would reach over a pole
            tag.extra["refine"] = "too near a pole"
            return
        # The frame's own scale (pixels per radian of latitude), or less for big tags so the view stays under
        # max_view_px: a big tag keeps plenty of edge pixels at half scale and the view costs a quarter as much.
        scale = height / np.pi if projection is None else projection.local_px_per_rad(centre)
        focal = min(scale, (self.max_view_px - 1) / (2 * np.tan(half_angle)))
        size = 2 * int(np.ceil(focal * np.tan(half_angle))) + 1
        view = TangentView(centre, focal, size, width, height, projection)
        patch = view.render(gray)
        # NAPA misses some borderline tags in very small images that it finds in the full frame; padding the view
        # with its own edge pixels to a minimum size brings them back (and costs little).
        pad = max(0, (self.min_view_px - size + 1) // 2)
        if pad:
            patch = cv2.copyMakeBorder(patch, pad, pad, pad, pad, cv2.BORDER_REPLICATE)
        best, best_d = None, 0.25 * size
        tag.extra["view_scale"] = round(focal / scale, 3)
        for o in self.napa.detect(patch):
            if int(o.tag_id) != tag.tag_id:
                continue
            d = float(np.linalg.norm(np.asarray(o.corners, float).reshape(4, 2).mean(0) - pad - view.c))
            if d < best_d:
                best, best_d = o, d
        if best is None:
            tag.extra["refine"] = "not found in view"
            return
        pc = np.asarray(best.corners, float).reshape(4, 2) - pad
        rays = view.view_to_ray(pc)
        rays /= np.linalg.norm(rays, axis=1, keepdims=True)
        px = ray_to_pixel(rays, width, height) if projection is None else projection.ray_to_pixel(rays, half=view.half)
        # Keep the full-frame corner order: match the view's corners to the old ones by direction.
        order = [int(np.argmax(rays @ r)) for r in tag.rays]
        if sorted(order) != [0, 1, 2, 3]:
            tag.extra["refine"] = "corner order mismatch"
            return
        px, rays = px[order], rays[order]
        if projection is None:
            px[:, 0] = unwrap_near(px[:, 0], tag.corners[:, 0], width)
        tag.corners, tag.rays, tag.refined = px, rays, True
        tag.hamming, tag.decision_margin = best.hamming, best.decision_margin
        tag.extra["view_px"] = size

    # -- pose
    @staticmethod
    def pose(tag: X4Tag, tag_size_m: float, frame_height: int = 1440, px_per_rad: float | None = None):
        """Tag pose in the camera frame (x right, y down, z forward), from a pinhole view centred on the tag.

        Returns (R 3x3, t 3, worst corner reprojection error in pixels at the frame's scale) for the lower-error IPPE
        solution, or None. t is the tag's centre from the middle of the camera body (the pose is found from the centre
        of the lens that sees the tag, ``lens_centre``, and moved by it). The error is in pixels of ``px_per_rad`` (default: the equirect
        frame's, ``frame_height / pi``). R's columns are the tag's axes in the camera frame, with OpenCV's
        square corners for NAPA's corner order (corner 0 is the printed bottom-right of ``napa_vision.tags.tag_image``):
        x towards the printed left edge, y towards the printed bottom, z out of the printed face. A tag seen nearly
        face-on has two close IPPE solutions; the chosen one can be tilted a few degrees the wrong way.
        """
        basis = TangentView(tag.center_ray, 1.0, 1, 2, 1).basis  # rows: right, down, forward
        local = tag.rays @ basis.T
        q = local[:, :2] / local[:, 2:3]
        half = tag_size_m / 2
        obj = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], float)
        n, rvecs, tvecs, errs = cv2.solvePnPGeneric(obj, q, np.eye(3), None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        best = None
        for rv, tv in zip(rvecs, tvecs):
            if tv[2, 0] <= 0:
                continue
            proj, _ = cv2.projectPoints(obj, rv, tv, np.eye(3), None)
            err = float(np.linalg.norm(proj.reshape(-1, 2) - q, axis=1).max())
            if best is None or err < best[2]:
                R_local = cv2.Rodrigues(rv)[0]
                best = (basis.T @ R_local, basis.T @ tv.ravel(), err)
        if best is None:
            return None
        scale = frame_height / np.pi if px_per_rad is None else px_per_rad
        # normalized units -> pixels at the frame's scale
        return best[0], best[1] + lens_centre(tag.center_ray), best[2] * scale


def _seen(tag: X4Tag, found: list[X4Tag], within_deg: float) -> bool:
    """Whether ``found`` has a tag with the same ID within ``within_deg`` of this one's direction."""
    cos = np.cos(np.radians(within_deg))
    return any(f.tag_id == tag.tag_id and float(f.center_ray @ tag.center_ray) > cos for f in found)


def _bow_px(corners: np.ndarray, rays: np.ndarray, projection: DualView) -> float:
    """``side_bow_px`` for another geometry: the largest gap between each side's chord midpoint and where the true
    (great-circle) side's midpoint lands."""
    front = bool(projection.half_of(corners[:, 1]).all())
    mids = projection.ray_to_pixel(rays + np.roll(rays, -1, 0), half="front" if front else "back")
    chord = (corners + np.roll(corners, -1, 0)) / 2
    return float(np.linalg.norm(mids - chord, axis=1).max())
