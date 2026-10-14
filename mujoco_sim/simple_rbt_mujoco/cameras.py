"""RGB/metric-depth cameras, independent of ROS.

Every pose uses optical axes: +X right, +Y down, +Z forward. A perspective
image stores distance along +Z (``depth_kind='z'``); an equirectangular image
stores distance from the optical centre (``depth_kind='range'``). Distances
are metres, and pixels without a surface before the far clip are infinity.
MuJoCo/OpenGL is loaded only when a camera first renders.
"""
from __future__ import annotations

from dataclasses import dataclass
from threading import RLock
from typing import Literal, Mapping

import numpy as np


@dataclass(frozen=True)
class CameraFrame:
    rgb: np.ndarray
    depth: np.ndarray
    depth_kind: Literal["z", "range"]


# Columns are each face's right, down, forward axes in the panorama frame.
CUBE_ROTATIONS = {
    "front": np.eye(3),
    "right": np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]]),
    "back": np.diag([-1., 1., -1.]),
    "left": np.array([[0., 0., -1.], [0., 1., 0.], [1., 0., 0.]]),
    "top": np.array([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]]),
    "bottom": np.array([[1., 0., 0.], [0., 0., 1.], [0., -1., 0.]]),
}


def _image_size(width: int, height: int) -> tuple[int, int]:
    if any(isinstance(v, bool) or not isinstance(v, (int, np.integer)) or v <= 0
           for v in (width, height)):
        raise ValueError("Image width and height must be positive integers")
    return int(width), int(height)


def _focal_length(height: int, fov_y_deg: float) -> float:
    if not np.isfinite(fov_y_deg) or not 0. < fov_y_deg < 180.:
        raise ValueError("Vertical field of view must be between 0 and 180 degrees")
    return height / (2. * np.tan(np.deg2rad(fov_y_deg) / 2.))


def perspective_rays(width: int, height: int, fov_y_deg: float) -> np.ndarray:
    """Pixel-centre optical rays [x, y, 1], *not* unit vectors.

    Multiplying by metric Z depth produces optical-frame XYZ coordinates.
    Square pixels are assumed, so horizontal FOV follows the aspect ratio.
    """
    width, height = _image_size(width, height)
    focal = _focal_length(height, fov_y_deg)
    x, y = np.meshgrid((np.arange(width) + .5 - width / 2.) / focal,
                       (np.arange(height) + .5 - height / 2.) / focal)
    return np.stack((x, y, np.ones_like(x)), axis=-1)


def equirectangular_rays(width: int, height: int) -> np.ndarray:
    """Unit optical rays for a full 360 x 180 degree panorama.

    The image centre looks +Z; longitude increases toward +X. The top looks
    -Y and the left/right seam looks -Z. All samples are pixel-centred.
    """
    width, height = _image_size(width, height)
    longitude = ((np.arange(width) + .5) / width - .5) * (2. * np.pi)
    latitude = (.5 - (np.arange(height) + .5) / height) * np.pi
    longitude, latitude = np.meshgrid(longitude, latitude)
    cos_latitude = np.cos(latitude)
    return np.stack((np.sin(longitude) * cos_latitude, -np.sin(latitude),
                     np.cos(longitude) * cos_latitude), axis=-1)


def _cubemap_lookup(width: int, height: int, face_size: int):
    """Cache flat gather/scatter indices and the original face-pixel factors."""
    rays = equirectangular_rays(width, height)
    # Stream argmax instead of retaining six score images and their stack.
    # Strict comparison preserves np.argmax's first-face choice at exact ties.
    rotations = iter(CUBE_ROTATIONS.values())
    best_scores = rays @ next(rotations)[:, 2]
    face_ids = np.zeros((height, width), dtype=np.uint8)
    for face_id, rotation in enumerate(rotations, start=1):
        scores = rays @ rotation[:, 2]
        better = scores > best_scores
        np.copyto(best_scores, scores, where=better)
        face_ids[better] = face_id
    del best_scores, scores, better
    lookup = []
    factors = np.linalg.norm(perspective_rays(face_size, face_size, 90.), axis=-1).ravel()
    index_dtype = np.int32 if max(width * height, face_size ** 2) <= np.iinfo(np.int32).max else np.intp
    for face_id, rotation in enumerate(CUBE_ROTATIONS.values()):
        mask = face_ids == face_id
        local = rays[mask] @ rotation
        uv = local[:, :2] / local[:, 2, None]
        pixels = np.floor((uv + 1.) * face_size / 2.).astype(int)
        pixels = np.clip(pixels, 0, face_size - 1)
        destination = np.flatnonzero(mask).astype(index_dtype, copy=False)
        source = (pixels[:, 1] * face_size + pixels[:, 0]).astype(index_dtype, copy=False)
        lookup.append((destination, source, factors[source]))
    return lookup


def _stitch_face(rgb, depth, face_rgb, face_depth, lookup):
    destination, source, factor = lookup
    rgb.reshape(-1, 3)[destination] = face_rgb.reshape(-1, 3)[source]
    # Keep the previous float32 depth * float64 factor operation before the
    # float32 output assignment, including infinity and nearest-pixel semantics.
    depth.ravel()[destination] = face_depth.ravel()[source] * factor


def _stitch(faces_rgb, faces_depth_z, width, height, lookup):
    rgb = np.empty((height, width, 3), dtype=np.uint8)
    depth = np.empty((height, width), dtype=np.float32)
    for name, indices in zip(CUBE_ROTATIONS, lookup):
        _stitch_face(rgb, depth, faces_rgb[name], faces_depth_z[name], indices)
    return CameraFrame(rgb=rgb, depth=depth, depth_kind="range")


def stitch_cubemap(faces_rgb: Mapping[str, np.ndarray],
                   faces_depth_z: Mapping[str, np.ndarray],
                   width: int, height: int) -> CameraFrame:
    """Stitch six square 90-degree views using nearest-neighbour sampling.

    Face names match ``CUBE_ROTATIONS``. Input depth is metric face Z, which
    is converted to radial range using the sampled face pixel's actual ray.
    Nearest sampling preserves silhouettes; angular resolution is limited
    by ``face_size``, regardless of the requested panorama output size.
    """
    width, height = _image_size(width, height)
    required = set(CUBE_ROTATIONS)
    if set(faces_rgb) != required or set(faces_depth_z) != required:
        raise ValueError("Cubemap requires front/right/back/left/top/bottom faces")
    sample = np.asarray(faces_depth_z["front"])
    if sample.ndim != 2 or sample.shape[0] != sample.shape[1] or not sample.shape[0]:
        raise ValueError("Cubemap depth faces must be non-empty square arrays")
    face_size = sample.shape[0]
    for name in CUBE_ROTATIONS:
        if np.shape(faces_rgb[name]) != (face_size, face_size, 3):
            raise ValueError(f"Invalid RGB face shape: {name}")
        if np.shape(faces_depth_z[name]) != (face_size, face_size):
            raise ValueError(f"Invalid depth face shape: {name}")
    lookup = _cubemap_lookup(width, height, face_size)
    return _stitch(faces_rgb, faces_depth_z, width, height, lookup)


def _pose(position, rotation):
    position = np.asarray(position, dtype=float)
    rotation = np.eye(3) if rotation is None else np.asarray(rotation, dtype=float)
    if position.shape != (3,) or not np.all(np.isfinite(position)):
        raise ValueError("Camera position must contain three finite coordinates")
    if (rotation.shape != (3, 3) or not np.all(np.isfinite(rotation))
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0)
            or not np.isclose(np.linalg.det(rotation), 1., atol=1e-6, rtol=0)):
        raise ValueError("rotation_world_from_optical must be a proper 3x3 rotation")
    return position, rotation


# MjrContext captures offscreen dimensions at construction. Temporarily using
# the requested viewport avoids allocating the model's larger default buffer.
# Serialize our initializers; callers must not initialize external renderers on
# this same model concurrently. Physics and every other visual setting stay as-is.
_RENDERER_INIT_LOCK = RLock()


def _make_renderer(model, width, height):
    import mujoco
    with _RENDERER_INIT_LOCK:
        visual = model.vis.global_
        previous = (visual.offwidth, visual.offheight)
        try:
            visual.offwidth, visual.offheight = width, height
            return mujoco.Renderer(model, height=height, width=width)
        finally:
            visual.offwidth, visual.offheight = previous


class _DepthBackgroundReader:
    """Read IDs left by classic Renderer's depth pass, with a public fallback.

    MuJoCo's classic Renderer enables SEGMENT+IDCOLOR for both depth and
    segmentation. Its depth render leaves that RGB framebuffer intact. This
    adapter contains the private _rect/_mjr_context/_gl_context access; unfamiliar
    renderer implementations continue through their public segmentation API.
    """
    def __init__(self, renderer, width, height):
        classic = any(cls.__module__ == "mujoco.rendering.classic.renderer"
                      for cls in type(renderer).__mro__)
        self.supported = classic and all(hasattr(renderer, attr) for attr in
                                        ("_rect", "_mjr_context", "_gl_context"))
        self.rgb = np.empty((height, width, 3), np.uint8) if self.supported else None
        self.ids = np.empty((height, width), np.uint32) if self.supported else None
        self.bits = np.empty((height, width), np.uint32) if self.supported else None
        self.no_surface_by_id = None

    def update_scene(self, renderer):
        if not self.supported:
            return
        # Preserve the public segmentation result's objid==-1 rule, including
        # nonzero segment IDs assigned to decorations with no model object ID.
        scene = renderer.scene
        mapping = np.ones(scene.ngeom + 1, dtype=bool)
        for geom in scene.geoms[:scene.ngeom]:
            if geom.segid != -1:
                mapping[geom.segid + 1] = geom.objid == -1
        self.no_surface_by_id = mapping

    def read(self, renderer):
        if self.supported:
            import mujoco
            try:
                mujoco.mjr_readPixels(self.rgb, None, renderer._rect, renderer._mjr_context)
            except (AttributeError, TypeError, RuntimeError):
                # Unsupported private/backend contract: use the established
                # public path now and for later views from this renderer.
                self.supported = False
            else:
                np.copyto(self.ids, self.rgb[..., 0], casting="unsafe")
                np.multiply(self.rgb[..., 1], 256, out=self.bits, dtype=np.uint32)
                np.add(self.ids, self.bits, out=self.ids)
                np.multiply(self.rgb[..., 2], 65536, out=self.bits, dtype=np.uint32)
                np.add(self.ids, self.bits, out=self.ids)
                mask = self.no_surface_by_id[self.ids]
                return mask[::-1] if renderer._gl_context is not None else mask
        renderer.enable_segmentation_rendering()
        try:
            return renderer.render()[..., 0] == -1
        finally:
            renderer.disable_segmentation_rendering()


class PerspectiveCamera:
    """Offscreen RGB and Z-depth camera with a caller-supplied world pose.

    Select an OpenGL backend before importing MuJoCo, e.g. MUJOCO_GL=egl.
    The caller must update kinematics (mj_forward/mj_step) before rendering.
    Camera objects own graphics contexts; use ``close`` or a context manager.
    """
    def __init__(self, model, width: int = 320, height: int = 240,
                 fov_y_deg: float = 60.):
        self.width, self.height = _image_size(width, height)
        self.fov_y_deg = float(fov_y_deg)
        self.focal_length = _focal_length(self.height, self.fov_y_deg)
        self.model = model
        self._renderer = None
        self._background_reader = None
        self._closed = False

    @property
    def intrinsics(self) -> np.ndarray:
        """K for integer pixel coordinates: principal point ((W-1)/2,(H-1)/2)."""
        return np.array([[self.focal_length, 0., (self.width - 1) / 2.],
                         [0., self.focal_length, (self.height - 1) / 2.],
                         [0., 0., 1.]])

    def _update_scene(self, data):
        if self._closed:
            raise RuntimeError("Cannot render with a closed camera")
        if self._renderer is None:
            self._renderer = _make_renderer(self.model, self.width, self.height)
            self._background_reader = _DepthBackgroundReader(self._renderer, self.width, self.height)
        self._renderer.update_scene(data)
        self._background_reader.update_scene(self._renderer)

    def render(self, data, position, rotation_world_from_optical=None) -> CameraFrame:
        if self._closed:
            raise RuntimeError("Cannot render with a closed camera")
        position, rotation = _pose(position, rotation_world_from_optical)
        self._update_scene(data)
        return self._render_view(position, rotation)

    def _render_view(self, position, rotation):
        """Render one pose of the already updated scene; owned arrays escape."""
        renderer = self._renderer
        near = float(self.model.vis.map.znear * self.model.stat.extent)
        far = float(self.model.vis.map.zfar * self.model.stat.extent)
        half_height = near * np.tan(np.deg2rad(self.fov_y_deg) / 2.)
        for camera in renderer.scene.camera:
            camera.pos[:] = position
            camera.forward[:] = rotation[:, 2]
            camera.up[:] = -rotation[:, 1]
            camera.orthographic = 0
            camera.frustum_near = near
            camera.frustum_far = far
            camera.frustum_center = 0.
            camera.frustum_bottom = -half_height
            camera.frustum_top = half_height
            # Zero requests horizontal bounds from the viewport aspect ratio.
            camera.frustum_width = 0.
        renderer.disable_depth_rendering()
        rgb = renderer.render()
        renderer.enable_depth_rendering()
        try:
            depth = renderer.render()
        finally:
            renderer.disable_depth_rendering()
        # Metric background depth can fall below the nominal far plane because
        # MuJoCo linearizes a float32 projection (notably at large far/near
        # ratios). Preserve the public segmentation no-object rule instead
        # of a distance threshold, which could erase valid distant surfaces.
        depth[self._background_reader.read(renderer)] = np.inf
        return CameraFrame(rgb=rgb, depth=depth, depth_kind="z")

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
        self._background_reader = None
        self._closed = True

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


class PanoramaCamera:
    """Full-sphere RGB and radial-range panorama from six 90-degree faces."""
    def __init__(self, model, width: int = 720, height: int = 360,
                 face_size: int = 256):
        self.width, self.height = _image_size(width, height)
        self.face_size, _ = _image_size(face_size, face_size)
        self._face_camera = PerspectiveCamera(model, face_size, face_size, 90.)
        self._lookup = _cubemap_lookup(self.width, self.height, face_size)

    def render(self, data, position, rotation_world_from_optical=None) -> CameraFrame:
        position, rotation = _pose(position, rotation_world_from_optical)
        rgb = np.empty((self.height, self.width, 3), dtype=np.uint8)
        depth = np.empty((self.height, self.width), dtype=np.float32)
        self._face_camera._update_scene(data)
        for face_rotation, lookup in zip(CUBE_ROTATIONS.values(), self._lookup):
            frame = self._face_camera._render_view(position, rotation @ face_rotation)
            _stitch_face(rgb, depth, frame.rgb, frame.depth, lookup)
        return CameraFrame(rgb=rgb, depth=depth, depth_kind="range")

    def close(self):
        self._face_camera.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
