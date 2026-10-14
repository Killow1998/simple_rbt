"""Projection/metric-depth contracts plus opt-in real offscreen rendering."""
import os

import numpy as np
import pytest

from simple_rbt_mujoco.cameras import (
    CUBE_ROTATIONS, PanoramaCamera, PerspectiveCamera,
    equirectangular_rays, perspective_rays, stitch_cubemap,
)


def test_perspective_pixel_centres_and_intrinsics():
    rays = perspective_rays(5, 5, 90.)
    np.testing.assert_allclose(rays[2, 2], [0., 0., 1.], atol=1e-15)
    np.testing.assert_allclose(rays[0, 0], [-.8, -.8, 1.])
    camera = PerspectiveCamera(None, width=5, height=5, fov_y_deg=90.)
    # A point on the optical axis projects to the centre pixel.
    np.testing.assert_allclose(camera.intrinsics @ np.array([0., 0., 1.]), [2, 2, 1])
    # Z=2 does not mean radial range=2 off the optical axis.
    assert np.linalg.norm(rays[0, 0] * 2.) > 3.


def test_panorama_axis_convention_and_full_sphere():
    rays = equirectangular_rays(9, 5)
    np.testing.assert_allclose(np.linalg.norm(rays, axis=-1), 1.)
    np.testing.assert_allclose(rays[2, 4], [0., 0., 1.], atol=1e-15)
    assert rays[2, 6, 0] > .98  # Right quarter looks optical +X.
    assert rays[2, 2, 0] < -.98
    assert rays[0, 4, 1] < -.95  # Top looks optical -Y.
    assert rays[-1, 4, 1] > .95
    assert rays[2, 0, 2] < -.93 and rays[2, -1, 2] < -.93


def test_stitch_preserves_direction_and_converts_z_to_range():
    size = 65
    local = perspective_rays(size, size, 90.)
    ray_lengths = np.linalg.norm(local, axis=-1)
    rgb, depth = {}, {}
    for name, rotation in CUBE_ROTATIONS.items():
        direction = (local / ray_lengths[..., None]) @ rotation.T
        # A sphere at radius 4 has nonconstant optical-Z depth on each face.
        rgb[name] = np.rint((direction + 1.) * 127.5).astype(np.uint8)
        depth[name] = (4. / ray_lengths).astype(np.float32)
    frame = stitch_cubemap(rgb, depth, width=129, height=65)
    expected_rgb = (equirectangular_rays(129, 65) + 1.) * 127.5
    assert np.max(np.abs(frame.rgb.astype(float) - expected_rgb)) < 3.3
    np.testing.assert_allclose(frame.depth, 4., atol=1e-6)
    assert frame.depth_kind == "range"
    assert frame.rgb.dtype == np.uint8 and frame.depth.dtype == np.float32


def test_stitch_does_not_turn_missing_depth_into_surface():
    rgb = {name: np.zeros((3, 3, 3), np.uint8) for name in CUBE_ROTATIONS}
    depth = {name: np.full((3, 3), np.inf) for name in CUBE_ROTATIONS}
    assert np.isinf(stitch_cubemap(rgb, depth, 17, 9).depth).all()


@pytest.mark.parametrize("kwargs", [{"width": 0}, {"height": 1.5},
                                     {"fov_y_deg": 180}, {"fov_y_deg": np.nan}])
def test_invalid_camera_parameters_fail_before_opening_graphics(kwargs):
    with pytest.raises(ValueError):
        PerspectiveCamera(None, **kwargs)


def test_invalid_pose_and_closed_camera_fail_before_opening_graphics():
    camera = PerspectiveCamera(None)
    with pytest.raises(ValueError, match="proper 3x3"):
        camera.render(None, [0, 0, 0], np.diag([1., 1., -1.]))
    camera.close()
    with pytest.raises(RuntimeError, match="closed"):
        camera.render(None, [0, 0, 0])


@pytest.mark.skipif(os.environ.get("SIMPLE_RBT_TEST_RENDER") != "1",
                    reason="Set SIMPLE_RBT_TEST_RENDER=1 with a host OpenGL backend")
def test_real_offscreen_cameras_inside_measured_room():
    """Six real surfaces at +/-2 m catch pose, depth and cubemap mistakes."""
    import mujoco

    model = mujoco.MjModel.from_xml_string('''
    <mujoco>
      <visual><map znear="0.001" zfar="10"/>
        <headlight ambient="1 1 1" diffuse="0 0 0" specular="0 0 0"/>
      </visual>
      <worldbody>
        <geom type="box" pos="0 0 2.05" size="2.1 2.1 .05" rgba="0 0 1 1"/>
        <geom type="box" pos="0 0 -2.05" size="2.1 2.1 .05" rgba="0 1 1 1"/>
        <geom type="box" pos="2.05 0 0" size=".05 2.1 2.1" rgba="1 0 0 1"/>
        <geom type="box" pos="-2.05 0 0" size=".05 2.1 2.1" rgba="1 1 0 1"/>
        <geom type="box" pos="0 -2.05 0" size="2.1 .05 2.1" rgba="0 1 0 1"/>
        <geom type="box" pos="0 2.05 0" size="2.1 .05 2.1" rgba="1 0 1 1"/>
      </worldbody>
    </mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    with PerspectiveCamera(model, width=81, height=61, fov_y_deg=60.) as camera:
        frame = camera.render(data, [0, 0, 0])
        np.testing.assert_allclose(frame.depth, 2., atol=2e-4)
        assert frame.depth_kind == "z"
        np.testing.assert_array_equal(frame.rgb[30, 40] > 200, [False, False, True])
        rotated = camera.render(data, [0, 0, 0], CUBE_ROTATIONS["right"])
        np.testing.assert_allclose(rotated.depth, 2., atol=2e-4)
        np.testing.assert_array_equal(rotated.rgb[30, 40] > 200, [True, False, False])
        # Translation must change metric depth, not merely the view orientation.
        shifted = camera.render(data, [0, 0, .5])
        assert shifted.depth[30, 40] == pytest.approx(1.5, abs=2e-4)
    with PanoramaCamera(model, width=129, height=65, face_size=65) as camera:
        frame = camera.render(data, [0, 0, 0])
        expected = 2. / np.max(np.abs(equirectangular_rays(129, 65)), axis=-1)
        np.testing.assert_allclose(frame.depth, expected, rtol=.017, atol=.002)
        assert frame.depth_kind == "range"
        np.testing.assert_array_equal(frame.rgb[32, 64] > 200, [False, False, True])
        np.testing.assert_array_equal(frame.rgb[32, 96] > 200, [True, False, False])
        np.testing.assert_array_equal(frame.rgb[0, 64] > 200, [False, True, False])
        np.testing.assert_array_equal(frame.rgb[-1, 64] > 200, [True, False, True])


@pytest.mark.skipif(os.environ.get("SIMPLE_RBT_TEST_RENDER") != "1",
                    reason="Set SIMPLE_RBT_TEST_RENDER=1 with a host OpenGL backend")
@pytest.mark.parametrize("near_fraction", [.01, .005, .0001])
def test_real_empty_scene_depth_is_infinite(near_fraction):
    import mujoco
    model = mujoco.MjModel.from_xml_string(
        f'<mujoco><visual><map znear="{near_fraction}" zfar="50"/></visual><worldbody/></mujoco>')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    with PerspectiveCamera(model, width=17, height=13) as camera:
        frame = camera.render(data, [0, 0, 0])
        assert np.isinf(frame.depth).all()


@pytest.mark.parametrize("face_size,width,height", [(1, 1, 1), (3, 4, 2), (5, 17, 9),
                                                         (11, 129, 65), (32, 720, 360)])
def test_flat_stitch_matches_original_mask_algorithm_exactly(face_size, width, height):
    """Indexed stitching keeps every nearest sample and float rounding unchanged."""
    from simple_rbt_mujoco.cameras import _cubemap_lookup, _stitch
    rng = np.random.default_rng(20261013)
    faces_rgb = {name: rng.integers(0, 256, (face_size, face_size, 3), dtype=np.uint8)
                 for name in CUBE_ROTATIONS}
    faces_depth = {name: rng.uniform(.1, 100., (face_size, face_size)).astype(np.float32)
                   for name in CUBE_ROTATIONS}
    for depth in faces_depth.values():
        depth[::3, ::2] = np.inf
    rays = equirectangular_rays(width, height)
    faces = np.argmax(np.stack([rays @ r[:, 2] for r in CUBE_ROTATIONS.values()], axis=-1), axis=-1)
    factor = np.linalg.norm(perspective_rays(face_size, face_size, 90.), axis=-1)
    expected_rgb = np.empty((height, width, 3), np.uint8)
    expected_depth = np.empty((height, width), np.float32)
    for face_id, (name, rotation) in enumerate(CUBE_ROTATIONS.items()):
        mask = faces == face_id
        local = rays[mask] @ rotation
        pixels = np.clip(np.floor((local[:, :2] / local[:, 2, None] + 1.) * face_size / 2.).astype(int),
                         0, face_size - 1)
        rows, cols = pixels[:, 1], pixels[:, 0]
        expected_rgb[mask] = faces_rgb[name][rows, cols]
        expected_depth[mask] = faces_depth[name][rows, cols] * factor[rows, cols]
    actual = _stitch(faces_rgb, faces_depth, width, height, _cubemap_lookup(width, height, face_size))
    np.testing.assert_array_equal(actual.rgb, expected_rgb)
    np.testing.assert_array_equal(actual.depth, expected_depth)


def test_depth_id_readback_preserves_objectless_geometry_and_image_flip(monkeypatch):
    """No-hit means mapped objid==-1, including a nonzero segment ID."""
    import sys
    from types import SimpleNamespace
    from simple_rbt_mujoco.cameras import _DepthBackgroundReader
    ids = np.array([[0, 1, 257], [65537, 1, 0]], np.uint32)
    encoded = np.stack((ids & 255, (ids >> 8) & 255, (ids >> 16) & 255), axis=-1).astype(np.uint8)
    class ClassicStub:
        __module__ = "mujoco.rendering.classic.renderer"
        _rect, _mjr_context, _gl_context = object(), object(), object()
        scene = SimpleNamespace(ngeom=65537, geoms=[
            SimpleNamespace(segid=0, objid=7), SimpleNamespace(segid=256, objid=-1),
            SimpleNamespace(segid=65536, objid=9), SimpleNamespace(segid=-1, objid=1)])
        def render(self):
            raise AssertionError("Depth-ID readback must not render again")
    reads = []
    def read_pixels(rgb, depth, rect, context):
        assert depth is None
        reads.append((rect, context))
        np.copyto(rgb, encoded)
    monkeypatch.setitem(sys.modules, "mujoco", SimpleNamespace(mjr_readPixels=read_pixels))
    renderer = ClassicStub()
    reader = _DepthBackgroundReader(renderer, 3, 2)
    reader.update_scene(renderer)
    np.testing.assert_array_equal(reader.read(renderer), [[False, False, True], [True, False, True]])
    assert len(reads) == 1
    renderer._gl_context = None
    np.testing.assert_array_equal(reader.read(renderer), [[True, False, True], [False, False, True]])


@pytest.mark.parametrize("private_contract_present", [False, True])
def test_unsupported_depth_id_readback_uses_public_segmentation(monkeypatch, private_contract_present):
    import sys
    from types import SimpleNamespace
    from simple_rbt_mujoco.cameras import _DepthBackgroundReader
    class RendererStub:
        _rect, _mjr_context, _gl_context = object(), object(), object()
        scene = SimpleNamespace(ngeom=0, geoms=[])
        enabled = False
        calls = 0
        def enable_segmentation_rendering(self):
            self.enabled = True
        def disable_segmentation_rendering(self):
            self.enabled = False
        def render(self):
            assert self.enabled
            self.calls += 1
            return np.array([[[-1, -1], [0, 5]]])
    if private_contract_present:
        RendererStub.__module__ = "mujoco.rendering.classic.renderer"
    def unavailable(*args):
        raise TypeError("unsupported readback interface")
    monkeypatch.setitem(sys.modules, "mujoco", SimpleNamespace(mjr_readPixels=unavailable))
    renderer = RendererStub()
    reader = _DepthBackgroundReader(renderer, 2, 1)
    reader.update_scene(renderer)
    np.testing.assert_array_equal(reader.read(renderer), [[True, False]])
    assert renderer.calls == 1 and not renderer.enabled
    assert not reader.supported


@pytest.mark.parametrize("constructor_fails", [False, True])
def test_renderer_uses_actual_viewport_and_restores_model_even_on_failure(monkeypatch, constructor_fails):
    import sys
    from types import SimpleNamespace
    from simple_rbt_mujoco.cameras import _make_renderer
    visual = SimpleNamespace(offwidth=1024, offheight=1024)
    model = SimpleNamespace(vis=SimpleNamespace(global_=visual))
    result = object()
    def construct(received_model, *, width, height):
        assert received_model is model
        assert (visual.offwidth, visual.offheight) == (width, height) == (81, 61)
        if constructor_fails:
            raise RuntimeError("allocation failed")
        return result
    monkeypatch.setitem(sys.modules, "mujoco", SimpleNamespace(Renderer=construct))
    if constructor_fails:
        with pytest.raises(RuntimeError, match="allocation failed"):
            _make_renderer(model, 81, 61)
    else:
        assert _make_renderer(model, 81, 61) is result
    assert (visual.offwidth, visual.offheight) == (1024, 1024)


def test_panorama_updates_scene_once_and_streams_faces_into_independent_frames():
    """One pose is shared across six views, while completed frames stay owned."""
    from simple_rbt_mujoco.cameras import CameraFrame
    class FaceCamera:
        def __init__(self):
            self.updates = self.views = 0
            self.rgb = np.zeros((5, 5, 3), np.uint8)
            self.depth = np.zeros((5, 5), np.float32)
        def _update_scene(self, data):
            self.updates += 1
        def _render_view(self, position, rotation):
            self.views += 1
            self.rgb[:] = self.views
            self.depth[:] = self.views
            return CameraFrame(self.rgb, self.depth, "z")
    panorama = PanoramaCamera(None, 17, 9, 5)
    face_camera = panorama._face_camera = FaceCamera()
    first = panorama.render(None, [0, 0, 0])
    saved_rgb, saved_depth = first.rgb.copy(), first.depth.copy()
    assert face_camera.updates == 1 and face_camera.views == 6
    assert set(np.unique(first.rgb)) == set(range(1, 7))
    second = panorama.render(None, [0, 0, 0])
    assert face_camera.updates == 2 and face_camera.views == 12
    np.testing.assert_array_equal(first.rgb, saved_rgb)
    np.testing.assert_array_equal(first.depth, saved_depth)
    assert not np.shares_memory(first.rgb, second.rgb)
    assert not np.shares_memory(first.depth, second.depth)


def test_streamed_face_selection_matches_stack_at_exact_edges_and_corners(monkeypatch):
    from itertools import product
    import simple_rbt_mujoco.cameras as cameras
    # All six axes, twelve two-face edges and eight three-face corners. Equal
    # nonzero components remain bit-identical after normalization.
    rays = np.array([v for v in product((-1., 0., 1.), repeat=3) if any(v)])
    rays /= np.linalg.norm(rays, axis=1, keepdims=True)
    rays = rays.reshape(2, 13, 3)
    monkeypatch.setattr(cameras, "equirectangular_rays", lambda width, height: rays)
    scores = np.stack([rays @ r[:, 2] for r in CUBE_ROTATIONS.values()], axis=-1)
    expected_faces = np.argmax(scores, axis=-1)
    for size in (1, 4, 9):
        lookup = cameras._cubemap_lookup(13, 2, size)
        actual_faces = np.empty(26, dtype=np.uint8)
        factor = np.linalg.norm(perspective_rays(size, size, 90.), axis=-1).ravel()
        for face_id, ((destination, source, scale), rotation) in enumerate(zip(lookup, CUBE_ROTATIONS.values())):
            expected_destination = np.flatnonzero(expected_faces == face_id)
            np.testing.assert_array_equal(destination, expected_destination)
            local = rays.reshape(-1, 3)[expected_destination] @ rotation
            pixels = np.clip(np.floor((local[:, :2] / local[:, 2, None] + 1.) * size / 2.).astype(int),
                             0, size - 1)
            expected_source = pixels[:, 1] * size + pixels[:, 0]
            np.testing.assert_array_equal(source, expected_source)
            np.testing.assert_array_equal(scale, factor[expected_source])
            actual_faces[destination] = face_id
        np.testing.assert_array_equal(actual_faces.reshape(2, 13), expected_faces)
        for direction, expected in [((1., 0., 1.), 0), ((1., -1., 0.), 1),
                                    ((0., -1., -1.), 2), ((-1., -1., 0.), 3)]:
            unit = np.array(direction) / np.linalg.norm(direction)
            index, = np.flatnonzero(np.all(rays.reshape(-1, 3) == unit, axis=1))
            assert actual_faces[index] == expected
