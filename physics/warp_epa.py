"""MuJoCo Warp 3.11 EPA face-selection correction; activate before graph capture."""
import linecache
import sys
import types
from importlib.metadata import version
from pathlib import Path

_FACE_PROJECTION = '''
@wp.func
def _face_projection_inside(pt: Polytope, i: int) -> bool:
  f = _get_face_verts(pt.face[i])
  a = pt.vert[2*f[0]] - pt.vert[2*f[0]+1]
  b = pt.vert[2*f[1]] - pt.vert[2*f[1]+1]
  c = pt.vert[2*f[2]] - pt.vert[2*f[2]+1]
  w = _tri_affine_coord(a, b, c, pt.face_pr[i])
  return w[0] >= 0.0 and w[1] >= 0.0 and w[2] >= 0.0
'''


def _load(name, source):
    filename = str(Path(__file__).parent / (name + '.py'))
    linecache.cache[filename] = (len(source), None, source.splitlines(True), filename)
    module = types.ModuleType(name)
    module.__file__ = filename
    sys.modules[name] = module
    exec(compile(source, filename, 'exec'), module.__dict__)
    return module


def install_epa_face_filter():
    """Call once before creating a Simulation; returns the activated kernel names.

    EPA's plane projection may lie outside a nearly degenerate face. Such a
    face must not supply interpolated witnesses. The rest of EPA, including
    face visibility and horizon expansion, remains the installed implementation.
    """
    if version('mujoco-warp') != '3.11.0':
        raise RuntimeError('EPA face filter requires MuJoCo Warp 3.11.0')
    from mujoco_warp._src import collision_convex, collision_gjk

    source = Path(collision_gjk.__file__).read_text()
    marker = '@wp.func\ndef _epa('
    selection = 'if not _is_invalid_face(pt.face[i]) and pt.face_norm2[i] < lower2:'
    if source.count(marker) != 1 or source.count(selection) != 1:
        raise RuntimeError('Installed EPA source does not match this correction')
    source = source.replace(marker, _FACE_PROJECTION + '\n' + marker, 1)
    source = source.replace(selection, selection[:-1] + ' and _face_projection_inside(pt, i):', 1)
    gjk = _load('_sprobo_epa_face_gjk', source)
    source = Path(collision_convex.__file__).read_text().replace(
        'from mujoco_warp._src.collision_gjk import', 'from _sprobo_epa_face_gjk import')
    # Upstream cache_kernel keys include the factory name, but not its module.
    source = source.replace('ccd_kernel_builder', 'face_ccd_kernel_builder')
    source = source.replace('ccd_hfield_kernel_builder', 'face_ccd_hfield_kernel_builder')
    convex = _load('_sprobo_epa_face_convex', source)
    factory = convex.face_ccd_kernel_builder
    if (factory.__wrapped__.__name__ == collision_convex.ccd_kernel_builder.__wrapped__.__name__
            or factory.__wrapped__.__globals__['epa_phase'] is not gjk.epa_phase):
        raise RuntimeError('EPA kernel factory binding did not change')
    collision_convex.ccd_kernel_builder = factory
    collision_convex.ccd_hfield_kernel_builder = convex.face_ccd_hfield_kernel_builder
    return {'mujoco_warp': '3.11.0', 'correction': 'epa_face_projection_membership',
            'ccd_factory': factory.__wrapped__.__name__,
            'hfield_factory': convex.face_ccd_hfield_kernel_builder.__wrapped__.__name__}
