from types import SimpleNamespace

import numpy as np
import pytest

from px4_powerplant_slam.geometry import Occlusion, pinhole, register_depth


def frame(values, encoding='32FC1', padding=0, big=False):
    buf = b''.join(v.tobytes() + bytes(padding) for v in values)
    return SimpleNamespace(width=values.shape[1], height=values.shape[0], encoding=encoding,
                           step=values.shape[1] * values.itemsize + padding,
                           data=buf, is_bigendian=big)


def test_registration_uses_principal_point_and_leaves_missing_rays_unknown():
    depth = frame(np.array([[1., 2., 4., 8.]], dtype='float32'), padding=8)
    src = (4, 1, 2., 1., 1., 0.)
    dst = (5, 1, 2., 1., 2., 0.)
    got = register_depth(depth, dst, src)
    np.testing.assert_allclose(got, [[np.nan, 1., 2., 4., 8.]], equal_nan=True)


def test_uint16_conversion_and_unknown_depth():
    depth = frame(np.array([[0, 2000]], dtype='>u2'), '16UC1', big=True)
    model = (2, 1, 3., 3., 0., 0.)
    np.testing.assert_allclose(register_depth(depth, model, model), [[np.nan, 2.]], equal_nan=True)
    depth.data = b''
    with pytest.raises(ValueError, match='stride'):
        register_depth(depth, model, model)


def test_calibration_must_match_its_image():
    k = [3., 0., 1., 0., 3., 1., 0., 0., 1.]
    info = SimpleNamespace(width=2, height=2, k=k, d=[0.] * 5)
    image = SimpleNamespace(width=2, height=2)
    assert pinhole(info, image) == (2, 2, 3., 3., 1., 1.)
    info.width = 3
    with pytest.raises(ValueError, match='dimensions'):
        pinhole(info, image)
    info.width = 2
    info.d[0] = 0.01
    with pytest.raises(ValueError, match='rectified'):
        pinhole(info, image)


def test_occlusion_uses_sample_time_and_releases_after_external_event():
    occlusion = Occlusion(start_s=1., duration_s=2.)
    assert not occlusion.masked(10_000_000_000)
    assert occlusion.masked(11_000_000_000)
    assert occlusion.masked(12_000_000_000)
    assert not occlusion.masked(13_000_000_000)
    occlusion.external = True
    assert occlusion.masked(14_000_000_000)
    occlusion.external = False
    assert not occlusion.masked(15_000_000_000)
