"""Native scan metadata must remain valid through offline geometry evaluation."""
from pathlib import Path
from types import SimpleNamespace
import importlib.util
import math
import pytest

VALIDATION = Path(__file__).resolve().parents[1]
SCRIPT = VALIDATION / 'sensors/verify_samples.py'
spec = importlib.util.spec_from_file_location('powerplant_sensor_geometry', SCRIPT)
geometry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(geometry)


def singleton_scan(**changes):
    # gz-sensors 8.2.2 emits NaN vertical step for its one horizontal layer.
    fields = dict(count=3, vertical_count=1, ranges=[3., 4., 5.],
                  range_min=.15, range_max=35., angle_min=-math.pi/2,
                  angle_max=math.pi/2, angle_step=math.pi/2,
                  vertical_angle_min=0., vertical_angle_max=0.,
                  vertical_angle_step=math.nan)
    return SimpleNamespace(**{**fields, **changes})


def test_native_single_layer_nan_step_preserves_finite_world_rays():
    np = pytest.importorskip('numpy')
    pose = np.eye(4)
    pose[:3, 3] = [25., 30., 5.42]
    samples = geometry.lidar_samples(singleton_scan(), pose)
    assert len(samples) == 3
    assert all(np.isfinite(sample['ray']).all() for sample in samples)
    np.testing.assert_allclose([sample['ray'] for sample in samples],
                               [[0., -1., 0.], [1., 0., 0.], [0., 1., 0.]], atol=1e-12)
    cloud = {'minimum_m': np.asarray([-1.615, 3.385, -1.115]),
             'maximum_m': np.asarray([11.615, 16.615, 17.115])}
    # These horizontal rays point away from the captured smoke volume.
    assert not any(geometry.intersects_box(s['origin'], s['ray'], s['maximum'],
                                          cloud['minimum_m'], cloud['maximum_m']) for s in samples)


def test_multiple_scan_layers_cannot_ignore_nan_step():
    np = pytest.importorskip('numpy')
    with pytest.raises(ValueError, match='vertical'):
        geometry.lidar_samples(singleton_scan(vertical_count=2, ranges=[3.] * 6), np.eye(4))


def test_singleton_elevation_and_world_rotation_are_preserved():
    np = pytest.importorskip('numpy')
    pose = np.eye(4)
    pose[:3, :3] = [[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]]
    scan = singleton_scan(count=1, ranges=[4.], angle_min=0., angle_max=0.,
                          angle_step=math.nan, vertical_angle_min=.2, vertical_angle_max=.2)
    ray = geometry.lidar_samples(scan, pose)[0]['ray']
    assert ray == pytest.approx([0., math.cos(.2), math.sin(.2)])


@pytest.mark.parametrize('field,value', [('angle_min', math.nan), ('angle_step', math.nan),
                                         ('range_max', math.inf), ('vertical_angle_min', math.nan)])
def test_invalid_scan_geometry_is_rejected(field, value):
    np = pytest.importorskip('numpy')
    with pytest.raises(ValueError):
        geometry.lidar_samples(singleton_scan(**{field: value}), np.eye(4))


def test_nan_ray_cannot_be_misreported_as_particle_exclusion():
    with pytest.raises(ValueError, match='finite'):
        geometry.intersects_box([25., 30., 5.42], [math.nan] * 3, 35., [-2., 3., -1.], [12., 17., 18.])


def test_single_layer_scan_reaches_geometry_comparison_instead_of_false_smoke_exclusion():
    np = pytest.importorskip('numpy')
    pose = np.eye(4)
    pose[:3, 3] = [25., 30., 5.42]
    scan = singleton_scan(ranges=[4.] * 3)
    scene = SimpleNamespace(first_hit=lambda origin, ray, maximum:
                            {'parameter': 4., 'model_name': 'known_surface'})
    clouds = [{'minimum_m': [-1.615, 3.385, -1.115],
               'maximum_m': [11.615, 16.615, 17.115]}]
    result = geometry.compare_samples(geometry.lidar_samples(scan, pose), scene, scene, clouds, 3, .08, .05)
    assert result['status'] == 'PASS'
    assert result['clean_samples'] == 3 and not result['excluded']


CORE = VALIDATION / 'sensors/lidar_rays.py'
core_spec = importlib.util.spec_from_file_location('powerplant_lidar_sampling', CORE)
sampling = importlib.util.module_from_spec(core_spec)
core_spec.loader.exec_module(sampling)


def test_cubemap_single_layer_prediction_is_independent_of_measured_ranges():
    np = pytest.importorskip('numpy')
    scan = singleton_scan()
    scan.ranges = [math.inf, .01, 1000.]
    first = sampling.predict_scan(scan, np.eye(4))
    scan.ranges = [0., math.nan, -1.]
    second = sampling.predict_scan(scan, np.eye(4))
    np.testing.assert_array_equal(first['world_rays'], second['world_rays'])
    assert first['first_pass_texture_size'] == 128
    assert first['directions_use_measured_ranges'] is False
    assert all(np.isfinite(ray).all() for ray in first['world_rays'])


@pytest.mark.parametrize('horizontal,vertical,face', [
    (0., 0., 4), (math.pi, 0., 5), (-math.pi/2, 0., 0),
    (math.pi/2, 0., 1), (0., math.pi/2, 2), (0., -math.pi/2, 3)])
def test_cubemap_all_six_faces_preserve_the_source_camera_orientation(horizontal, vertical, face):
    np = pytest.importorskip('numpy')
    observed_face, uv = sampling.cubemap_uv(horizontal, vertical)
    assert observed_face == face
    ray = sampling.cube_pixel_to_flu(face, *uv)
    np.testing.assert_allclose(ray, [math.cos(vertical)*math.cos(horizontal),
                                  math.cos(vertical)*math.sin(horizontal), math.sin(vertical)], atol=1e-7)


def test_cubemap_face_seam_uses_clamp_instead_of_wrapping_to_opposite_edge():
    np = pytest.importorskip('numpy')
    scan = singleton_scan(count=1, ranges=[5.], angle_min=-math.pi/4,
                          angle_max=-math.pi/4, angle_step=math.nan)
    result = sampling.predict_scan(scan, np.eye(4))
    assert result['rays'][0]['angular_difference_deg'] < .65
    assert result['rays'][0]['pixel'][0] in (0, 127)


def test_cubemap_uv_float32_rounding_happens_before_nearest_pixel_selection():
    np = pytest.importorskip('numpy')
    boundary = 65/128
    double_uv_below_boundary = boundary-1e-10
    angle = -math.atan(2*double_uv_below_boundary-1)
    scan = singleton_scan(count=1, ranges=[5.], angle_min=angle, angle_max=angle, angle_step=math.nan)
    record = sampling.predict_scan(scan, np.eye(4))['rays'][0]
    assert record['face'] == 4
    assert record['uv_float32'][0] == boundary
    assert record['pixel'][0] == 65  # float64 lookup would incorrectly select 64


@pytest.fixture
def renderer_evidence(tmp_path, monkeypatch):
    import hashlib
    resources = tmp_path/'resources'
    resources.mkdir()
    recorded_resources, resource_hashes = [], {}
    for name in sampling.SUPPORTED_RESOURCES:
        path = resources/name
        path.write_bytes(('trusted source resource '+name).encode())
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        resource_hashes[name] = digest
        recorded_resources.append({'path': str(path), 'sha256': digest})
    recorded_libraries, library_hashes = [], {}
    for name in sampling.SUPPORTED_LIBRARIES:
        path = tmp_path/name
        path.write_bytes(('trusted binary '+name).encode())
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        library_hashes[name] = {digest}
        recorded_libraries.append({'path': str(path), 'sha256': digest})
    # Inject a trusted build catalog at the evidence-verification seam. Tests
    # do not depend on this host's installed shared libraries or ROS package.
    monkeypatch.setattr(sampling, 'SUPPORTED_LIBRARIES', library_hashes)
    monkeypatch.setattr(sampling, 'SUPPORTED_RESOURCES', resource_hashes)
    log = tmp_path/'ogre2.log'
    log.write_text("Version 2.3.3\nGL3PlusRenderSystem\nGL_RENDERER = test fixture\n"
                   f"Added resource location '{resources}' of type 'FileSystem'\n")
    report = {'runtime_renderer_libraries': recorded_libraries,
              'runtime_renderer_resources': recorded_resources,
              'renderer_resource_log_sha256': hashlib.sha256(log.read_bytes()).hexdigest()}
    return report, log


def test_cubemap_capture_binds_registered_shader_and_runtime_binary_hashes(renderer_evidence):
    report, log = renderer_evidence
    result = sampling.verify_renderer_evidence(report, log)
    assert result['verified'] and result['version'] == 'gz-rendering 8.2.3'
    assert result['direction_uses_measured_ranges'] is False


def test_cubemap_historical_capture_without_renderer_hashes_is_unconfirmed(tmp_path):
    with pytest.raises(sampling.RendererEvidenceUnavailable, match='lacks'):
        sampling.verify_renderer_evidence({}, tmp_path/'ogre2.log')


def test_cubemap_unknown_renderer_version_is_rejected(renderer_evidence):
    report, log = renderer_evidence
    original = report['runtime_renderer_libraries'][0]
    original['path'] = original['path'].replace('.8.2.3', '.8.2.4')
    with pytest.raises(sampling.RendererEvidenceUnavailable, match='unsupported'):
        sampling.verify_renderer_evidence(report, log)


def test_cubemap_changed_shader_cannot_reuse_prior_evidence(renderer_evidence):
    report, log = renderer_evidence
    Path(report['runtime_renderer_resources'][0]['path']).write_text('changed filtering')
    with pytest.raises(sampling.RendererEvidenceMismatch, match='changed'):
        sampling.verify_renderer_evidence(report, log)


def test_cubemap_ambiguous_resource_resolution_is_not_accepted(renderer_evidence):
    report, log = renderer_evidence
    report['runtime_renderer_resources'].append(dict(report['runtime_renderer_resources'][0]))
    with pytest.raises(sampling.RendererEvidenceUnavailable, match='ambiguous'):
        sampling.verify_renderer_evidence(report, log)
