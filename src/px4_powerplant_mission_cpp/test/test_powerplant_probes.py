"""A finite-radius contact must be predicted against the complete surface."""
from pathlib import Path
import importlib.util
import math
import numpy as np
import pytest

p = Path(__file__).resolve().parents[1] / 'scripts/build_powerplant_supplemental_fixtures.py'
spec = importlib.util.spec_from_file_location('supplemental_probes', p)
probes = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probes)


def test_sphere_touches_adjacent_edge_before_center_ray_plane():
    edge = np.array([.04, .03, 0.])
    def nearest(point):
        plane = np.array([0., point[1], point[2]])
        q = min((plane, edge), key=lambda q: np.linalg.norm(point-q))
        return q, float(np.linalg.norm(point-q))
    hit = probes.sphere_first_contact(nearest, [.2, 0., 0.], [-.1, 0., 0.], .05)
    np.testing.assert_allclose(hit['surface'], edge, atol=1e-7)
    np.testing.assert_allclose(hit['center'], [.08, 0., 0.], atol=2e-7)
    assert hit['travel_m'] == pytest.approx(.12, abs=2e-7)
    assert np.linalg.norm(hit['surface']) > .025  # center-ray expectation is wrong


def test_sphere_contacts_slanted_plane_at_nearest_surface_point():
    n = np.array([1., 1., 0.])/math.sqrt(2)
    def nearest(point):
        distance = float(point@n)
        return point-distance*n, abs(distance)
    hit = probes.sphere_first_contact(nearest, [.3, 0., 0.], [-.1, 0., 0.], .05)
    assert hit['center'][0] == pytest.approx(.05*math.sqrt(2), abs=2e-7)
    assert np.linalg.norm(hit['center']-hit['surface']) == pytest.approx(.05, abs=1e-7)


def test_sphere_no_contact_before_endpoint_is_explicit():
    def nearest(point):
        return np.array([0., point[1], point[2]]), abs(float(point[0]))
    assert probes.sphere_first_contact(nearest, [.3, 0., 0.], [.2, 0., 0.], .05) is None
    with pytest.raises(ValueError, match='initial'):
        probes.sphere_first_contact(nearest, [.03, 0., 0.], [-.1, 0., 0.], .05)
