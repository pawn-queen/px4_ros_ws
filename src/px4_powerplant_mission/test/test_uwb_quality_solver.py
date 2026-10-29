import numpy as np
import pytest

from px4_powerplant_mission.localization.uwb import solve_uwb_ranges


ANCHORS = np.array([
    [-25.0, -10.0, 6.0],
    [34.0, -10.0, 6.0],
    [34.0, 50.0, 18.0],
    [-25.0, 50.0, 6.0],
    [4.5, 20.0, 26.0],
    [0.0, 20.0, 14.0],
    [22.0, 4.0, 12.0],
])
TRUTH = np.array([4.0, 8.0, 5.0])


def solve(anchors, ranges, **overrides):
    parameters = dict(
        range_std_m=0.08,
        min_range_m=0.05,
        max_range_m=80.0,
        max_residual_rms_m=0.75,
        max_geometry_condition=1000.0,
    )
    parameters.update(overrides)
    return solve_uwb_ranges(anchors, ranges, **parameters)


def test_clean_ranges_solve_without_truth_initialization_and_report_covariance():
    ranges = np.linalg.norm(ANCHORS[:5] - TRUTH, axis=1)
    ranges += np.array([0.03, -0.05, 0.01, -0.02, 0.04])
    result = solve(ANCHORS[:5], ranges)

    assert np.linalg.norm(result.position_enu - TRUTH) < 0.15
    assert result.used_indices == (0, 1, 2, 3, 4)
    assert result.rejected_indices == ()
    assert result.residual_rms_m < 0.08
    assert 1.0 <= result.geometry_condition < 1000.0
    assert result.position_covariance_m2.shape == (3, 3)
    assert np.all(np.linalg.eigvalsh(result.position_covariance_m2) > 0)


def test_position_covariance_scales_with_range_noise():
    ranges = np.linalg.norm(ANCHORS[:5] - TRUTH, axis=1)
    low_noise = solve(ANCHORS[:5], ranges, range_std_m=0.08)
    high_noise = solve(ANCHORS[:5], ranges, range_std_m=0.16)

    assert np.allclose(high_noise.position_enu, low_noise.position_enu)
    assert np.allclose(
        high_noise.position_covariance_m2,
        4.0 * low_noise.position_covariance_m2,
    )


def test_invalid_and_out_of_range_samples_are_discarded_not_clipped():
    ranges = np.linalg.norm(ANCHORS - TRUTH, axis=1)
    ranges[0] = np.nan
    ranges[1] = 0.01
    ranges[2] = 90.0
    result = solve(ANCHORS, ranges)

    assert result.used_indices == (3, 4, 5, 6)
    assert result.rejected_indices == (0, 1, 2)
    assert np.linalg.norm(result.position_enu - TRUTH) < 1e-3


@pytest.mark.parametrize("boundary", [0.05, 80.0])
def test_saturated_range_endpoints_are_not_accepted(boundary):
    ranges = np.linalg.norm(ANCHORS[:5] - TRUTH, axis=1)
    ranges[0] = boundary
    result = solve(ANCHORS[:5], ranges)

    assert result.rejected_indices == (0,)
    assert result.used_indices == (1, 2, 3, 4)
    assert np.linalg.norm(result.position_enu - TRUTH) < 1e-3


def test_one_positive_bias_is_excluded_and_position_is_recomputed():
    ranges = np.linalg.norm(ANCHORS[:5] - TRUTH, axis=1)
    ranges += np.array([0.02, -0.02, 0.0, 0.03, -0.01])
    ranges[2] += 4.0
    result = solve(ANCHORS[:5], ranges)

    assert result.used_indices == (0, 1, 3, 4)
    assert result.rejected_indices == (2,)
    assert np.linalg.norm(result.position_enu - TRUTH) < 0.2
    assert result.residual_rms_m < 0.1


def test_three_or_coplanar_anchors_cannot_claim_a_3d_solution():
    ranges = np.linalg.norm(ANCHORS - TRUTH, axis=1)
    with pytest.raises(ValueError, match="at least four"):
        solve(ANCHORS[:3], ranges[:3])
    planar = ANCHORS[:4].copy()
    planar[:, 2] = 6.0
    with pytest.raises(ValueError, match="span three dimensions"):
        solve(planar, np.linalg.norm(planar - TRUTH, axis=1))


def test_nearly_coplanar_anchors_fail_geometry_gate():
    anchors = ANCHORS[:5].copy()
    anchors[:, 2] = 6.0
    anchors[4, 2] = 6.0001
    ranges = np.linalg.norm(anchors - TRUTH, axis=1)
    with pytest.raises(ValueError, match="geometry"):
        solve(anchors, ranges, max_geometry_condition=100.0)


def test_large_unresolvable_residual_fails_quality_gate():
    ranges = np.linalg.norm(ANCHORS[:5] - TRUTH, axis=1)
    ranges[0] += 8.0
    ranges[2] += 6.0
    with pytest.raises(ValueError, match="quality gate"):
        solve(ANCHORS[:5], ranges, max_residual_rms_m=0.2)


def test_each_single_positive_bias_is_removed_with_redundant_height_anchors():
    clean = np.linalg.norm(ANCHORS[:5] - TRUTH, axis=1)
    for biased_index in range(5):
        ranges = clean.copy()
        ranges[biased_index] += 2.0
        result = solve(ANCHORS[:5], ranges)
        assert result.rejected_indices == (biased_index,)
        assert np.linalg.norm(result.position_enu - TRUTH) < 1e-3


def test_normalized_residual_rejects_bias_below_loose_absolute_rms_limit():
    # A single elevated anchor cannot be checked by deleting it when the
    # remaining four are coplanar. The normalized residual still catches this
    # particular inconsistent measurement instead of trusting the 0.75 m cap.
    anchors = np.array([
        [-10.0, -10.0, 6.0],
        [34.0, -10.0, 6.0],
        [34.0, 34.0, 6.0],
        [-10.0, 34.0, 6.0],
        [12.0, 12.0, 26.0],
    ])
    ranges = np.linalg.norm(anchors - TRUTH, axis=1)
    ranges[4] += 2.0
    with pytest.raises(ValueError, match="quality gate"):
        solve(anchors, ranges)


def test_parameter_validation_rejects_unusable_noise_model():
    ranges = np.linalg.norm(ANCHORS[:5] - TRUTH, axis=1)
    with pytest.raises(ValueError, match="range_std_m"):
        solve(ANCHORS[:5], ranges, range_std_m=0.0)
