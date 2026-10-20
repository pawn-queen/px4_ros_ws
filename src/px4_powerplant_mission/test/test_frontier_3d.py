import math

import numpy as np

from px4_powerplant_mission.path_planning.frontier_3d import (
    ExplorationConfig,
    _visible,
    _view_gain,
    _first_non_free,
    extract_frontiers,
    path_is_known_free,
    plan_frontier,
    plan_known_free_path,
)


def box(xrange, yrange, zrange):
    return {(x, y, z) for x in xrange for y in yrange for z in zrange}


def test_frontier_is_known_free_adjacent_to_unknown_within_bounds():
    frontiers = extract_frontiers(
        {(1, 1, 1), (2, 1, 1)},
        {(2, 2, 1)},
        ((1, 1, 1), (2, 2, 1)),
    )
    assert frontiers[(1, 1, 1)] == {(1, 2, 1)}
    assert (2, 2, 1) not in frontiers.get((2, 1, 1), set())
    assert all(1 <= axis <= 2 for neighbors in frontiers.values() for key in neighbors for axis in key)


def test_known_free_path_uses_3d_gap_and_rejects_unknown_or_obstacle():
    free = box(range(5), range(5), range(5))
    occupied = {(2, y, z) for y in range(4) for z in range(4)}
    free -= occupied
    bounds = ((0, 0, 0), (4, 4, 4))
    config = ExplorationConfig(clearance_radius_m=0.1)
    start = (1.5, 1.5, 1.5)
    goal = (3.5, 1.5, 1.5)

    route = plan_known_free_path(free, occupied, 1.0, bounds, start, goal, config)
    assert route
    assert any(point[1] >= 4.5 or point[2] >= 4.5 for point in route)
    assert path_is_known_free(free, occupied, 1.0, bounds, route, config)
    assert not path_is_known_free(free, occupied, 1.0, bounds, [start, goal], config)

    # A map update blocks the already-issued local path.
    visited = tuple(np.floor(route[len(route) // 2]).astype(int))
    assert not path_is_known_free(free - {visited}, occupied, 1.0, bounds, route, config)
    assert not path_is_known_free(free - {visited}, occupied | {visited}, 1.0, bounds, route, config)


def test_explorer_chooses_free_viewpoint_and_only_short_local_segment():
    free = box(range(6), range(5), range(5))
    bounds = ((0, 0, 0), (8, 4, 4))
    current = np.array([1.5, 2.5, 2.5])
    config = ExplorationConfig(
        clearance_radius_m=0.1,
        max_local_path_m=0.5,
        reserve_s=5.0,
        max_candidates=48,
    )
    plan = plan_frontier(
        free, set(), 1.0, bounds, current, current,
        remaining_flight_s=50.0, config=config,
    )

    assert plan is not None
    assert tuple(np.floor(plan.goal_xyz).astype(int)) in free
    assert plan.frontier_key not in free
    assert plan.frontier_key[0] == 6
    assert plan.visible_unknown_count > 0
    assert plan.return_time_s >= 0.0
    assert plan.reserve_margin_s >= 0.0
    assert math.isclose(plan.goal_yaw_rad, 0.0, abs_tol=math.pi / 6)
    assert np.allclose(plan.local_path_xyz[0], current)
    length = sum(
        np.linalg.norm(b - a)
        for a, b in zip(plan.local_path_xyz, plan.local_path_xyz[1:])
    )
    assert 0.0 < length <= config.max_local_path_m + 1e-9
    assert path_is_known_free(free, set(), 1.0, bounds, plan.local_path_xyz, config)
    assert np.linalg.norm(plan.local_path_xyz[-1] - plan.goal_xyz) > 0.0


def test_surface_gain_is_scored_but_unknown_gap_is_not_visible():
    free = box(range(6), range(5), range(5))
    occupied = {(6, 2, 2)}
    bounds = ((0, 0, 0), (8, 4, 4))
    current = (1.5, 2.5, 2.5)
    config = ExplorationConfig(clearance_radius_m=0.1, reserve_s=5.0)
    plan = plan_frontier(
        free, occupied, 1.0, bounds, current, current,
        remaining_flight_s=50.0,
        surface_gain_by_key={(6, 2, 2): 4.0, (7, 2, 2): 100.0},
        config=config,
    )
    assert plan is not None
    assert plan.surface_gain == 4.0


def test_explorer_holds_when_start_has_no_safe_move_or_return_budget():
    free = {(0, 0, 0), (2, 0, 0)}
    bounds = ((0, 0, 0), (3, 0, 0))
    config = ExplorationConfig(
        clearance_radius_m=0.0,
        min_goal_distance_m=0.75,
        reserve_s=0.0,
    )
    assert plan_frontier(
        free, set(), 1.0, bounds, (0.5, 0.5, 0.5), (0.5, 0.5, 0.5),
        remaining_flight_s=50.0, config=config,
    ) is None

    free = box(range(6), range(5), range(5))
    bounds = ((0, 0, 0), (8, 4, 4))
    config = ExplorationConfig(clearance_radius_m=0.1, reserve_s=49.0)
    assert plan_frontier(
        free, set(), 1.0, bounds, (1.5, 2.5, 2.5), (1.5, 2.5, 2.5),
        remaining_flight_s=50.0, config=config,
    ) is None
    assert plan_frontier(
        free, set(), 1.0, bounds, (1.5, 2.5, 2.5), (1.5, 2.5, 2.5),
        remaining_flight_s=50.0, position_variance_m2=5.0,
        config=ExplorationConfig(clearance_radius_m=0.1),
    ) is None
    assert plan_frontier(
        free, set(), 1.0, bounds, (1.5, 2.5, 2.5), (1.5, 2.5, 2.5),
        remaining_flight_s=50.0,
        localization_variance_by_key={
            key: 5.0 for key in free if key != (1, 2, 2)
        },
        config=ExplorationConfig(clearance_radius_m=0.1, reserve_s=5.0),
    ) is None


def test_viewpoint_backs_off_from_unknown_by_clearance_radius():
    free = box(range(6), range(5), range(5))
    bounds = ((0, 0, 0), (8, 4, 4))
    current = (1.5, 2.5, 2.5)
    config = ExplorationConfig(clearance_radius_m=1.1, reserve_s=5.0)
    plan = plan_frontier(
        free, set(), 1.0, bounds, current, current,
        remaining_flight_s=50.0, config=config,
    )
    assert plan is not None
    assert plan.frontier_key[0] == 6
    assert plan.goal_xyz[0] <= 4.5
    assert path_is_known_free(free, set(), 1.0, bounds, plan.local_path_xyz, config)


def test_explorer_rejects_unreachable_return_anchor():
    free = box(range(3), range(5), range(5)) | box(range(5, 8), range(5), range(5))
    bounds = ((0, 0, 0), (9, 4, 4))
    config = ExplorationConfig(clearance_radius_m=0.1, reserve_s=5.0)
    plan = plan_frontier(
        free, set(), 1.0, bounds, (1.5, 2.5, 2.5), (6.5, 2.5, 2.5),
        remaining_flight_s=100.0, config=config,
    )
    assert plan is None
    assert plan_known_free_path(
        free, set(), 1.0, bounds, (1.5, 2.5, 2.5), (6.5, 2.5, 2.5), config,
    ) == []


def test_explorer_skips_only_previously_reached_viewpoints():
    free = box(range(6), range(5), range(5))
    bounds = ((0, 0, 0), (8, 4, 4))
    current = (1.5, 2.5, 2.5)
    config = ExplorationConfig(clearance_radius_m=0.1, reserve_s=5.0)
    first = plan_frontier(
        free, set(), 1.0, bounds, current, current,
        remaining_flight_s=50.0, config=config,
    )
    assert first is not None
    reached_key = tuple(np.floor(first.goal_xyz).astype(int))
    second = plan_frontier(
        free, set(), 1.0, bounds, current, current,
        remaining_flight_s=50.0, visited_goal_keys={reached_key}, config=config,
    )
    assert second is not None
    assert tuple(np.floor(second.goal_xyz).astype(int)) != reached_key


def test_candidate_cap_does_not_displace_feasible_return_budget():
    free = box(range(6), range(5), range(5))
    bounds = ((0, 0, 0), (8, 4, 4))
    current = (1.5, 2.5, 2.5)
    config = ExplorationConfig(
        clearance_radius_m=0.1,
        sensor_range_m=3.5,
        reserve_s=1.0,
        max_candidates=1,
    )
    plan = plan_frontier(
        free, set(), 1.0, bounds, current, current,
        remaining_flight_s=5.0, config=config,
    )
    assert plan is not None
    assert plan.reserve_margin_s >= 0.0
    assert plan.outbound_time_s + plan.return_time_s + config.reserve_s <= 5.0


def test_visibility_ray_stops_at_first_unknown_voxel():
    origin = np.array([0.5, 0.5, 0.5])
    target = (3, 0, 0)
    assert not _visible(origin, target, {(0, 0, 0), (1, 0, 0)}, 1.0)
    assert _visible(origin, target, {(0, 0, 0), (1, 0, 0), (2, 0, 0)}, 1.0)


def test_view_gain_counts_first_unknown_on_slab_surface():
    free = {(x, 0, z) for x in range(6) for z in range(2, 5)}
    origin = np.array([0.5, 0.5, 3.5])
    distant_unknown = (5, 0, 1)
    first = _first_non_free(origin, distant_unknown, free, 1.0)
    assert first == (4, 0, 1)
    gain = _view_gain(
        origin, [distant_unknown], np.array([[5.5, 0.5, 1.5]]),
        np.array([1.0]), np.array([True]), {(4, 0, 1)}, free,
        1.0, ExplorationConfig(),
    )
    assert gain[0] == 1
    assert gain[3] == first
