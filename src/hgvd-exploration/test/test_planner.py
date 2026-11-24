import numpy as np

from hgvd.Bidirectional_a_star import bidirectional_astar
from hgvd.explore import Planner
from hgvd.gvd import sample_gvd
from hgvd.mean_shift3 import mean_shift
from hgvd.utils import Connectivity, denoise


def test_denoise_uses_rows_and_columns_and_preserves_unmodified_regions():
    grid = np.full((5, 12), -1, dtype=np.int8)
    grid[1:4, 7:10] = 0
    grid[1, 10] = 100
    clean = denoise(grid, None, None)
    assert clean[2, 10] == 0
    assert clean[1, 10] == 100
    assert grid[2, 10] == -1
    updated = grid.copy()
    updated[2, 1] = 100
    assert denoise(updated, grid, clean)[2, 10] == 0


def test_sample_units_near_rectangular_map_edge_are_clipped():
    grid = np.zeros((11, 24), dtype=np.int8)
    grid[[0, 10], :] = 100
    samples = sample_gvd(grid, grid, grid == 0, (1, 5), 25, 1, 4)
    assert any(s.cell[0] < s.radius for s in samples)
    assert all(grid[y, x] == 0 for (x, y) in (s.cell for s in samples))


def test_no_obstacles_still_explores_reachable_unknown_boundary():
    grid = np.full((6, 11), -1, dtype=np.int8)
    grid[:, :4] = 0
    grid[:, 8:] = 0  # Known free, but separated by unknown space.
    planner = Planner(5.0, 2, 8, 2.0, 0.0, 0.6)
    planner.update_map(grid, 0.2)
    assert planner.update_topology((1, 2))
    assert planner.samples == []
    targets = list(planner.viewpoints((1, 2), {}))
    assert targets and all(x == 3 for x, _ in targets)
    assert all(np.isfinite(score) for _, score in planner.scores((1, 2), 0.0, targets))


def test_direct_connection_swaps_whole_endpoints_and_checks_both():
    free = np.ones((5, 7), dtype=bool)
    free[1, 1] = False
    links = Connectivity(free)
    assert not links.direct((4, 4), (0, 0))
    assert not links.direct((0, 0), (4, 4))
    assert not links.direct((0, 1), (1, 1))
    assert links.direct((4, 4), (0, 4))


def test_mean_shift_deduplicates_nearby_modes_but_keeps_wall_separated_modes():
    traversable = np.ones((5, 10), dtype=bool)
    traversable[:, 4] = False
    samples = [(0, 2), (1, 2), (3, 2), (5, 1), (5, 3), (6, 2), (8, 2)]
    centers = mean_shift(samples, Connectivity(traversable), 10)
    assert len(centers) == 2
    assert sorted(x < 4 for x, _ in centers) == [False, True]


def test_grid_path_search_only_uses_known_free_cells():
    free = np.ones((6, 10), dtype=bool)
    free[:5, 4] = False
    links = Connectivity(free)
    assert bidirectional_astar(links, (1, 1), (7, 1)) == 14
    assert bidirectional_astar(links, (1, 1), (10, 1)) == float('inf')


def test_topology_recomputes_when_robot_moves_or_map_changes():
    grid = np.zeros((11, 24), dtype=np.int8)
    grid[[0, 10], :] = 100
    planner = Planner(1.0, 2, 8, 2.0, 0.0, 0.6)
    planner.update_map(grid, 0.2)
    assert planner.update_topology((2, 5))
    before = planner.sample_origin
    assert planner.update_topology((21, 5))
    assert planner.sample_origin != before
    updated = grid.copy()
    updated[:, 16] = 100
    planner.update_map(updated, 0.2)
    assert planner.update_topology((21, 5))
    assert not planner.links.contains((2, 5))
    assert all(s.cell[0] > 16 for s in planner.samples)



def test_goal_distance_filters_before_selecting_the_frontier_representative():
    grid = np.full((7, 10), -1, dtype=np.int8)
    grid[:, :4] = 0
    planner = Planner(5.0, 2, 8, 2.0, 0.0, 10.0)
    planner.update_map(grid, 0.2)
    robot = (2, 2)
    assert planner.update_topology(robot)
    assert list(planner.viewpoints(robot, {})) == [(3, 2)]

    pose = (2.4, 2.0)
    goals = list(planner.viewpoints(robot, {}, 0.25, robot_position=pose))
    assert len(goals) == 1
    assert goals[0][0] == 3
    assert abs(goals[0][1] - robot[1]) >= 2
