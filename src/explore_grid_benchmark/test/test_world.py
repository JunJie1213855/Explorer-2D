"""Checks for sensing occlusion, perceived-map planning and collision safety."""
import math

import numpy as np
import pytest

from explore_grid_benchmark.world import GridWorld, load_world


def room():
    grid = np.zeros((9, 11), dtype=int)
    grid[[0, -1], :] = 100
    grid[:, [0, -1]] = 100
    return grid


def test_sensor_does_not_reveal_behind_wall():
    grid = room()
    grid[:, 5] = 100
    world = GridWorld(grid, 1.0, [2, 4])
    world.observe(20)
    assert world.known[4, 5] == 100
    assert world.known[4, 6] == -1
    assert world.coverage == 1.0  # Isolated far room is excluded from denominator.


def test_planner_uses_observation_not_hidden_truth():
    grid = room()
    grid[:, 5] = 100
    world = GridWorld(grid, 1.0, [2, 4])
    world.observe(1.0)
    assert world.plan((7, 4)) is not None
    world.observe(20)
    assert world.plan((7, 4)) is None


def test_motion_cannot_tunnel_through_hidden_wall():
    grid = room()
    grid[:, 5] = 100
    world = GridWorld(grid, 1.0, [2, 4])
    assert not world.move_to(8.5, 4.5)
    assert world.x < 5
    assert world.collisions == 1
    assert world.known[4, 5] == 100
    assert world.distance == pytest.approx(world.x - 2.5)


def test_coordinate_conversion_and_coverage():
    world = GridWorld(room(), 0.2, [2, 4], [-2.0, 3.0])
    assert world.world_cell(*world.cell_center((3, 5))) == (3, 5)
    assert world.world_cell(-2.01, 3.0) == (-1, 0)
    assert world.coverage == 0
    world.observe(0.01)
    assert world.coverage == pytest.approx(1 / 63)


def test_inflation_only_uses_observed_walls():
    world = GridWorld(room(), 1.0, [4, 4])
    world.observe(1.0)
    assert not np.any(world.costmap(1.0) == 100)
    world.observe(20)
    assert world.costmap(1.0)[1, 4] == 100


@pytest.mark.parametrize('change', ['unknown', 'border', 'start', 'resolution'])
def test_reject_invalid_world(change):
    grid = room()
    start, resolution = [2, 4], 1.0
    if change == 'unknown':
        grid[3, 3] = -1
    elif change == 'border':
        grid[0, 3] = 0
    elif change == 'start':
        start = [0, 0]
    else:
        resolution = math.nan
    with pytest.raises(ValueError):
        GridWorld(grid, resolution, start)


def test_load_formats_and_metadata(tmp_path):
    grid = room()
    np.save(tmp_path / 'room.npy', grid)
    np.savetxt(tmp_path / 'room.csv', grid, delimiter=',', fmt='%d')
    (tmp_path / 'room.yaml').write_text(
        'resolution: 0.5\nstart: [2, 3]\ngrid: ' + str(grid.tolist()))
    for suffix in ('npy', 'csv', 'yaml'):
        world = load_world(tmp_path / ('room.' + suffix), 0.25, [3, 4])
        assert world.resolution == 0.25
        assert world.start == (3, 4)
    assert load_world(tmp_path / 'room.yaml').resolution == 0.5
