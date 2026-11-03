"""Scenario loading, override precedence and suite expansion."""
from pathlib import Path

import numpy as np
import pytest
import yaml

from explore_grid_benchmark.scenarios import resolve_scenario, suite_jobs
from explore_grid_benchmark.world import load_world

SHARE = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('name', ['open', 'rooms', 'corridor', 'clutter'])
def test_presets_are_valid_connected_worlds(name):
    settings = resolve_scenario(name, SHARE)
    world = load_world(settings['map_file'])
    assert settings['scenario'] == name
    assert settings['scenario_file'].endswith(f'{name}.yaml')
    assert len(world.reachable) == np.count_nonzero(world.truth == 0)
    assert settings['sensor_range'] > 0


def test_external_scenario_relative_map_and_overrides(tmp_path):
    (tmp_path / 'map.csv').write_text('100,100,100\n100,0,100\n100,100,100\n')
    scene = tmp_path / 'custom.yaml'
    scene.write_text(yaml.safe_dump({'name': 'custom', 'map_file': 'map.csv',
                                    'simulation': {'sensor_range': 2.0, 'max_duration': 80}}))
    settings = resolve_scenario(str(scene), SHARE, {'sensor_range': '0.7', 'speed': '',
                                                   'map_file': '', 'start_x': '1', 'start_y': '1'})
    assert settings['map_file'] == str(tmp_path / 'map.csv')
    assert settings['sensor_range'] == 0.7
    assert settings['max_duration'] == 80.0
    assert settings['speed'] == 0.8
    assert settings['start_x'] == 1


def test_explicit_map_file_overrides_scenario(tmp_path):
    alternate = tmp_path / 'alternate.csv'
    alternate.write_text('100,100,100\n100,0,100\n100,100,100\n')
    settings = resolve_scenario('rooms', SHARE, {'map_file': str(alternate)})
    assert settings['map_file'] == str(alternate)


@pytest.mark.parametrize('config', [
    {'map_file': 'missing.csv'},
    {'name': '../escape', 'map_file': 'map.csv'},
    {'map_file': 'map.csv', 'simulation': {'sensor_ragne': 1}},
    {'map_file': 'map.csv', 'simulation': {'speed': True}},
    {'map_file': 'map.csv', 'simulation': {'start_x': 1.2}},
    {'map_file': 'map.csv', 'unexpected': 1},
    {'map_file': 'map.csv', 'simulation': {'speed': -1}},
    {'map_file': 'map.csv', 'simulation': {'sensor_range': float('nan')}},
    {'map_file': 'map.csv', 'simulation': {'coverage_target': 1.5}},
    {'map_file': 'map.csv', 'simulation': {'start_x': 2}},
])
def test_reject_bad_scenarios(tmp_path, config):
    (tmp_path / 'map.csv').write_text('0')
    scene = tmp_path / 'bad.yaml'
    scene.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError):
        resolve_scenario(str(scene), SHARE)


def test_unknown_preset_explains_choices():
    with pytest.raises(ValueError, match='presets:'):
        resolve_scenario('missing', SHARE)


def test_suite_runs_each_scenario_with_each_algorithm():
    assert suite_jobs('open, corridor', 'rrt,mrtsp') == [
        {'scenario': 'open', 'algorithm': 'rrt'},
        {'scenario': 'open', 'algorithm': 'mrtsp'},
        {'scenario': 'corridor', 'algorithm': 'rrt'},
        {'scenario': 'corridor', 'algorithm': 'mrtsp'},
    ]
    for scenes, algorithms in [('', 'mrtsp'), ('rooms,', 'rrt'), ('rooms', 'typo')]:
        with pytest.raises(ValueError):
            suite_jobs(scenes, algorithms)
