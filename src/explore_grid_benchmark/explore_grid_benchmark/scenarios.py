"""Resolve reusable scenarios without ROS dependencies.

Precedence: explicit launch arguments > scenario simulation settings > defaults.
"""
from pathlib import Path
import math
import re

import yaml

DEFAULTS = {
    'resolution': 0.0, 'start_x': -1, 'start_y': -1,
    'sensor_range': 1.5, 'speed': 0.8, 'angular_speed': 1.5,
    'inflation_radius': 0.0, 'max_duration': 180.0,
    'stall_timeout': 45.0, 'coverage_target': 0.98,
}
ALGORITHMS = ('frontier', 'rrt', 'explore_lite', 'mrtsp')


def resolve_scenario(selection, share, overrides=None):
    share = Path(share)
    path = Path(selection).expanduser()
    if path.suffix.lower() not in ('.yaml', '.yml'):
        if not re.fullmatch(r'[A-Za-z0-9_-]+', selection):
            raise ValueError('scenario must be a preset name or YAML path')
        path = share / 'scenarios' / f'{selection}.yaml'
    path = path.resolve()
    if not path.is_file():
        available = ', '.join(p.stem for p in sorted((share / 'scenarios').glob('*.yaml')))
        raise ValueError(f'Scenario does not exist: {path}; presets: {available}')
    with path.open() as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError('Scenario YAML must be a mapping')
    unknown = set(config) - {'name', 'description', 'map_file', 'simulation'}
    if unknown:
        raise ValueError(f'Unknown scenario keys: {sorted(unknown)}')
    name = config.get('name', path.stem)
    if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z0-9_-]+', name):
        raise ValueError('Scenario name must contain only letters, digits, underscores or hyphens')
    simulation = config.get('simulation', {})
    if not isinstance(simulation, dict):
        raise ValueError('simulation must be a parameter mapping')
    if set(simulation) - set(DEFAULTS):
        raise ValueError(f'Unknown simulation parameters: {sorted(set(simulation) - set(DEFAULTS))}')
    map_file = config.get('map_file')
    if not isinstance(map_file, str) or not map_file:
        raise ValueError('Scenario must specify map_file')
    map_path = Path(map_file).expanduser()
    if not map_path.is_absolute():
        map_path = path.parent / map_path
    settings = {**DEFAULTS, **simulation, 'map_file': str(map_path.resolve())}
    for key, value in (overrides or {}).items():
        if key not in settings:
            raise ValueError(f'Unknown override: {key}')
        if value is not None and value != '':
            settings[key] = value
    settings['map_file'] = str(Path(settings['map_file']).expanduser().resolve())
    if not Path(settings['map_file']).is_file():
        raise ValueError(f"Map does not exist: {settings['map_file']}")
    for key in DEFAULTS:
        value = settings[key]
        if isinstance(value, bool):
            raise ValueError(f'{key} must be numeric, not boolean')
        if key in ('start_x', 'start_y'):
            if not float(value).is_integer():
                raise ValueError(f'{key} must be an integer')
            settings[key] = int(value)
        else:
            settings[key] = float(value)
        if not math.isfinite(settings[key]):
            raise ValueError(f'{key} must be finite')
    for key in ('sensor_range', 'speed', 'angular_speed', 'max_duration', 'stall_timeout'):
        if settings[key] <= 0:
            raise ValueError(f'{key} must be positive')
    if settings['resolution'] < 0 or settings['inflation_radius'] < 0:
        raise ValueError('resolution and inflation_radius must be nonnegative')
    if not 0 < settings['coverage_target'] <= 1:
        raise ValueError('coverage_target must be in (0, 1]')
    if (settings['start_x'] < 0) != (settings['start_y'] < 0):
        raise ValueError('start_x and start_y must be specified together')
    return {**settings, 'scenario': name, 'scenario_file': str(path)}


def suite_jobs(scenarios, algorithms):
    selections = [s.strip() for s in scenarios.split(',')]
    methods = [a.strip() for a in algorithms.split(',')]
    if not all(selections) or not all(methods):
        raise ValueError('scenarios and algorithms must be nonempty comma-separated lists')
    if any(a not in ALGORITHMS for a in methods):
        raise ValueError(f'algorithms must be selected from {ALGORITHMS}')
    return [{'scenario': scene, 'algorithm': algorithm}
            for scene in selections for algorithm in methods]
