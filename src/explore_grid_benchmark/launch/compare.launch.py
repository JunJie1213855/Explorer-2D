"""Run four explorers sequentially in one launch service with fresh simulators."""
from pathlib import Path
import runpy


def generate_launch_description():
    benchmark = runpy.run_path(str(Path(__file__).with_name('benchmark.launch.py')))
    return benchmark['generate_launch_description'](
        sequence=['frontier', 'rrt', 'explore_lite', 'mrtsp'])
