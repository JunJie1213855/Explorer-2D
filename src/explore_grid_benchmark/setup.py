from glob import glob
from setuptools import find_packages, setup

name = 'explore_grid_benchmark'
setup(
    name=name, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml', 'README.md', 'LICENSE']),
        ('share/' + name + '/launch', glob('launch/*.launch.py')),
        ('share/' + name + '/maps', glob('maps/*.yaml')),
        ('share/' + name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Explorer-2D maintainers', maintainer_email='maintainer@example.com',
    description='Independent 2D matrix exploration benchmark', license='Apache-2.0',
    entry_points={'console_scripts': ['grid_simulator = explore_grid_benchmark.node:main']},
)
