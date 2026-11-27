from glob import glob

from setuptools import find_packages, setup

name = 'explore_nav_bridge'
setup(
    name=name, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml', 'README.md']),
        ('share/' + name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Explorer-2D maintainers', maintainer_email='maintainer@example.com',
    description='NavigateToPose action server bridging to topic-based planners',
    license='Apache-2.0',
    entry_points={
        'console_scripts': ['navigate_bridge = explore_nav_bridge.bridge_node:main'],
    },
)
