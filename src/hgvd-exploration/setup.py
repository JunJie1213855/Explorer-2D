from setuptools import setup

package_name = 'hgvd_exploration_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, 'hgvd'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', ['launch/explore.launch.py', 'launch/rviz.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/hgvd.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='littleBurgerrr',
    maintainer_email='343626720@qq.com',
    description='Hierarchical generalized Voronoi exploration for ROS 2 and Nav2',
    license='Unspecified in upstream source',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'hgvd_explorer = hgvd_exploration_ros2.explorer_node:main',
    ]},
)
