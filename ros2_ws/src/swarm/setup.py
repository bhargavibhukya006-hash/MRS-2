from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'swarm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Somid',
    maintainer_email='somid@todo.todo',
    description='Swarm simulation migrated to ROS 2 continuous space',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_manager_node = swarm_nodes.fleet_manager_node:main',
            'astar_global_planner_node = swarm_nodes.astar_global_planner_node:main',
            'robot_controller_node = swarm_nodes.robot_controller_node:main',
            'rl_local_planner_node = swarm_nodes.rl_local_planner_node:main',
            'dqn_local_planner_node = swarm_nodes.dqn_local_planner_node:main',
            'environment_visualizer_node = swarm_nodes.environment_visualizer_node:main',
            'replanner_node = swarm_nodes.replanner_node:main',
        ],
    },
)
