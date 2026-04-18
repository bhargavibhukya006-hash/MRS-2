from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'swarm_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='ROS 2 package for swarm simulation, transitioning from Pygame.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_manager_node = swarm_sim.fleet_manager_node:main',
            'astar_global_planner_node = swarm_sim.astar_global_planner_node:main',
            'robot_controller_node = swarm_sim.robot_controller_node:main',
            'rl_local_planner_node = swarm_sim.rl_local_planner_node:main',
            'dqn_local_planner_node = swarm_sim.dqn_local_planner_node:main',
            'environment_visualizer_node = swarm_sim.environment_visualizer_node:main'
        ],
    },
)
