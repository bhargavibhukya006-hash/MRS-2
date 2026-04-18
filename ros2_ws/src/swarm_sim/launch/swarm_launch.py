import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='swarm_sim',
            executable='fleet_manager_node',
            name='fleet_manager',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='swarm_sim',
            executable='prm_global_planner_node',
            name='prm_global_planner',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='swarm_sim',
            executable='environment_visualizer_node',
            name='environment_visualizer',
            output='screen'
        ),
        # Launch multiple robot controllers
        launch_ros.actions.Node(
            package='swarm_sim',
            executable='robot_controller_node',
            name='robot_0',
            namespace='robot_0',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='swarm_sim',
            executable='robot_controller_node',
            name='robot_1',
            namespace='robot_1',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='swarm_sim',
            executable='robot_controller_node',
            name='robot_2',
            namespace='robot_2',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='swarm_sim',
            executable='robot_controller_node',
            name='robot_3',
            namespace='robot_3',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='swarm_sim',
            executable='robot_controller_node',
            name='robot_4',
            namespace='robot_4',
            output='screen'
        )
    ])
