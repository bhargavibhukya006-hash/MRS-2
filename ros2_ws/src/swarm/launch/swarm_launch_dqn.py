import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='swarm',
            executable='fleet_manager_node',
            name='fleet_manager',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='swarm',
            executable='astar_global_planner_node',
            name='astar_global_planner',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='swarm',
            executable='environment_visualizer_node',
            name='environment_visualizer',
            output='screen'
        ),
        # Launch multiple robot controllers WITH their dedicated DQN Local Planners
        launch_ros.actions.Node(
            package='swarm',
            executable='robot_controller_node',
            name='robot_0_controller',
            namespace='robot_0',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='swarm',
            executable='dqn_local_planner_node',
            name='robot_0_dqn',
            namespace='robot_0',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='swarm',
            executable='robot_controller_node',
            name='robot_1_controller',
            namespace='robot_1',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='swarm',
            executable='dqn_local_planner_node',
            name='robot_1_dqn',
            namespace='robot_1',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='swarm',
            executable='robot_controller_node',
            name='robot_2_controller',
            namespace='robot_2',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='swarm',
            executable='dqn_local_planner_node',
            name='robot_2_dqn',
            namespace='robot_2',
            output='screen'
        )
    ])
