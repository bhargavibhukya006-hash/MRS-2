import launch
import launch_ros.actions

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
            executable='astar_global_planner_node',
            name='astar_global_planner',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='swarm_sim',
            executable='environment_visualizer_node',
            name='environment_visualizer',
            output='screen'
        ),
        # Example of using the DQN Local Planner instead of basic RL
        launch_ros.actions.Node(
            package='swarm_sim',
            executable='dqn_local_planner_node',
            name='dqn_planner',
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
        )
    ])
