import launch
import launch_ros.actions

# Just launches several nodes to start the game.
def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='turtle_hunter',
            executable='game_processing_node',
            name='game_processing_node'),
        launch_ros.actions.Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim_node'),
        launch_ros.actions.Node(
            package='turtle_hunter',
            executable='hunter_node',
            name='hunter_node'),
    ])
