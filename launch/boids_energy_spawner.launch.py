from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Start turtlesim
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),
        
        # Start our turtle manager
        Node(
            package='turtle_spawner',
            executable='turtle_manager',
            name='turtle_manager',
            output='screen'
        ),
        
        # Start Boids prey controller (intelligent flocking behavior)
        Node(
            package='turtle_spawner',
            executable='boids_prey_controller',
            name='boids_prey_controller',
            output='screen'
        ),
        
        # Start energy predator controller (cheetah mode)
        Node(
            package='turtle_spawner',
            executable='energy_predator_controller',
            name='energy_predator_controller',
            output='screen'
        )
    ]) 