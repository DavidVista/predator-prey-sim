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
        
        # Start our turtle manager (spawns 3 prey + 1 predator)
        Node(
            package='turtle_spawner',
            executable='turtle_manager',
            name='turtle_manager',
            output='screen'
        ),
        
        # Start Multi-Prey Boids controllers for all 3 prey
        Node(
            package='turtle_spawner',
            executable='multi_prey_boids_controller',
            name='multi_prey_boids_controller_1',
            output='screen',
            parameters=[{'prey_name': 'prey_turtle_1'}]
        ),
        
        Node(
            package='turtle_spawner',
            executable='multi_prey_boids_controller',
            name='multi_prey_boids_controller_2',
            output='screen',
            parameters=[{'prey_name': 'prey_turtle_2'}]
        ),
        
        Node(
            package='turtle_spawner',
            executable='multi_prey_boids_controller',
            name='multi_prey_boids_controller_3',
            output='screen',
            parameters=[{'prey_name': 'prey_turtle_3'}]
        ),
        
        # Start energy-efficient predator controller (multi-target)
        Node(
            package='turtle_spawner',
            executable='energy_efficient_predator_controller',
            name='energy_efficient_predator_controller',
            output='screen'
        )
    ]) 