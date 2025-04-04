import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch configuration variables
    width = LaunchConfiguration('width', default='10')
    height = LaunchConfiguration('height', default='10')
    start_x = LaunchConfiguration('start_x', default='0')
    start_y = LaunchConfiguration('start_y', default='0')
    goal_x = LaunchConfiguration('goal_x', default='9')
    goal_y = LaunchConfiguration('goal_y', default='9')
    obstacle_density = LaunchConfiguration('obstacle_density', default='0.1')

    return LaunchDescription([
        DeclareLaunchArgument('width', default_value='10', description='Grid width'),
        DeclareLaunchArgument('height', default_value='10', description='Grid height'),
        DeclareLaunchArgument('start_x', default_value='0', description='Starting x position of the robot'),
        DeclareLaunchArgument('start_y', default_value='0', description='Starting y position of the robot'),
        DeclareLaunchArgument('goal_x', default_value='9', description='Goal x position'),
        DeclareLaunchArgument('goal_y', default_value='9', description='Goal y position'),
        DeclareLaunchArgument('obstacle_density', default_value='0.1', description='Density of obstacles in the grid'),

        # Grid Environment Node
        Node(
            package='gritt_robotics',
            executable='grid_environment',
            name='grid_environment',
            parameters=[{
                'width': width,
                'height': height,
                'start_x': start_x,
                'start_y': start_y,
                'goal_x': goal_x,
                'goal_y': goal_y,
                'obstacle_density': obstacle_density
            }]
        ),

        # Dijkstra Planner Node
        Node(
            package='gritt_robotics',
            executable='dijkstra_planner',
            name='dijkstra_planner',
            parameters=[{
                'width': width,
                'height': height,
                'goal_x': goal_x,
                'goal_y': goal_y,
                'obstacle_density': obstacle_density
            }]
        ),
    ])
