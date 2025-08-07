from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    
    # Declare launch arguments
    use_yasmin_viewer_arg = DeclareLaunchArgument(
        'use_yasmin_viewer',
        default_value='true',
        description='Whether to launch yasmin viewer'
    )
    
    # Get launch configurations
    use_yasmin_viewer = LaunchConfiguration('use_yasmin_viewer')
    
    # Yasmin waypoint navigator node
    yasmin_waypoint_nav_node = Node(
        package='yasmin_waypoint_nav',
        executable='yasmin_waypoint_navigator',
        name='yasmin_waypoint_navigator',
        output='screen',
        parameters=[],
    )
    
    # YASMIN viewer node (conditional)
    yasmin_viewer_node = Node(
        package='yasmin_viewer',
        executable='yasmin_viewer_node',
        name='yasmin_viewer',
        output='screen',
        condition=IfCondition(use_yasmin_viewer)
    )
    
    return LaunchDescription([
        use_yasmin_viewer_arg,
        yasmin_waypoint_nav_node,
        yasmin_viewer_node,
    ])
