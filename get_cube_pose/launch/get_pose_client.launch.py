import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare the launch argument
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time',default_value='True',
        description='Use simulation (Gazebo) clock if true')
    # Use the launch argument in the node configuration
    use_sim_time = LaunchConfiguration('use_sim_time')

    # MoveItCpp demo executable
    get_pose_client_node = Node(
        name="get_pose_client",
        package="get_cube_pose",
        executable="get_pose_client",
        output="screen",
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        get_pose_client_node,
    ])
