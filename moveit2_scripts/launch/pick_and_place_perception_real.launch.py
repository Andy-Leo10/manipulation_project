import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("my").to_moveit_configs()

    # Declare the launch argument
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time',default_value='False',
        description='Use simulation (Gazebo) clock if true')
    # Use the launch argument in the node configuration
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Perception action server for perception
    basic_grasping_perception_node = Node(
        name="a_action_server_perception_node_reaal",
        package='simple_grasping',
        executable='basic_grasping_perception_node_real',
        output='screen',
        parameters=[{'debug_topics': True}],
    )

    # MoveItCpp demo executable
    moveit_cpp_node = Node(
        name="a_pick_and_place_perception_real",
        package="moveit2_scripts",
        executable="pick_and_place_perception_real",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': use_sim_time},
            {'is_robot_sim': use_sim_time},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        basic_grasping_perception_node,
        LogInfo(msg='\n\n\n----------------------waiting a moment for PERCEPTION ACTION------------------------\n\n\n'),
        TimerAction(
            period=5.0,
            actions=[moveit_cpp_node,],
        )
    ])
