from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    ld = LaunchDescription()

    # Launch arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')
    )

    joy_node_2 = Node(
        package = "joy",
        executable = "joy_node_2",
        respawn=True,
        respawn_delay=1,
    )

    velocity_pub = Node(
        package ='velocity_pub',
        executable='robot_control.py',
        output='screen',
        respawn=True,  
        respawn_delay=1,
    )

    ld.add_action(joy_node_2)
    ld.add_action(velocity_pub)

    return ld