from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    joy_parmas = os.path.join(
        get_package_share_directory('autunomous_proto'),'config','joy_config.yaml'
    )

    joystick_node = Node(
        package='joy',
        executable='joy_node',
        name='joystick',
        parameters=[joy_parmas]
    )

    # convert joy to twist
    joy_twist_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name="teleop_node",
        parameters=[joy_parmas],
        remappings=[
            ('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')
        ]
    )

    return LaunchDescription([
        joystick_node,
        joy_twist_node
    ])