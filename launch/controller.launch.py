from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # create a new node for each controller
    diff_drive_controller_spawn = Node(
        package="controller_manager",
        executable="spawner",               # spawn the controller
        arguments=[
            "diff_drive_controller", 
            "--controller-manager",
            "/controller_manager"
        ]
    )

    joint_state_broadcaster_spawn = Node(
        package="controller_manager",
        executable="spawner",               # spawn the controller
        arguments=[
            "joint_state_broadcaster", 
            "--controller-manager",
            "/controller_manager"
        ]
        )
    return LaunchDescription([
        diff_drive_controller_spawn,
        joint_state_broadcaster_spawn
    ])