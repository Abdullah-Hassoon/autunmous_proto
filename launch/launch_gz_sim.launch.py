from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package_name = 'autunomous_proto'
    # Launch the robot_state_publisher node
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare(package_name), '/launch/rsp.launch.py'
        ]), launch_arguments= {'use_sim_time': 'true'}.items()

    )

    gazebo_resource_path = SetEnvironmentVariable(
        "IGN_SIM_RESOURCE_PATH",
        value=[
            str(Path(get_package_share_directory("autunomous_proto")).parent.resolve())
        ]
        )
    # load the my_world.sdf world file
    world_path = os.path.join(
        get_package_share_directory(package_name), 'worlds', 'my_world.world'
    )

    # Launch the Gazebo simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py'
        ]),    
         launch_arguments=[
                ("gz_args", [" -v 4", " -r", f" {world_path}"])
         ]
    )

    #Spawn the robot in Gazebo 
    spawn_node = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "autunomous_vechile",
                   '-z', '0.5']
    )

    # load the bridge parameter
    bridge_params = os.path.join(get_package_share_directory(package_name), 'config', 'gz_bridge.yaml')
    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}'
        ]
    )

    # # for image data run the following command
    # ros_gz_image_bridge_node = Node(
    #     package='ros_gz_bridge',
    #     executable='image_bridge',
    #     arguments=["/camera/image_raw"],
    # )

    return LaunchDescription([
        rsp_launch,
        gazebo_launch,
        spawn_node,
        ros_gz_bridge_node
    ])