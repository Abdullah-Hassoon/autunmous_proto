from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():

    package_name = 'autunomous_proto'
    # arugment for the launch file to turn the image viwer on or not
    image_viewer_arg = DeclareLaunchArgument(
        'image_viewer',
        default_value='false',
        description='Turn on the image viewer'
    )
    
    # Retrieve the value of the image_viewer argument
    image_viewer = LaunchConfiguration('image_viewer')

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
        get_package_share_directory(package_name), 'worlds', 'world.sdf'
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

    compressed_image_node = Node(
        package='image_transport',
        executable='republish',
        arguments=['raw', 'compressed'],
        remappings=[
            ("in", "/camera/image_raw"),
            ("out/compressed", "/camera/image_raw/compressed")
        ],
        output='screen'
    )

    
    image_viewer_node = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        condition=IfCondition(image_viewer)
    )
  


    return LaunchDescription([
        image_viewer_arg,
        rsp_launch,
        gazebo_launch,
        spawn_node,
        ros_gz_bridge_node,
        compressed_image_node,
        image_viewer_node
    ])