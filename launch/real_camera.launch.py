from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():

        # arugment for the launch file to turn the image viwer on or not
    image_path_arg = DeclareLaunchArgument(
        'camera_path',
        default_value='/dev/video0',
        description='Path to the video device'
    )

    image_viewer_arg = DeclareLaunchArgument(
        'image_viewer',
        default_value='true',
        description='Turn on the image viewer or off'
    )
    
    image_viewer = LaunchConfiguration('image_viewer')
    # Retrieve the value of the image_viewer argument
    camera_path = LaunchConfiguration('camera_path')
    # Launch the camera node
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'camera_frame_id': 'camera_link_optical',
            'camera_info_url': 'http://localhost:8080/camera_info',
            'video_device': camera_path
        }]
    )

    # Launch the image viewer node
    image_viewer_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }],
        condition=IfCondition(image_viewer)
    )

    # Launch the image_viewer node

    return LaunchDescription([
        image_path_arg,
        camera_node,
        image_viewer_arg,
        image_viewer_node
        
    ])