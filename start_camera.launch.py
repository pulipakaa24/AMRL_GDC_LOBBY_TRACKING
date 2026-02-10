import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer

#from perspective of cameras
LEFT_CAMERA_NAME = 'left'
RIGHT_CAMERA_NAME = 'right'
LEFT_CAMERA_NAMESPACE = 'stereo'
RIGHT_CAMERA_NAMESPACE = 'stereo'

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('spinnaker_camera_driver'),
        'config',
        'blackfly_s.yaml'
    )

    return LaunchDescription([
        ComposableNodeContainer(
            name='vision_container',
            namespace='',
            
        )
        Node(
            package='spinnaker_camera_driver',
            executable='camera_driver_node',
            name=LEFT_CAMERA_NAME,
            namespace=LEFT_CAMERA_NAMESPACE,
            parameters=[{
                'parameter_file': config_path,
                'serial_number': '25282106',
                'camera_info_url': 'file:///home/sentry/camera_ws/stereoCal/left.yaml'
            }]
        ),

        Node(
            package='spinnaker_camera_driver',
            executable='camera_driver_node',
            name=RIGHT_CAMERA_NAME,
            namespace=RIGHT_CAMERA_NAMESPACE,
            parameters=[{
                'parameter_file': config_path,
                'serial_number': '25235293',
                'camera_info_url': 'file:///home/sentry/camera_ws/stereoCal/right.yaml'
            }]
        ),

        # Node(
        #     package='stereo_image_proc',
        #     executable='point_cloud_node',
        #     namespace='stereo',
        #     parameters=[{'approximate_sync': True}],
        #     remappings=[
        #         ('left/image_rect_color', 'left/image_rect'),
        #         ('right/image_rect_color', 'right/image_rect')
        #     ],
        #     output='screen'
        # )

    ])