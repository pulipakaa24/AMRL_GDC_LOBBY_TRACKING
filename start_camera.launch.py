import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

# From perspective of cameras
LEFT_CAMERA_NAME = 'left'
RIGHT_CAMERA_NAME = 'right'


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('spinnaker_camera_driver'),
        'config',
        'blackfly_s.yaml'
    )

    # ── Container 1: Camera Drivers (component_container_mt) ─────────────
    #
    # Both cameras share a single process because the FLIR Spinnaker SDK
    # requires USB3 cameras to negotiate bandwidth within one process.
    # Using _mt so both capture callbacks can fire on separate threads.
    #
    # Topics published (per camera):
    #   /stereo/{left,right}/image_raw    (sensor_msgs/Image)
    #   /stereo/{left,right}/camera_info  (sensor_msgs/CameraInfo)
    camera_container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # ── Left Blackfly S (serial 25282106) ──
            # namespace='stereo', name='left' → node name creates sub-namespace
            # publishes: /stereo/left/image_raw, /stereo/left/camera_info
            ComposableNode(
                package='spinnaker_camera_driver',
                plugin='spinnaker_camera_driver::CameraDriver',
                name=LEFT_CAMERA_NAME,
                namespace='stereo',
                parameters=[{
                    'parameter_file': config_path,
                    'serial_number': '25282106',
                    'camera_info_url': 'file:///home/sentry/camera_ws/stereoCal/left.yaml',
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            # ── Right Blackfly S (serial 25235293) ──
            # namespace='stereo', name='right' → node name creates sub-namespace
            # publishes: /stereo/right/image_raw, /stereo/right/camera_info
            ComposableNode(
                package='spinnaker_camera_driver',
                plugin='spinnaker_camera_driver::CameraDriver',
                name=RIGHT_CAMERA_NAME,
                namespace='stereo',
                parameters=[{
                    'parameter_file': config_path,
                    'serial_number': '25235293',
                    'camera_info_url': 'file:///home/sentry/camera_ws/stereoCal/right.yaml',
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    # ── Container 2: Image Processing (component_container_mt) ───────────
    #
    # Debayer + Rectify for both cameras. Grouped together so that
    # debayer→rectify is zero-copy (intra-process) per side, while _mt
    # lets left and right pipelines run concurrently on different threads.
    #
    # Per-side data flow:
    #   image_raw → DebayerNode → image_mono → RectifyNode → image_rect
    #                           ↘ image_color (available for other consumers)
    image_proc_container = ComposableNodeContainer(
        name='image_proc_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # ── Left Debayer ──
            # Subscribes: /stereo/left/image_raw   (default topic, matches camera)
            # Publishes:  /stereo/left/image_mono   (grayscale debayered)
            #             /stereo/left/image_color  (color debayered)
            ComposableNode(
                package='image_proc',
                plugin='image_proc::DebayerNode',
                name='debayer',
                namespace='stereo/left',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            # ── Left Rectify ──
            # Remap: 'image' → 'image_mono' so it subscribes to debayer output.
            # image_transport auto-derives camera_info from the image topic's
            # namespace: /stereo/left/image_mono → /stereo/left/camera_info ✓
            # Publishes: /stereo/left/image_rect
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_mono',
                namespace='stereo/left',
                remappings=[('image', 'image_mono')],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            # ── Right Debayer ──
            # Subscribes: /stereo/right/image_raw
            # Publishes:  /stereo/right/image_mono, /stereo/right/image_color
            ComposableNode(
                package='image_proc',
                plugin='image_proc::DebayerNode',
                name='debayer',
                namespace='stereo/right',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            # ── Right Rectify ──
            # Remap: 'image' → 'image_mono'
            # Subscribes: /stereo/right/image_mono + /stereo/right/camera_info
            # Publishes:  /stereo/right/image_rect
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_mono',
                namespace='stereo/right',
                remappings=[('image', 'image_mono')],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    # ── Container 3: Stereo Processing (component_container_mt) ──────────
    #
    # DisparityNode + PointCloudNode isolated from image processing so the
    # heavy stereo matching doesn't starve debayer/rectify callbacks.
    # Zero-copy between disparity → point_cloud within this container.
    #
    # Data flow:
    #   left/image_rect + right/image_rect → DisparityNode → disparity
    #   disparity + left/image_rect → PointCloudNode → points2
    stereo_proc_container = ComposableNodeContainer(
        name='stereo_proc_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # ── Disparity ──
            # Subscribes: /stereo/left/image_rect   (from left RectifyNode)
            #             /stereo/left/camera_info   (from left CameraDriver)
            #             /stereo/right/image_rect  (from right RectifyNode)
            #             /stereo/right/camera_info  (from right CameraDriver)
            # Publishes:  /stereo/disparity          (stereo_msgs/DisparityImage)
            ComposableNode(
                package='stereo_image_proc',
                plugin='stereo_image_proc::DisparityNode',
                name='disparity_node',
                namespace='stereo',
                parameters=[{'approximate_sync': True}],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            # ── Point Cloud ──
            # Remap: left/image_rect_color → left/image_rect because
            #        RectifyNode publishes 'image_rect', not 'image_rect_color'.
            # Subscribes: /stereo/left/image_rect   (remapped from left/image_rect_color)
            #             /stereo/left/camera_info
            #             /stereo/right/camera_info
            #             /stereo/disparity          (from DisparityNode — zero-copy)
            # Publishes:  /stereo/points2            (sensor_msgs/PointCloud2)
            ComposableNode(
                package='stereo_image_proc',
                plugin='stereo_image_proc::PointCloudNode',
                name='point_cloud_node',
                namespace='stereo',
                parameters=[{'approximate_sync': True}],
                remappings=[
                    ('left/image_rect_color', 'left/image_rect'),
                    ('right/image_rect_color', 'right/image_rect'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    # ── Static TF: world → camera_link ──────────────────────────────────
    #
    # Provides a fixed frame for RViz. Identity transform places
    # camera_link at the world origin. The stereo calibration YAMLs
    # handle the actual left↔right extrinsics.
    tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'world',
            '--child-frame-id', 'camera_link',
        ],
    )

    return LaunchDescription([
        camera_container,
        image_proc_container,
        stereo_proc_container,
        tf_publisher,
    ])
