
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
pkg_share = get_package_share_directory('tello_pilot')
# rviz_cinfig = os.path.join(
#     pkg_share,
#     'config',
#     'teleop_sse_config.rviz'
# )


def generate_launch_description():
    node_list = [
        Node(
            package='opencv_cam',
            executable='opencv_cam_main',
            name='opencv_cam',
            output='screen',
            parameters=[
                {'index': 0},
                {'image_width': 1920},
                {'image_height': 1080},
                {'framerate': 30},
                {'camera_frame_id': 'camera_cv_frame'},
            ],
            remappings=[('/image_raw', '/cam_image_raw')],
        ),
        # Node(
        #     package='opencv_cam',
        #     executable='opencv_cam_main',
        #     name='opencv_cam',
        #     output='screen',
        #     parameters=[
        #         {'index': 4},
        #         {'image_width': 640},
        #         {'image_height': 480},
        #         {'framerate': 25},
        #         {'camera_frame_id': 'camera_cv_frame'},
        #     ],
        #     remappings=[('/image_raw', '/cam_image_raw')],
        # ),

        Node(
            package='tello_pilot',
            executable='ar_detector',
            output='screen',
        ),
        Node(
            package='tello_pilot',
            executable='pid_controller',
            output='screen',
        ),

        Node(       # PC camera center frame [pixel] (1920x1080)
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '9.6',   # [100 * pixel]
                '--y', '5.4',   # [100 * pixel]
                '--z', '0',
                '--yaw', '0',
                '--pitch', '0',
                '--roll', '0',
                '--frame-id', 'camera_cv_frame',
                '--child-frame-id', 'camera_center_frame'
            ]
        ),
        Node(
            package='tello_pilot',
            executable='pid_controller',
            output='screen',
        ),
        Node(
            package='joy',
            executable='joy_node',
            output='screen',
        ),
        Node(
            package='tello_pilot',
            executable='cmd_multiplexer',
            output='screen',
        ),

        # Node(
        #     package='tello_driver',
        #     executable='tello_driver_main',
        #     output='screen',
        # ),

    ]


    return LaunchDescription(node_list)





