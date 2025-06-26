
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
                # {'index': 4},
                {'image_width': 640},
                {'image_height': 480},
                {'framerate': 25},
                {'camera_frame_id': 'camera_frame'},
            ],
            remappings=[('/image_raw', '/cam_image_raw')],
        ),
        Node(
            package='tello_pilot',
            executable='ar_detector_tf2',
            output='screen',
        ),
        Node(
            package='tello_pilot',
            executable='auto_centerizer',
            output='screen',
        ),
    ]


    return LaunchDescription(node_list)





