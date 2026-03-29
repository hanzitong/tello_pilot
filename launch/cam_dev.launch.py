
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
pkg_share = get_package_share_directory('tello_pilot')

_video_tello = '/dev/video_tello'
if os.path.exists(_video_tello):
    _actual_device = os.path.realpath(_video_tello)
    cam_index = int(_actual_device.replace('/dev/video', ''))
else:
    cam_index = 4  # フォールバック: udev 未設定時
# rviz_cinfig = os.path.join(
#     pkg_share,
#     'config',
#     'teleop_sse_config.rviz'
# )


pc_cam_pixels = [1920, 1080]
usb_cam_pixels = [640, 480]
realsense_cam_pixels = [0, 0]


def generate_launch_description():
    node_list = [
        Node(
            package='opencv_cam',
            executable='opencv_cam_main',
            name='opencv_cam',
            output='screen',
            parameters=[
                {'index': cam_index},    # /dev/tello_cam が指すデバイス番号
                {'width': pc_cam_pixels[0]},
                {'height': pc_cam_pixels[1]},
                {'fps': 30},
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
        Node(       # PC camera center frame [pixel] (1920x1080)
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                # '--x', '9.6',   # [100 * pixel]
                # '--y', '5.4',   # [100 * pixel]
                '--x', str(pc_cam_pixels[0]/2/100),   # [100 * pixel]
                '--y', str(pc_cam_pixels[1]/2/100),   # [100 * pixel]
                '--z', '0',
                '--yaw', '0',
                '--pitch', '0',
                '--roll', '0',
                '--frame-id', 'camera_cv_frame',
                '--child-frame-id', 'camera_center_frame'
            ]
        ),
        
        Node(
            package='joy',
            executable='joy_node',
            output='screen',
        ),
        # Node(
        #     package='tello_driver',
        #     executable='tello_driver_main',
        #     output='screen',
        # ),

        Node(
            package='tello_pilot',
            executable='ar_detector',
            output='screen',
        ),
        Node(
            package='tello_pilot',
            executable='cmd_multiplexer',
            output='screen',
        ),
        Node(
            package='tello_pilot',
            executable='pid_controller',
            output='screen',
        ),
        Node(
            package='tello_pilot',
            executable='auto_lander',
            output='screen',
        ),


    ]


    return LaunchDescription(node_list)





