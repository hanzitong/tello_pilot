
from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.actions import RegisterEventHandler, Shutdown
# from launch.event_handlers import OnSignal

import os
from ament_index_python.packages import get_package_share_directory
pkg_share = get_package_share_directory('tello_pilot')

# udev シンボリックリンク /dev/tello_cam が指す実デバイス番号を取得する
# 例: /dev/tello_cam -> /dev/video5 → index = 5
_video_tello = '/dev/video_tello'
if os.path.exists(_video_tello):
    _actual_device = os.path.realpath(_video_tello)   # /dev/videoN
    cam_index = int(_actual_device.replace('/dev/video', ''))
else:
    cam_index = 4  # フォールバック: udev 未設定時
rviz_config = os.path.join(
    pkg_share,
    'config',
    'teleop_sse_config.rviz'
)

# import signal
# shutdown_handler = RegisterEventHandler(
#     OnSignal(signal=signal.SIGINT, on_signal=lambda *args, **kwargs: Shutdown())
# )



pc_cam_pixels = [1920, 1080]
usb_cam_pixels = [640, 480]
realsense_cam_pixels = [0, 0]

# 使用する解像度。GStreamer が MJPG に対応していない環境では usb_cam_pixels (640x480) を使う
cam_pixels = usb_cam_pixels


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='opencv_cam',
            executable='opencv_cam_main',
            name='opencv_cam',
            output='screen',
            parameters=[
                {'index': 0},    # /dev/tello_cam が指すデバイス番号
                # {'index': cam_index},    # /dev/tello_cam が指すデバイス番号
                {'width': cam_pixels[0]},
                {'height': cam_pixels[1]},
                {'fps': 25},            # YUYV 640x480 max fps
                {'camera_frame_id': 'camera_frame'},
            ],
            remappings=[('/image_raw', '/cam_image_raw')],
        ),
        Node(       # camera center frame: cam_pixels に合わせて自動計算
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', str(cam_pixels[0]/2/100),   # [100 * pixel]
                '--y', str(cam_pixels[1]/2/100),   # [100 * pixel]
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
        Node(
            package='tello_driver',
            executable='tello_driver_main',
            output='screen',
        ),
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
        Node(
            package='tello_pilot',
            executable='cmd_multiplexer',
            output='screen',
        ),
        Node(
            package='tello_pilot',
            executable='auto_lander',
            output='screen',
        ),


        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', rviz_config]
        # )
    ])


