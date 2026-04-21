
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import math
from ament_index_python.packages import get_package_share_directory

pkg_share = get_package_share_directory('tello_pilot')
rviz_config = os.path.join(pkg_share, 'config', 'tello_test.rviz')


# pc_cam_pixels = {
#     'width': 1920,
#     'height': 1080
# }
usb_cam_pixels = {
    'width': 640,
    'height': 480
}
calibration_file_path = os.path.join(
    get_package_share_directory('tello_pilot'),
    'config',
    'ost.yaml'
)


def generate_launch_description():

    usb_cam = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam_node',
        output='screen',
        parameters=[
            {
                'video_device': '/dev/video_tello', # it may sometimes be changed automatically
                'image_width': usb_cam_pixels['width'],
                'image_height': usb_cam_pixels['height'],
                'camera_frame_id': 'camera_frame',
                'camera_info_url': f'file://{calibration_file_path}',

                # ===== Use YUYV
                'pixel_format': 'yuyv',
                'framerate': 25.0,

                # ===== Use MJPEG
                # 'pixel_format': 'mjpeg2rgb',
                # 'framerate': 30.0,  # 30fps max
            }
        ],
        remappings=[
            ('/image_raw', '/camera/image_raw'),
        ],
        respawn=True,
        # respawn_delay=0.2,
    )

    camera_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'world',
            '--child-frame-id', 'camera_frame'
        ]
    )

    # drone_frame: marker_23_frame を X 軸回りに 180° 回転するとドローン座標系になる
    # ARマーカーの Z 軸（カメラ方向）が反転し、ドローンの Z 軸（上方向）に対応する
    drone_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', str(math.pi),   # Rx(180°): marker Z (下向き) → drone Z (上向き)
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'marker_23_frame',
            '--child-frame-id', 'drone_frame'
        ]
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        output='screen',
    )

    tello_node_list = [
        Node(
            package='tello_driver',
            executable='tello_driver_main',
            output='screen',
        ),
        Node(
            package='tello_pilot',
            executable='ar_detector_node_2',
            name='ar_detector_node_2',
            output='screen',
            remappings=[
                ('/cam_image_raw', '/camera/image_raw')
            ]
        ),
        Node(
            package='tello_pilot',
            executable='cmd_multiplexer_node',
            output='screen',
        ),
        Node(
            package='tello_pilot',
            executable='pid_controller_node',
            output='screen',
        ),
        Node(
            package='tello_pilot',
            executable='auto_lander_node',
            output='screen',
        ),
        Node(
            package='tello_pilot',
            executable='cmd_vel_visualizer_node',
            output='screen',
        ),
    ]

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )


    return LaunchDescription([usb_cam] + [joy_node] + [camera_frame, drone_frame] + tello_node_list + [rviz])
