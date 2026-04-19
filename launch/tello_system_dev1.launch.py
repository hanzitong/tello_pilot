
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


realsense_cam_pixels = [-1, -1]
pc_cam_pixels = {
    'width': 1920,
    'height': 1080
}
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
                # 'camera_info_url': 'file:///home/han_zitong/ws_tello/install/tello_pilot/share/tello_pilot/config/ost.yaml',

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

    tf_node = Node(       # PC camera center frame [pixel] (1920x1080)
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            # '--x', '9.6',   # [100 * pixel]
            # '--y', '5.4',   # [100 * pixel]
            '--x', str(usb_cam_pixels['width']/2/100),   # [100 * pixel]
            '--y', str(usb_cam_pixels['height']/2/100),   # [100 * pixel]
            '--z', '0',
            '--yaw', '0',
            '--pitch', '0',
            '--roll', '0',
            '--frame-id', 'camera_frame',
            '--child-frame-id', 'camera_center_frame'
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
            executable='ar_detector_node',
            name='ar_detector_node',
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
        output='screen',
    )


    return LaunchDescription([usb_cam] + [joy_node] + [tf_node] + tello_node_list + [rviz])
    # return LaunchDescription([joy_node])



