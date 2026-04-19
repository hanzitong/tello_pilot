
from launch import LaunchDescription
from launch_ros.actions import Node


usb_cam = Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
            output='screen',
            parameters=[
                {
                    'video_device': '/dev/video_tello', # it may sometimes be changed automatically
                    'image_width': 640,
                    'image_height': 480,
                    'camera_frame_id': 'camera_frame',

                    # ===== Use YUYV
                    'pixel_format': 'yuyv',
                    'framerate': 25.0,

                    # ===== Use MJPEG
                    # 'pixel_format': 'mjpeg2rgb',
                    # 'framerate': 30.0,
                }
            ],
            remappings=[
                ('/image_raw', '/camera/image_raw'),
            ],
            respawn=True,
            # respawn_delay=0.2,
        )


def generate_launch_description():
    return LaunchDescription([
        usb_cam,
    ])

