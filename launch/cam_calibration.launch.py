

from launch import LaunchDescription
from launch_ros.actions import Node


usb_cam = Node(
    package='usb_cam',
    executable='usb_cam_node_exe',
    name='usb_cam_node',
    output='screen',
    parameters=[    # declared parameters
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

calibration_node = Node(
    package='camera_calibration',
    executable='cameracalibrator',
    name='calibrator_node',
    output='screen',
    arguments=[     # CLI arguments
        'size': '8x6',
        'square': 0.025,
    ]
    remappings=[
        ('/image', '/camera/image_raw'),
        ('/camera', '/camera'),
    ]
)



def generate_launch_description():
    return LaunchDescription([
        usb_cam,
        calibration_node,
    ])


