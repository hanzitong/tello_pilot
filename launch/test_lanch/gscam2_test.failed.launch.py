
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    gscam_pipeline = (
        "v4l2src device=/dev/video4 do-timestamp=true ! "
        "image/jpeg,width=640,height=480,framerate=30/1 ! "
        "queue max-size-buffers=1 leaky=downstream ! "
        "jpegdec ! "
        "videoconvert"
    )

    return LaunchDescription([
        Node(
            package='gscam2',
            executable='gscam_main',
            name='gscam_publisher',
            output='screen',
            parameters=[
                {
                    'gscam_config': gscam_pipeline,
                    'preroll': False,
                    'use_gst_timestamps': True,
                    'camera_name': 'usb_camera',
                    'frame_id': 'camera_frame',
                    'image_encoding': 'rgb8',
                    'sync_sink': False,
                }
            ],
            remappings=[
                ('/image_raw', '/camera/image_raw'),
                ('/camera_info', '/camera/camera_info'),
            ],
        )
    ])

