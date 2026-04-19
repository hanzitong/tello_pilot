
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    opencv_cam = Node(
                    package='opencv_cam',   # maybe cannot use "MJPEG" streaming
                    executable='opencv_cam_main',
                    name='opencv_cam_node',
                    output='both',
                    parameters=[
                        {
                            'index': 5,
                            'width': 640,
                            'height': 480,
                            'fps': 25,      # max fps in YUYV
                        }
                    ]
                 )

    return LaunchDescription([
                opencv_cam,
            ])
    


