"""
sim_landing.launch.py

Ignition Gazebo 6 (Fortress) シミュレーション起動ファイル。
地面に固定した ArUco marker_23 をドローン搭載カメラで見下ろし、
tello_pilot の全制御スタック（ar_detector → pid_controller → auto_lander）を検証する。

実機との差異:
  実機  : 地面固定カメラ(↑) + ドローン底面 marker
  本sim : ドローン搭載カメラ(↓) + 地面固定 marker_23
  → 制御ロジック（マーカーを画像中心へ）は等価。詳細は docs/sim_vs_real.md 参照。

起動方法:
  source install/setup.bash
  ros2 launch tello_pilot sim_landing.launch.py

確認コマンド:
  ros2 topic list                      # /drone1/camera/image, /cam_image_raw 等を確認
  ros2 topic hz /cam_image_raw         # 30Hz 前後であること
  ros2 topic echo /pid_vel             # マーカー検出後に値が出ること
  ign topic -e -t /drone1/camera/image # Ignition 側カメラ確認
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

# ----- パス設定 -----
_pkg = get_package_share_directory('tello_pilot')
_world = os.path.join(_pkg, 'worlds', 'sim_landing_ign.world')
_urdf  = os.path.join(_pkg, 'urdf', 'tello_sim.urdf')

# Ignition にモデルパスを伝える（marker_23 の URI 解決に必要）
os.environ['IGN_GAZEBO_RESOURCE_PATH'] = _pkg + '/models'

# カメラ解像度（sim_landing_ign.world の camera sensor と合わせる）
_cam_w, _cam_h = 960, 720


def generate_launch_description():
    return LaunchDescription([

        # ===== Ignition Gazebo =====
        # -r: シミュレーション即時開始
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', _world],
            output='screen',
        ),

        # ===== ros_gz_bridge: Ignition カメラ → ROS2 =====
        # Ignition topic: /drone1/camera/image  (ignition.msgs.Image)
        # ROS2    topic : /cam_image_raw         (sensor_msgs/msg/Image)
        #
        # 書式: ros2_topic@ros_type[ign_type  ([ = Ignition→ROS2 方向)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_cam_bridge',
            output='screen',
            arguments=[
                '/drone1/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            ],
            remappings=[
                ('/drone1/camera/image', '/cam_image_raw'),
            ],
        ),

        # ===== ドローン TF（URDF から生成）=====
        # base_link_1 → camera_link_1 の固定 joint を配信する
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            arguments=[_urdf],
        ),

        # ===== 静的 TF: world → base_link_1 =====
        # Ignition の静的ドローンは z=1.0m に配置されているため、
        # TF ツリーでも同じ位置に固定する。
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '1.0',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'world',
                '--child-frame-id', 'base_link_1',
            ],
        ),

        # ===== 静的 TF: camera_link_1 → camera_cv_frame =====
        # ar_detector は TF 親フレームを "camera_cv_frame" にハードコードしているため、
        # Ignition カメラの物理フレーム (camera_link_1) と接続する。
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'camera_link_1',
                '--child-frame-id', 'camera_cv_frame',
            ],
        ),

        # ===== 静的 TF: camera_cv_frame → camera_center_frame =====
        # 画像中心オフセット [pixel/100]: 960/2/100=4.8, 720/2/100=3.6
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', str(_cam_w / 2 / 100),
                '--y', str(_cam_h / 2 / 100),
                '--z', '0',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'camera_cv_frame',
                '--child-frame-id', 'camera_center_frame',
            ],
        ),

        # ===== ジョイスティック =====
        Node(
            package='joy',
            executable='joy_node',
            output='screen',
        ),

        # ===== ar_detector =====
        # /cam_image_raw を受け取る（ros_gz_bridge がリマップ済み）
        Node(
            package='tello_pilot',
            executable='ar_detector',
            output='screen',
        ),

        # ===== pid_controller =====
        Node(
            package='tello_pilot',
            executable='pid_controller',
            output='screen',
        ),

        # ===== cmd_multiplexer =====
        # tello_action → /drone1/tello_action (sim_tello_driver が提供)
        Node(
            package='tello_pilot',
            executable='cmd_multiplexer',
            output='screen',
            remappings=[
                ('cmd_vel',      '/drone1/cmd_vel'),
                ('tello_action', '/drone1/tello_action'),
            ],
        ),

        # ===== auto_lander =====
        Node(
            package='tello_pilot',
            executable='auto_lander',
            output='screen',
            remappings=[
                ('tello_action', '/drone1/tello_action'),
            ],
        ),

        # ===== sim_tello_driver =====
        # /drone1/tello_action サービスを提供（takeoff/land を受け付け OK を返す）
        Node(
            package='tello_pilot',
            executable='sim_tello_driver.py',
            output='screen',
        ),

    ])
