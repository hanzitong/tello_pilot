#!/usr/bin/env python3
"""
sim_tello_driver.py

Ignition Gazebo シミュレーション用の簡易 tello_action サービスノード。
実ドローンの tello_driver_main が提供する /tello_action サービスを代替する。

現在は takeoff/land コマンドをログに記録して OK を返すだけ。
Gazebo モデルの物理的な移動は行わない（静的ドローンの smoke test 用）。

提供サービス:
  /drone1/tello_action  (tello_msgs/srv/TelloAction)
"""

import rclpy
from rclpy.node import Node
from tello_msgs.srv import TelloAction


class SimTelloDriver(Node):
    def __init__(self):
        super().__init__('sim_tello_driver')
        self.srv = self.create_service(
            TelloAction,
            '/drone1/tello_action',
            self.action_callback
        )
        self.get_logger().info('sim_tello_driver ready: /drone1/tello_action')

    def action_callback(self, request, response):
        self.get_logger().info(f'[sim] tello_action received: "{request.cmd}"')
        response.rc = TelloAction.Response.OK
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SimTelloDriver()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
