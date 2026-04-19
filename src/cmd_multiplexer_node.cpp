// tello_pilot cmd_multiplexer.cpp

#include <memory>
#include <functional>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tello_msgs/srv/tello_action.hpp"

using namespace std::chrono_literals;


class CmdMultiplexer : public rclcpp::Node
{
public:
  CmdMultiplexer(): Node("cmd_multiplexer")
  {
    // Subscribe to joystick input
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10,
      std::bind(&CmdMultiplexer::joy_callback, this, std::placeholders::_1)
    );

    // Subscribe /pid_vel
    pid_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "pid_vel", 10,
      std::bind(&CmdMultiplexer::pid_callback, this, std::placeholders::_1)
    );

    // Publisher for cmd_vel
    selected_twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel", 10
    );

    // Service client for takeoff / land
    action_client_ = this->create_client<tello_msgs::srv::TelloAction>("tello_action");

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&CmdMultiplexer::timer_callback, this)
    );
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
  {
    last_joy_ = *joy_msg;
    got_joy_ = true;
  }

  void pid_callback(const geometry_msgs::msg::Twist::SharedPtr pid_msg)
  {
    last_pid_ = *pid_msg;
  }

  void send_action(const std::string & cmd)
  {
    auto request = std::make_shared<tello_msgs::srv::TelloAction::Request>();
    request->cmd = cmd;
    action_client_->async_send_request(request);
    RCLCPP_INFO(this->get_logger(), "Sent tello_action: %s", cmd.c_str());
  }

  void timer_callback()
  {
    geometry_msgs::msg::Twist selected_twist;

    selected_twist.linear.x = 0.;
    selected_twist.linear.y = 0.;
    selected_twist.linear.z = 0.;
    selected_twist.angular.x = 0.;
    selected_twist.angular.y = 0.;
    selected_twist.angular.z = 0.;

    if (got_joy_) {
      // --- 一発トリガー（ボタンの立ち上がりエッジで1回だけ送信）---
      // Start（buttons[7]）: テイクオフ
      if (rising_edge(7)) {
        send_action("takeoff");
      }
      // Back（buttons[6]）: 手動着陸
      if (rising_edge(6)) {
        send_action("land");
      }

      // --- 速度コマンドの選択 ---
      const auto nb = static_cast<int>(last_joy_.buttons.size());
      const auto na = static_cast<int>(last_joy_.axes.size());
      if (nb > 5 && last_joy_.buttons[5] == 1) {        // RB: 自動モード
        selected_twist.linear.x = last_pid_.linear.x;
        selected_twist.linear.y = last_pid_.linear.y;
        selected_twist.angular.z = last_pid_.angular.z;
      } else if (nb > 4 && last_joy_.buttons[4] == 1) { // LB: テスト前進
        selected_twist.linear.x = 0.5;
        selected_twist.linear.y = 0.;
        selected_twist.angular.z = 0.;
      } else {                                           // 手動モード
        selected_twist.linear.x  = (na > 4) ? last_joy_.axes[4] : 0.;
        selected_twist.linear.y  = (na > 3) ? last_joy_.axes[3] : 0.;
        selected_twist.angular.z = (na > 0) ? last_joy_.axes[0] : 0.;
      }
      selected_twist.linear.z = (na > 1) ? last_joy_.axes[1] : 0.;
    }

    selected_twist_pub_->publish(selected_twist);

    prev_joy_ = last_joy_;
  }

  // ボタン i の立ち上がりエッジを検出（前回0・今回1）
  bool rising_edge(int i)
  {
    int cur  = (i < static_cast<int>(last_joy_.buttons.size()))  ? last_joy_.buttons[i]  : 0;
    int prev = (i < static_cast<int>(prev_joy_.buttons.size()))  ? prev_joy_.buttons[i]  : 0;
    return (prev == 0 && cur == 1);
  }


  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr   joy_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr pid_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr  selected_twist_pub_;
  rclcpp::Client<tello_msgs::srv::TelloAction>::SharedPtr  action_client_;
  rclcpp::TimerBase::SharedPtr                             timer_;

  sensor_msgs::msg::Joy   last_joy_;
  sensor_msgs::msg::Joy   prev_joy_;
  geometry_msgs::msg::Twist last_pid_;
  bool got_joy_{false};
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdMultiplexer>());
  rclcpp::shutdown();
  return 0;
}
