// tello_pilot cmd_multiplexer.cpp

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class ArucoTwistNode : public rclcpp::Node
{
public:
  ArucoTwistNode(): Node("aruco_twist_node")
  {
    // Subscribe to joystick input
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10,
      std::bind(&ArucoTwistNode::joy_callback, this, std::placeholders::_1)
    );

    // Subscribe to incoming ArUco twist messages
    aruco_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/aruco_twist", 10,
      std::bind(&ArucoTwistNode::aruco_callback, this, std::placeholders::_1)
    );

    // Publisher for cmd_vel
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel", 10
    );

    // Timer: publish cmd_vel at 10Hz
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ArucoTwistNode::timer_callback, this)
    );
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
  {
    last_joy_ = *joy_msg;
    got_joy_ = true;
  }

  void aruco_callback(const geometry_msgs::msg::Twist::SharedPtr aruco_msg)
  {
    last_aruco_twist_ = *aruco_msg;
    got_aruco_ = true;
  }

  void timer_callback()
  {
    geometry_msgs::msg::Twist twist;

    if (got_joy_ && last_joy_.buttons.size() > 5 && last_joy_.buttons[5] == 1 && got_aruco_) {
      twist = last_aruco_twist_;

      // Example modification: boost forward speed and add yaw offset
      twist.linear.x = 0;
      twist.angular.z -= 0.5;

      // TODO: use pid control here

      // TODO: position control
      // 0, get position vector between ar_marker and the center of image 
      // 1, apply rotation matrix to position vector with tf2
      // 2, make cmd_vel using pid contro.

      // TODO: angle control (optional) 

    } else if (got_joy_) {
      // low speed
      // twist.linear.x  = 0.5 * last_joy_.axes[4];
      // twist.linear.y  = 0.5 * last_joy_.axes[3];
      // twist.linear.z  =  0.5 * last_joy_.axes[1];

      // normal speed
      twist.linear.x  =  last_joy_.axes[4];
      twist.linear.y  =  last_joy_.axes[3];
      twist.linear.z  =  last_joy_.axes[1];

      // twist.angular.x  // ignored
      // twist.angular.y  // ignored
      if(last_joy_.buttons[5] > 0.5)twist.angular.z  = -0.5;
      if(last_joy_.buttons[4] > 0.5)twist.angular.z  = 0.5;

    }

    twist_pub_->publish(twist);
  }

  // if(last_joy_.buttons)

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr aruco_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::Joy last_joy_;
  geometry_msgs::msg::Twist last_aruco_twist_;
  bool got_joy_{false};
  bool got_aruco_{false};
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoTwistNode>());
  rclcpp::shutdown();
  return 0;
}

