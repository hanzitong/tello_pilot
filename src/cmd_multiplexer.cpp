// tello_pilot cmd_multiplexer.cpp

#include <memory>
#include <functional>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

// #include "../../tello_ros/tell_msgs/src/tello_action.hpp"
// #include "TelloAction.srv"
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
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&CmdMultiplexer::timer_callback, this)
    );

    // client_ = this->create_client<tello_msgs::srv::TelloAction>("/tello_action");
    // while (!client_->wait_for_service(1s)) {
    //   RCLCPP_INFO(this->get_logger(), "Waiting for /tello_action service ...");
    // }
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
  {
    last_joy_ = *joy_msg;
    got_joy_ = true;
    // last_joy_time_ = this->now();
  }

  void pid_callback(const geometry_msgs::msg::Twist::SharedPtr pid_msg)
  {
    last_pid_ = *pid_msg;
    // got_pid_ = true; // if pid_vel dead, Twist will be all 0 at pid_controller
  }

  /* for calling takeoff service from joy */
  // void send_takeoff()
  // {
  //   auto request = std::make_shared<TelloAction::Request>();
  //   request->cmd = "takeoff";
  //   RCLCPP_INFO(this->get_logger(), "Send takeoff command ...");

  //   // async_send_request
  //   client_->async_send_request(request);
  // }


  void timer_callback()
  {
    geometry_msgs::msg::Twist selected_twist;
    // rclcpp::Time now = this->now();
    // bool joy_alive = (now - last_joy_time_) < joy_timeout_;
    // bool joy_alive = (last_joy_time_ - now) < joy_timeout_;
    // RCLCPP_INFO(this->get_logger(), "now time: %f", now.seconds());

    selected_twist.linear.x = 0.;
    selected_twist.linear.y = 0.;
    selected_twist.linear.z = 0.;
    selected_twist.angular.x = 0.;
    selected_twist.angular.y = 0.;
    selected_twist.angular.z = 0.;

    // if (joy_alive) {
    if (got_joy_) {
      if (last_joy_.buttons[5] == 1) { // auto mode
        // if exceed [-1, 1], it seems to be ignored ...
        selected_twist.linear.x = last_pid_.linear.x;
        selected_twist.linear.y = last_pid_.linear.y;
        // selected_twist.angular.x = last_pid_.angular.x;  // ignored at tello_driver
        // selected_twist.angular.y = last_pid_.angular.y;  // ignored at tello_driver
        selected_twist.angular.z = last_pid_.angular.z;
      } else if (last_joy_.buttons[4] == 1) {
        selected_twist.linear.x = 0.5;
        selected_twist.linear.y = 0.;
        selected_twist.angular.z = 0.;
      } else {                        // manual mode
        selected_twist.linear.x  =  last_joy_.axes[4];
        selected_twist.linear.y  =  last_joy_.axes[3];
        // selected_twist.angular.x  // ignored at tello_driver
        // selected_twist.angular.y  // ignored at tello_driver
        selected_twist.angular.z = last_joy_.axes[0];
      }
      selected_twist.linear.z  =  last_joy_.axes[1];

    }

    selected_twist_pub_->publish(selected_twist);

    // if(got_joy_ && last_joy_.buttons[4] == 1){
    //   send_takeoff();
    // }


  } // timer_callback()


  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr pid_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr selected_twist_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  // rclcpp::Client<TelloAction>::SharedPtr client_;

  sensor_msgs::msg::Joy last_joy_;
  geometry_msgs::msg::Twist last_pid_;
  bool got_joy_{false};
  // bool joy_alive_{false};
  // rclcpp::Time last_joy_time_{0};
  // rclcpp::Duration joy_timeout_{0, 500000000};
  // bool got_pid_{false};

};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdMultiplexer>());
  rclcpp::shutdown();
  return 0;
}

