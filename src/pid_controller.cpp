
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "pid_controller.hpp"


using namespace std::chrono_literals;

class PidVelPublisher : public rclcpp::Node
{
  public:
    PidVelPublisher()
    // PidVelPublisher(int pixel_width, int pixel_height)
    : Node("pid_controller"),
    // pid_x_{0.5, 0.001, 0.},  // 0.3
    // pid_y_{0.5, 0.001, 0.}   // 0.2
    pid_x_{0.55, 0., 0.},  // 0.3
    pid_y_{0.55, 0., 0.},  // 0.2
    last_time_(this->now())
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("pid_vel", 10);
      timer_ = this->create_wall_timer(
        100ms, std::bind(&PidVelPublisher::timer_callback, this)
        // 100ms,
        // [this, pixel_width, pixel_height](){this->timer_callback(pixel_width_, pixel_height_);}
      );
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


      // debug_t_tello_pub = this->create_publisher<geometry_msgs::msg::TransformStamped>("t_tello", 10);
    }

  private:
    void timer_callback()
    {
        const rclcpp::Time now = this->now();
        const double dt = (now - last_time_).seconds();
        last_time_ = now;

        geometry_msgs::msg::TransformStamped t_tello; // contain listened transform
        geometry_msgs::msg::Twist pid_vel_msg;        // contain cmd_vel generated with pid to be published

        pid_vel_msg.linear.x = 0.;
        pid_vel_msg.linear.y = 0.;
        pid_vel_msg.linear.z = 0.;
        pid_vel_msg.angular.x = 0.;
        pid_vel_msg.angular.y = 0.;
        pid_vel_msg.angular.z = 0.;

        try{
          // camera_center_frame から見た marker_23_frame の位置を取得する
          // = (marker_pixel - center_pixel) / 100
          // マーカーが右/下にあるとき translation.x/y が正になる（直感的な符号）
          t_tello = tf_buffer_->lookupTransform("camera_center_frame", "marker_23_frame", tf2::TimePointZero);

          // TODO: check time of tf

          // normarize x and y (not z yet)
          // double pidxy_norm = std::sqrt(std::pow(pixel_width_, 2) + std::pow(pixel_height_, 2)) / 100; // [100 * pixel]
          // double pidxy_norm = 10; // heuristic
          // double pidxy_norm = 7; // heuristic
          double pidxy_norm = 4; // heuristic

          // generate /pid_vel
          pid_vel_msg.linear.x = pid_x_.compute(t_tello.transform.translation.x, 0., dt) / pidxy_norm;
          pid_vel_msg.linear.y = pid_y_.compute(t_tello.transform.translation.y, 0., dt) / pidxy_norm;
          // pid_vel_msg.linear.x = pid_x_.compute(-1 * t_tello.transform.translation.x, 0., 0.1) / pidxy_norm;  // 100ms
          // pid_vel_msg.linear.y = pid_y_.compute(-1 * t_tello.transform.translation.y, 0., 0.1) / pidxy_norm;
            // -1 is for adjusting axis
          pid_vel_msg.linear.z = 0.;
          pid_vel_msg.angular.x = 0.;
          pid_vel_msg.angular.y = 0.;
          pid_vel_msg.angular.z = 0.;
          // pid_vel_msg.angular.z = pid_z_.compute();

          // debug_t_tello_pub->publish(t_tello);

        }catch(const tf2::TransformException & ex){
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Could not transform from camera_center_frame to marker_23_frame: %s", ex.what());
        }


        // geometry_msgs::msg::Twist pid_vel_msg;
        // pid_vel_msg.linear.x = pid_.compute(t_tello.transform.translation.x, 0., 0.1);
        // pid_vel_msg.linear.y = pid_.compute(-1 * t_tello.transform.translation.y, 0., 0.1);
        // pid_vel_msg.linear.z = 0.;
        if (pid_vel_msg.linear.x > 0.8) pid_vel_msg.linear.x = 0.5;
        if (pid_vel_msg.linear.x < -0.8) pid_vel_msg.linear.x = -0.5;
        if (pid_vel_msg.linear.y > 0.8) pid_vel_msg.linear.y = 0.5;
        if (pid_vel_msg.linear.y < -0.8) pid_vel_msg.linear.y = -0.5;


        publisher_->publish(pid_vel_msg);


    }


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    PIDController pid_x_;
    PIDController pid_y_;
    rclcpp::Time last_time_;


    // rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr debug_t_tello_pub;
};


int main(int argc, char * argv[])
{
  // if (argc < 3) {
  //   std::cout << "give arugment !!! (pixel_width, pixel_height)" << std::endl;
  //   return 1;
  // }
  // int pixel_width = std::stoi(argv[1]);
  // int pixel_height = std::stoi(argv[2]);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PidVelPublisher>());
  // rclcpp::spin(std::make_shared<PidVelPublisher>(pixel_width, pixel_height));
  rclcpp::shutdown();
  return 0;
}

