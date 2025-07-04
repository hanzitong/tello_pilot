
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

class ControlInputPublisher : public rclcpp::Node
{
  public:
    ControlInputPublisher()
    : Node("pid_controller"),
    pid_x_{1.0, 0., 0.1},
    pid_y_{1.3, 0., 0.1}
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("pid_vel", 10);
      // subscriber_ = this->create_subscription<>();
      timer_ = this->create_wall_timer(
        100ms, std::bind(&ControlInputPublisher::timer_callback, this)
      );

      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


      debug_t_tello_pub = this->create_publisher<geometry_msgs::msg::TransformStamped>("t_tello", 10);
    }

  private:
    // void timer_callback(geometry_msgs::msg::Vector3Stamped marker23_translation_camframe)
    void timer_callback()
    {
        // get transform
        // geometry_msgs::msg::TransformStamped t_cam;
        geometry_msgs::msg::TransformStamped t_tello;
        // t_cam = tf_buffer_ -> lookupTransform("camera_cv_frame","marker_23_frame", tf2::TimePointZero); 

        geometry_msgs::msg::Twist pid_vel_msg;
        pid_vel_msg.linear.x = 0.;
        pid_vel_msg.linear.y = 0.;
        pid_vel_msg.linear.z = 0.;
        pid_vel_msg.angular.x = 0.;
        pid_vel_msg.angular.y = 0.;
        pid_vel_msg.angular.z = 0.;

        try{
          // to, from (matrix:from camera, watch marker)
          t_tello = tf_buffer_->lookupTransform("marker_23_frame", "camera_center_frame", tf2::TimePointZero);

          // check time of tf


          pid_vel_msg.linear.x = pid_x_.compute(t_tello.transform.translation.x, 0., 0.1);
          pid_vel_msg.linear.y = pid_y_.compute(-1 * t_tello.transform.translation.y, 0., 0.1);
          pid_vel_msg.linear.z = 0.;
          pid_vel_msg.angular.x = 0.;
          pid_vel_msg.angular.y = 0.;
          pid_vel_msg.angular.z = 0.;
          // pid_vel_msg.angular.z = pid_z_.compute();

          debug_t_tello_pub->publish(t_tello);

        }catch(const tf2::TransformException & ex){
          RCLCPP_WARN(this->get_logger(),
            "Could not transform from camera_center_frame to marker_23_frame: %s", ex.what());
        }


        // geometry_msgs::msg::Twist pid_vel_msg;
        // pid_vel_msg.linear.x = pid_.compute(t_tello.transform.translation.x, 0., 0.1);
        // pid_vel_msg.linear.y = pid_.compute(-1 * t_tello.transform.translation.y, 0., 0.1);
        // pid_vel_msg.linear.z = 0.;
        publisher_->publish(pid_vel_msg);


        /* what is it ????????????????  forgot
        geometry_msgs::msg::Vector3Stamped marker23_translation_camframe;
        marker23_translation_camframe.header.stamp = t_cam.header.stamp;
        marker23_translation_camframe.header.frame_id = "camera_cv_frame";
        marker23_translation_camframe.vector.x = t_cam.transform.translation.x;
        marker23_translation_camframe.vector.y = t_cam.transform.translation.y;
        geometry_msgs::msg::Vector3Stamped center_translation_marker23frame;
        // tf_buffer_->transform(marker23_translation_camframe, center_translation_marker23frame, "camera_center_frame"); // argument order: v_in, v_out, "frame_destination"
        tf_buffer_->transform(marker23_translation_camframe, center_translation_marker23frame, "camera_cv_frame");
        */

    }


    rclcpp::TimerBase::SharedPtr timer_;
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_{nullptr};
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    // std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;


    PIDController pid_x_;
    PIDController pid_y_;


    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr debug_t_tello_pub;

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlInputPublisher>());
  rclcpp::shutdown();
  return 0;
}

