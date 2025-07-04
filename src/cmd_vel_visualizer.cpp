// tello_pilot cmd_vel_visualizer.cpp

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>


class CmdVelVisualizer : public rclcpp::Node
{






    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdMultiplexer>());
    rclcpp::shutdown();

    return 0;
}





