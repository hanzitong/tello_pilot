// cmd_vel_arrow_visualizer_node.cpp
//
// /cmd_vel と /pid_vel (Twist) を drone_frame 座標系の矢印として RViz に表示する。
// RViz の MarkerArray Display で /cmd_vel_markers を購読すると可視化できる。
//
//   namespace "cmd_vel": 緑矢印 — cmd_multiplexer が実際に送る速度指令
//   namespace "pid_vel": 黄矢印 — PID が計算した速度指令
//
//   各矢印の軸対応（drone_frame 基準、始点 = drone_frame 原点）:
//     id=0  linear.x  → drone 前後方向 (X 軸)
//     id=1  linear.y  → drone 左右方向 (Y 軸)
//     id=2  linear.z  → drone 上下方向 (Z 軸)  ※ pid_vel は X/Y のみ
//
// [ゼロ値の扱い]
//   RViz は始点と終点が同じ ARROW を描画しないため、値が kMinDisplay 未満のときは
//   正方向に kMinDisplay のスタブを表示して「ゼロに近い」ことを示す。

#include <chrono>
#include <functional>
#include <string>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;

// 値がこれより小さい場合は正方向スタブを表示する
static constexpr double kMinDisplay = 0.01;


class CmdVelArrowVisualizerNode : public rclcpp::Node
{
public:
    CmdVelArrowVisualizerNode() : Node("cmd_vel_arrow_visualizer")
    {
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", rclcpp::QoS(1),
            std::bind(&CmdVelArrowVisualizerNode::cmdVelCallback, this, std::placeholders::_1)
        );
        pid_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/pid_vel", rclcpp::QoS(1),
            std::bind(&CmdVelArrowVisualizerNode::pidVelCallback, this, std::placeholders::_1)
        );
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/cmd_vel_markers", rclcpp::QoS(1)
        );
        timer_ = create_wall_timer(
            50ms, std::bind(&CmdVelArrowVisualizerNode::timerCallback, this)
        );
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) { last_cmd_vel_ = *msg; }
    void pidVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) { last_pid_vel_ = *msg; }

    // 値が小さすぎる場合に正方向スタブに置き換える
    static double nonzero(double v)
    {
        return std::abs(v) < kMinDisplay ? kMinDisplay : v;
    }

    visualization_msgs::msg::Marker makeArrow(
        const std::string & ns, int id,
        double dx, double dy, double dz,
        float r, float g, float b)
    {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "drone_frame";
        m.header.stamp    = this->now();
        m.ns              = ns;
        m.id              = id;
        m.type            = visualization_msgs::msg::Marker::ARROW;
        m.action          = visualization_msgs::msg::Marker::ADD;

        geometry_msgs::msg::Point p0, p1;
        p0.x = 0.0; p0.y = 0.0; p0.z = 0.0;
        p1.x = dx;  p1.y = dy;  p1.z = dz;
        m.points = {p0, p1};

        m.scale.x = 0.005;  // shaft diameter [m]
        m.scale.y = 0.012;  // head diameter [m]
        m.scale.z = 0.0;    // head length [m] (0 = auto)

        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        m.color.a = 1.0f;

        return m;
    }

    void timerCallback()
    {
        visualization_msgs::msg::MarkerArray arr;

        arr.markers.push_back(makeArrow("cmd_vel", 0, nonzero(last_cmd_vel_.linear.x), 0,                              0,                              0.0f, 0.86f, 0.0f));
        arr.markers.push_back(makeArrow("cmd_vel", 1, 0,                              nonzero(last_cmd_vel_.linear.y), 0,                              0.0f, 0.86f, 0.0f));
        arr.markers.push_back(makeArrow("cmd_vel", 2, 0,                              0,                              nonzero(last_cmd_vel_.linear.z), 0.0f, 0.86f, 0.43f));

        arr.markers.push_back(makeArrow("pid_vel", 0, nonzero(last_pid_vel_.linear.x), 0,                              0, 0.86f, 0.86f, 0.0f));
        arr.markers.push_back(makeArrow("pid_vel", 1, 0,                              nonzero(last_pid_vel_.linear.y), 0, 0.86f, 0.86f, 0.0f));

        marker_pub_->publish(arr);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr         cmd_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr         pid_vel_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr                                        timer_;

    geometry_msgs::msg::Twist last_cmd_vel_;
    geometry_msgs::msg::Twist last_pid_vel_;
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelArrowVisualizerNode>());
    rclcpp::shutdown();
    return 0;
}
