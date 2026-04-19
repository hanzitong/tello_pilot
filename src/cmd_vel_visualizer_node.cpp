// cmd_vel_visualizer_node.cpp
//
// /pid_vel と /cmd_vel (Twist) の内容を /camera/image_raw にオーバーレイして
// /image_cmd_vel として publish するデバッグノード。
//
//   /pid_vel: PID が計算した速度指令（黄色矢印）— ジョイスティック不要で常に表示
//   /cmd_vel: cmd_multiplexer が実際にドローンへ送る速度指令（緑矢印）
//             ジョイスティック未接続 or RBボタン未押下 → 常に 0
//
// 座標系の注意:
//   両トピックともドローン機体フレームの表現:
//     linear.x > 0 → 前進 (Tello fb)
//     linear.y > 0 → 右移動 (Tello lr = -linear.y * 100 < 0)
//     linear.z > 0 → 上昇 (Tello ud)
//     angular.z > 0 → 反時計回り yaw
//   PID設計上、画像X誤差→linear.x、画像Y誤差→linear.y と対応している。

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


class CmdVelVisualizerNode : public rclcpp::Node
{
public:
    CmdVelVisualizerNode() : Node("cmd_vel_visualizer")
    {
        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&CmdVelVisualizerNode::imageCallback, this, std::placeholders::_1)
        );

        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&CmdVelVisualizerNode::cmdVelCallback, this, std::placeholders::_1)
        );

        pid_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/pid_vel", 10,
            std::bind(&CmdVelVisualizerNode::pidVelCallback, this, std::placeholders::_1)
        );

        image_pub_ = create_publisher<sensor_msgs::msg::Image>("/image_cmd_vel", 10);
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_cmd_vel_ = *msg;
    }

    void pidVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_pid_vel_ = *msg;
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        cv::Mat img = cv_ptr->image;

        const int cx = img.cols / 2;
        const int cy = img.rows / 2;

        const double cvx = last_cmd_vel_.linear.x;
        const double cvy = last_cmd_vel_.linear.y;
        const double cvz = last_cmd_vel_.linear.z;
        const double cwz = last_cmd_vel_.angular.z;

        const double pvx = last_pid_vel_.linear.x;
        const double pvy = last_pid_vel_.linear.y;

        // --- 中心十字 ---
        cv::line(img, {cx - 10, cy}, {cx + 10, cy}, {0, 0, 255}, 2);
        cv::line(img, {cx, cy - 10}, {cx, cy + 10}, {0, 0, 255}, 2);

        // --- /pid_vel 矢印（黄色）: PID が計算した速度指令 ---
        const int pid_tip_x = cx + static_cast<int>(pvx * kVelScale);
        const int pid_tip_y = cy + static_cast<int>(pvy * kVelScale);
        cv::arrowedLine(
            img,
            cv::Point(cx, cy),
            cv::Point(pid_tip_x, pid_tip_y),
            cv::Scalar(0, 220, 220),   // 黄色 (BGR)
            2, cv::LINE_8, 0, 0.25
        );

        // --- /cmd_vel 矢印（緑）: ドローンへ実際に送る速度指令 ---
        const int cmd_tip_x = cx + static_cast<int>(cvx * kVelScale);
        const int cmd_tip_y = cy + static_cast<int>(cvy * kVelScale);
        cv::arrowedLine(
            img,
            cv::Point(cx, cy),
            cv::Point(cmd_tip_x, cmd_tip_y),
            cv::Scalar(0, 220, 0),     // 緑 (BGR)
            3, cv::LINE_8, 0, 0.25
        );

        // --- テキスト: 速度値（左上） ---
        char buf[128];
        std::snprintf(buf, sizeof(buf),
            "[PID] Vx:%+.2f  Vy:%+.2f", pvx, pvy);
        cv::putText(img, buf, cv::Point(10, 28),
            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 220, 220), 1, cv::LINE_AA);

        std::snprintf(buf, sizeof(buf),
            "[CMD] Vx:%+.2f  Vy:%+.2f  Vz:%+.2f", cvx, cvy, cvz);
        cv::putText(img, buf, cv::Point(10, 56),
            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 220, 0), 1, cv::LINE_AA);

        const std::string wz_str = (cwz > 0.01) ? "CCW" : (cwz < -0.01) ? " CW" : "---";
        std::snprintf(buf, sizeof(buf), "[CMD] Wyaw:%+.2f (%s)", cwz, wz_str.c_str());
        cv::putText(img, buf, cv::Point(10, 84),
            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 220, 0), 1, cv::LINE_AA);

        // --- 凡例（右下） ---
        cv::putText(img, "yellow: /pid_vel  green: /cmd_vel",
            cv::Point(10, img.rows - 10),
            cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(150, 150, 150), 1, cv::LINE_AA);

        image_pub_->publish(*cv_bridge::CvImage(msg->header, "bgr8", img).toImageMsg());
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr   image_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr pid_vel_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr      image_pub_;

    geometry_msgs::msg::Twist last_cmd_vel_;   // zero-initialized by default constructor
    geometry_msgs::msg::Twist last_pid_vel_;   // zero-initialized by default constructor

    static constexpr int kVelScale = 150;      // [px / unit velocity]
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelVisualizerNode>());
    rclcpp::shutdown();
    return 0;
}
