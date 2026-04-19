// cmd_vel_visualizer_node.cpp
//
// /cmd_vel (Twist) の内容を /camera/image_raw にオーバーレイして /image_cmd_vel として publish する
// デバッグノード。制御の符号・軸対応（画像座標 vs ドローン機体座標）の確認に使う。
//
// 座標系の注意:
//   /cmd_vel はドローン機体フレームの表現:
//     linear.x > 0 → 前進 (Tello fb)
//     linear.y > 0 → 右移動 (Tello lr = -linear.y * 100 < 0)
//     linear.z > 0 → 上昇 (Tello ud)
//     angular.z > 0 → 反時計回り yaw (Tello yaw = -angular.z * 100)
//   PID設計上、画像X誤差→linear.x、画像Y誤差→linear.y と対応しているため、
//   (linear.x, linear.y) を画像X/Y成分として描画すると「PID応答方向」が可視化できる。
//   ドローンのyawが変わると画像との対応も変わる点に注意。

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

        image_pub_ = create_publisher<sensor_msgs::msg::Image>("/image_cmd_vel", 10);
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_cmd_vel_ = *msg;
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        cv::Mat img = cv_ptr->image;

        const int cx = img.cols / 2;
        const int cy = img.rows / 2;

        const double vx  = last_cmd_vel_.linear.x;
        const double vy  = last_cmd_vel_.linear.y;
        const double vz  = last_cmd_vel_.linear.z;
        const double wz  = last_cmd_vel_.angular.z;

        // --- 中心十字 ---
        cv::line(img, {cx - 10, cy}, {cx + 10, cy}, {0, 0, 255}, 2);
        cv::line(img, {cx, cy - 10}, {cx, cy + 10}, {0, 0, 255}, 2);

        // --- (linear.x, linear.y) の2D速度矢印 ---
        // linear.x → 画像X方向、linear.y → 画像Y方向 (PID設計上の近似)
        const int tip_x = cx + static_cast<int>(vx * kVelScale);
        const int tip_y = cy + static_cast<int>(vy * kVelScale);
        cv::arrowedLine(
            img,
            cv::Point(cx, cy),
            cv::Point(tip_x, tip_y),
            cv::Scalar(0, 220, 0),   // 緑 (BGR)
            3, cv::LINE_8, 0, 0.25
        );

        // --- テキスト: 速度値（左上） ---
        char buf[128];
        std::snprintf(buf, sizeof(buf),
            "Vx:%+.2f  Vy:%+.2f  Vz:%+.2f", vx, vy, vz);
        cv::putText(img, buf, cv::Point(10, 28),
            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

        const std::string wz_str = (wz > 0.01) ? "CCW" : (wz < -0.01) ? " CW" : "---";
        std::snprintf(buf, sizeof(buf), "Wyaw:%+.2f (%s)", wz, wz_str.c_str());
        cv::putText(img, buf, cv::Point(10, 56),
            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

        // --- 座標系注釈（右下） ---
        const std::string note = "arrow: linear.x/y in image coords (drone frame approx)";
        cv::putText(img, note,
            cv::Point(10, img.rows - 10),
            cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(150, 150, 150), 1, cv::LINE_AA);

        image_pub_->publish(*cv_bridge::CvImage(msg->header, "bgr8", img).toImageMsg());
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr   image_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr      image_pub_;

    geometry_msgs::msg::Twist last_cmd_vel_;   // zero-initialized by default constructor

    static constexpr int kVelScale = 150;      // [px / unit velocity]
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelVisualizerNode>());
    rclcpp::shutdown();
    return 0;
}
