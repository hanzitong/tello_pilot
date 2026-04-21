// cmd_vel_visualizer_node.cpp
//
// /pid_vel と /cmd_vel (Twist) の内容を /camera/image_raw にオーバーレイして
// /image_cmd_vel として publish するデバッグノード。
//
//   /pid_vel: PID が計算した速度指令（黄色矢印）— ジョイスティック不要で常に表示
//   /cmd_vel: cmd_multiplexer が実際にドローンへ送る速度指令（緑矢印）
//             ジョイスティック未接続 or RBボタン未押下 → 常に 0
//
// 矢印の始点: drone_frame を camera_frame に投影した画像座標
//             TF または /camera_info が未取得の場合は画像中心にフォールバック
//
// 矢印の方向・長さ: linear.x/y [drone_frame] をピクセルにスケーリング
//   drone X(前後) ≈ 画像 X 方向（右）、drone Y(左右) ≈ 画像 Y 方向（下）
//   ※ yaw=0・水平飛行時のみ正確な対応。yaw が変わると画像軸との対応がずれる。

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>


class CmdVelVisualizerNode : public rclcpp::Node
{
public:
    CmdVelVisualizerNode() : Node("cmd_vel_visualizer")
    {
        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        // KeepLast(N): 履歴ポリシー。キューに最新 N 件のみ保持し、超過分は破棄する。
        // depth=1 にすることで古いフレームがキューに溜まらず、常に最新データを処理できる。
        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw",
            rclcpp::SensorDataQoS(rclcpp::KeepLast(1)),
            std::bind(&CmdVelVisualizerNode::imageCallback, this, std::placeholders::_1)
        );

        camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera_info",
            rclcpp::SensorDataQoS(rclcpp::KeepLast(1)),
            std::bind(&CmdVelVisualizerNode::cameraInfoCallback, this, std::placeholders::_1)
        );

        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", rclcpp::QoS(1),
            std::bind(&CmdVelVisualizerNode::cmdVelCallback, this, std::placeholders::_1)
        );

        pid_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/pid_vel", rclcpp::QoS(1),
            std::bind(&CmdVelVisualizerNode::pidVelCallback, this, std::placeholders::_1)
        );

        image_pub_ = create_publisher<sensor_msgs::msg::Image>(
            "/image_cmd_vel", rclcpp::QoS(1));
    }

private:
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (camera_info_ready_) return;
        // K は行優先 3×3。インデックス: [0]=fx [2]=cx [4]=fy [5]=cy
        fx_ = msg->k[0];
        fy_ = msg->k[4];
        cx_ = msg->k[2];
        cy_ = msg->k[5];
        camera_info_ready_ = true;
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) { last_cmd_vel_ = *msg; }
    void pidVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) { last_pid_vel_ = *msg; }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        cv::Mat img = cv_ptr->image;

        const int cx_img = img.cols / 2;
        const int cy_img = img.rows / 2;

        // --- drone_frame の camera_frame への透視投影を矢印の始点にする ---
        // px = fx * (tx / tz) + cx_calib
        // py = fy * (ty / tz) + cy_calib
        int origin_x = cx_img;
        int origin_y = cy_img;
        bool drone_visible = false;

        if (camera_info_ready_) {
            try {
                const auto t = tf_buffer_->lookupTransform(
                    "camera_frame", "drone_frame", tf2::TimePointZero);
                const double tx = t.transform.translation.x;
                const double ty = t.transform.translation.y;
                const double tz = t.transform.translation.z;
                if (tz > 1e-6) {
                    origin_x = static_cast<int>(fx_ * tx / tz + cx_);
                    origin_y = static_cast<int>(fy_ * ty / tz + cy_);
                    drone_visible = true;
                }
            } catch (const tf2::TransformException &) {
                // TF 未取得: 画像中心にフォールバック
            }
        }

        const double cvx = last_cmd_vel_.linear.x;
        const double cvy = last_cmd_vel_.linear.y;
        const double cvz = last_cmd_vel_.linear.z;
        const double cwz = last_cmd_vel_.angular.z;
        const double pvx = last_pid_vel_.linear.x;
        const double pvy = last_pid_vel_.linear.y;

        // --- 中心十字（赤）: 制御目標（画像中心 ≈ 主点） ---
        cv::line(img, {cx_img - 10, cy_img}, {cx_img + 10, cy_img}, {0, 0, 255}, 2);
        cv::line(img, {cx_img, cy_img - 10}, {cx_img, cy_img + 10}, {0, 0, 255}, 2);

        // --- drone_frame 投影点マーカー（白丸）: 矢印の始点 ---
        cv::circle(img, cv::Point(origin_x, origin_y), 5,
            drone_visible ? cv::Scalar(255, 255, 255) : cv::Scalar(100, 100, 100), -1);

        // --- /pid_vel 矢印（黄色）: PID が計算した速度指令 ---
        cv::arrowedLine(
            img,
            cv::Point(origin_x, origin_y),
            cv::Point(origin_x + static_cast<int>(pvx * kVelScale),
                      origin_y + static_cast<int>(pvy * kVelScale)),
            cv::Scalar(0, 220, 220), 2, cv::LINE_8, 0, 0.25
        );

        // --- /cmd_vel 矢印（緑）: ドローンへ実際に送る速度指令 ---
        cv::arrowedLine(
            img,
            cv::Point(origin_x, origin_y),
            cv::Point(origin_x + static_cast<int>(cvx * kVelScale),
                      origin_y + static_cast<int>(cvy * kVelScale)),
            cv::Scalar(0, 220, 0), 3, cv::LINE_8, 0, 0.25
        );

        // --- テキスト: 速度値（左上） ---
        char buf[128];
        std::snprintf(buf, sizeof(buf), "[PID] Vx:%+.2f  Vy:%+.2f", pvx, pvy);
        cv::putText(img, buf, cv::Point(10, 28),
            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 220, 220), 1, cv::LINE_AA);

        std::snprintf(buf, sizeof(buf), "[CMD] Vx:%+.2f  Vy:%+.2f  Vz:%+.2f", cvx, cvy, cvz);
        cv::putText(img, buf, cv::Point(10, 56),
            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 220, 0), 1, cv::LINE_AA);

        const std::string wz_str = (cwz > 0.01) ? "CCW" : (cwz < -0.01) ? " CW" : "---";
        std::snprintf(buf, sizeof(buf), "[CMD] Wyaw:%+.2f (%s)", cwz, wz_str.c_str());
        cv::putText(img, buf, cv::Point(10, 84),
            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 220, 0), 1, cv::LINE_AA);

        // --- 凡例（左下） ---
        cv::putText(img, "yellow:/pid_vel  green:/cmd_vel  red+:target  white-o:drone",
            cv::Point(10, img.rows - 10),
            cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(150, 150, 150), 1, cv::LINE_AA);

        image_pub_->publish(*cv_bridge::CvImage(msg->header, "bgr8", img).toImageMsg());
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr    cmd_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr    pid_vel_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr         image_pub_;
    std::unique_ptr<tf2_ros::Buffer>                              tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener>                   tf_listener_;

    geometry_msgs::msg::Twist last_cmd_vel_;
    geometry_msgs::msg::Twist last_pid_vel_;

    double fx_{1.0}, fy_{1.0}, cx_{0.0}, cy_{0.0};
    bool   camera_info_ready_{false};

    static constexpr int kVelScale = 150;
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelVisualizerNode>());
    rclcpp::shutdown();
    return 0;
}
