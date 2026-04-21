// ar_detector_node_2.cpp
//
// ar_detector_node の改良版。
//
// 変更点（ar_detector_node との差分）:
//   1. /camera_info を購読して実際のカメラ行列・歪み係数を使用する
//   2. marker_23_frame の translation に tvec [m] をそのまま登録する
//      （ar_detector_node は pixel/100 の2D値を使っていた）
//   3. marker_23_frame の rotation は ArUco の生の姿勢（R_x180 補正なし）
//      drone_frame は launch で定義する静的 TF (marker_23_frame → drone_frame, Rx(π)) に委ねる
//
// TF ツリー:
//   camera_frame
//     └─(このノードが broadcast)─► marker_23_frame  [tvec m, 生の rotation]
//         └─(static TF, Rx(π))─► drone_frame

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>


class ArDetectorNode2 : public rclcpp::Node
{
public:
    ArDetectorNode2() : Node("ar_detector_2")
    {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        // KeepLast(N): 履歴ポリシー。キューに最新 N 件のみ保持し、超過分は破棄する。
        // depth=1 にすることで古いフレームがキューに溜まらず、常に最新データを処理できる。
        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/cam_image_raw",
            rclcpp::SensorDataQoS(rclcpp::KeepLast(1)),
            std::bind(&ArDetectorNode2::imageCallback, this, std::placeholders::_1)
        );

        camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera_info",
            rclcpp::SensorDataQoS(rclcpp::KeepLast(1)),
            std::bind(&ArDetectorNode2::cameraInfoCallback, this, std::placeholders::_1)
        );

        image_pub_ = create_publisher<sensor_msgs::msg::Image>(
            "/image_ar", rclcpp::QoS(1));

        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        parameters_ = cv::aruco::DetectorParameters::create();
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr         image_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>                tf_broadcaster_;

    cv::Ptr<cv::aruco::Dictionary>         dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> parameters_;

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    bool    camera_matrix_ready_{false};

    static constexpr double kMarkerLength = 0.03;  // [m]
    static constexpr int    kMarkerId     = 23;

private:
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (camera_matrix_ready_) return;

        // K: 行優先 3×3 行列
        camera_matrix_ = cv::Mat(3, 3, CV_64F);
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                camera_matrix_.at<double>(i, j) = msg->k[i * 3 + j];

        // D: 最初の 5 係数を使用（plumb_bob モデル）
        const int n = std::min(static_cast<int>(msg->d.size()), 5);
        dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
        for (int i = 0; i < n; ++i)
            dist_coeffs_.at<double>(i) = msg->d[i];

        camera_matrix_ready_ = true;
        RCLCPP_INFO(this->get_logger(),
            "Camera matrix initialized (fx=%.1f fy=%.1f cx=%.1f cy=%.1f)",
            camera_matrix_.at<double>(0, 0), camera_matrix_.at<double>(1, 1),
            camera_matrix_.at<double>(0, 2), camera_matrix_.at<double>(1, 2));
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!camera_matrix_ready_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Waiting for /camera_info ...");
            return;
        }

        auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        cv::Mat image_ar = cv_ptr->image;

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(image_ar, dictionary_, corners, ids, parameters_);

        const int cx = image_ar.cols / 2;
        const int cy = image_ar.rows / 2;

        if (!ids.empty()) {
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(
                corners, kMarkerLength, camera_matrix_, dist_coeffs_, rvecs, tvecs);

            for (size_t i = 0; i < ids.size(); ++i) {
                if (ids[i] != kMarkerId) continue;

                // rvec → 回転行列 → 四元数
                // R_x180 補正は行わない。drone_frame は静的 TF (Rx(π)) で定義する。
                cv::Mat R;
                cv::Rodrigues(rvecs[i], R);
                tf2::Matrix3x3 tf_R(
                    R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
                    R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
                    R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2)
                );
                tf2::Quaternion q;
                tf_R.getRotation(q);

                // marker_23_frame を broadcast（tvec [m] の生の3D値）
                // msg->header.stamp（カメラドライバの V4L2 タイムスタンプ）は
                // ROS クロック（this->now()）と異なる時刻源を持つ場合がある。
                // staleness チェックが同一クロック源で比較できるよう this->now() を使う。
                geometry_msgs::msg::TransformStamped ts;
                ts.header.stamp    = this->now();   // don't use msg's time-stamp which is generated by v4l2.
                ts.header.frame_id = "camera_frame";
                ts.child_frame_id  = "marker_" + std::to_string(ids[i]) + "_frame";
                ts.transform.translation.x = tvecs[i][0];
                ts.transform.translation.y = tvecs[i][1];
                ts.transform.translation.z = tvecs[i][2];
                ts.transform.rotation = tf2::toMsg(q);
                tf_broadcaster_->sendTransform(ts);

                RCLCPP_DEBUG(this->get_logger(),
                    "marker %d: t=(%.3f, %.3f, %.3f) [m]",
                    ids[i], tvecs[i][0], tvecs[i][1], tvecs[i][2]);

                // 可視化: 3D 座標軸
                cv::aruco::drawAxis(
                    image_ar, camera_matrix_, dist_coeffs_,
                    rvecs[i], tvecs[i], kMarkerLength * 0.5f);

                // 可視化: 画像中心からマーカー中心への矢印
                const int px = cvRound(
                    (corners[i][0].x + corners[i][1].x + corners[i][2].x + corners[i][3].x) * 0.25f);
                const int py = cvRound(
                    (corners[i][0].y + corners[i][1].y + corners[i][2].y + corners[i][3].y) * 0.25f);
                cv::arrowedLine(image_ar,
                    cv::Point(cx, cy), cv::Point(px, py),
                    cv::Scalar(100, 20, 200), 3, cv::LINE_8, 0, 0.1);

                break;  // marker 23 は1枚だけを対象にする
            }

            // 可視化: バウンディングボックス（全検出マーカー）
            for (const auto& mc : corners) {
                for (int j = 0; j < 4; ++j)
                    cv::line(image_ar, mc[j], mc[(j + 1) % 4], cv::Scalar(255, 255, 0), 7);
            }
        }

        // 可視化: 中心十字
        cv::line(image_ar, {cx - 10, cy}, {cx + 10, cy}, {0, 0, 255}, 2);
        cv::line(image_ar, {cx, cy - 10}, {cx, cy + 10}, {0, 0, 255}, 2);

        image_pub_->publish(*cv_bridge::CvImage(msg->header, "bgr8", image_ar).toImageMsg());
    }

};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArDetectorNode2>());
    rclcpp::shutdown();
    return 0;
}
