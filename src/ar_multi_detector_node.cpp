// ar_multi_detector_node.cpp
//
// 4枚のARマーカー（各プロペラ下）からドローン中心を推定し drone_frame を broadcast するノード。
// 検出された全マーカーそれぞれからドローン中心を推定し、その平均を drone_frame として出力する。
// 1枚認識失敗しても他のマーカーから継続動作できる。
//
// ar_single_detector との違い:
//   - drone_frame を camera_frame の子として直接 broadcast する（静的 TF 不要）
//   - 各マーカーの個別 TF（marker_{id}_frame）もデバッグ用に broadcast する
//
// TF ツリー（このノードが broadcast するもの）:
//   camera_frame ─(毎フレーム)─► marker_{id}_frame  （各検出マーカー、デバッグ用）
//   camera_frame ─(毎フレーム)─► drone_frame         （全検出マーカーの推定平均）
//
// ドローン底面の物理配置:
//   各マーカーは drone_frame 原点（4プロペラの重心）から kMarkerDist [m] の位置に取り付け。
//   プロペラは Z 軸方向から見て 45°/135°/225°/315° の点対称配置。
//   マーカーの向きは ar_single と同様（Z 軸がカメラ方向＝下向き、Rx(π) で drone_frame に対応）。
//
// ドローン中心の推定式:
//   R_cam_drone = R_cam_marker * Rx(π)
//   pos_drone   = tvec - R_cam_drone * p_i
//   （p_i: drone_frame でのマーカー i の取り付け位置）

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <map>
#include <cmath>
#include <vector>

static constexpr double kMarkerLength = 0.03;  // マーカー1辺の長さ [m]
static constexpr double kMarkerDist   = 0.06;  // ドローン中心から各マーカーまでの距離 [m]

// マーカーID → drone_frame 上の取り付け位置の対応
// プロペラは 45°/135°/225°/315° 方向。各成分 = kMarkerDist / √2
// ※ 実機の ID →位置の対応に合わせて調整すること
struct MarkerDef {
    int    id;
    double nx, ny;  // drone_frame X/Y 方向の符号（実距離は kMarkerDist / √2 を乗算）
};
static constexpr MarkerDef kMarkerDefs[] = {
    { 9,  1,  1},   // drone_frame +X+Y 方向
    {20, -1,  1},   // drone_frame -X+Y 方向
    {21, -1, -1},   // drone_frame -X-Y 方向
    {26,  1, -1},   // drone_frame +X-Y 方向
};


class ArMultiDetectorNode : public rclcpp::Node
{
public:
    ArMultiDetectorNode() : Node("ar_multi_detector")
    {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/cam_image_raw",
            rclcpp::SensorDataQoS(rclcpp::KeepLast(1)),
            std::bind(&ArMultiDetectorNode::imageCallback, this, std::placeholders::_1)
        );
        camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera_info",
            rclcpp::SensorDataQoS(rclcpp::KeepLast(1)),
            std::bind(&ArMultiDetectorNode::cameraInfoCallback, this, std::placeholders::_1)
        );
        image_pub_ = create_publisher<sensor_msgs::msg::Image>(
            "/image_ar", rclcpp::QoS(1));

        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        parameters_ = cv::aruco::DetectorParameters::create();

        // ID → drone_frame オフセットマップを構築
        const double d = kMarkerDist / std::sqrt(2.0);
        for (const auto& def : kMarkerDefs) {
            marker_offset_[def.id] = tf2::Vector3(def.nx * d, def.ny * d, 0.0);
        }
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

    std::map<int, tf2::Vector3> marker_offset_;


private:
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (camera_matrix_ready_) return;

        camera_matrix_ = cv::Mat(3, 3, CV_64F);
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                camera_matrix_.at<double>(i, j) = msg->k[i * 3 + j];

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
        cv::Mat& img = cv_ptr->image;

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(img, dictionary_, corners, ids, parameters_);

        std::vector<cv::Vec3d> rvecs, tvecs;
        if (!ids.empty()) {
            cv::aruco::estimatePoseSingleMarkers(
                corners, kMarkerLength, camera_matrix_, dist_coeffs_, rvecs, tvecs);
        }

        // Rx(π): マーカー座標系 → ドローン座標系の回転（Z 軸反転）
        static const tf2::Matrix3x3 Rx180(1, 0, 0,  0, -1, 0,  0, 0, -1);

        tf2::Vector3    pos_sum(0.0, 0.0, 0.0);
        tf2::Quaternion q_sum(0.0, 0.0, 0.0, 0.0);
        int             count = 0;

        const rclcpp::Time now = this->now();

        for (size_t i = 0; i < ids.size(); ++i) {
            auto it = marker_offset_.find(ids[i]);
            if (it == marker_offset_.end()) continue;

            const tf2::Vector3& p = it->second;

            // rvec → 回転行列 (camera_frame → marker_frame)
            cv::Mat R_cv;
            cv::Rodrigues(rvecs[i], R_cv);
            const tf2::Matrix3x3 R_cam_marker(
                R_cv.at<double>(0,0), R_cv.at<double>(0,1), R_cv.at<double>(0,2),
                R_cv.at<double>(1,0), R_cv.at<double>(1,1), R_cv.at<double>(1,2),
                R_cv.at<double>(2,0), R_cv.at<double>(2,1), R_cv.at<double>(2,2)
            );

            // R_cam_drone = R_cam_marker * Rx(π)
            const tf2::Matrix3x3 R_cam_drone = R_cam_marker * Rx180;

            // ドローン中心推定: tvec - R_cam_drone * p
            const tf2::Vector3 tvec(tvecs[i][0], tvecs[i][1], tvecs[i][2]);
            pos_sum += tvec - R_cam_drone * p;

            // 四元数の平均: q と -q は同じ回転なので符号を統一してから加算する
            tf2::Quaternion q;
            R_cam_drone.getRotation(q);
            if (count > 0 && q.dot(q_sum) < 0.0) q = tf2::Quaternion(-q.x(), -q.y(), -q.z(), -q.w());
            q_sum = q_sum + tf2::Quaternion(q.x(), q.y(), q.z(), q.w());

            count++;

            // デバッグ用: 個別マーカーの TF broadcast（生の姿勢、Rx(π) なし）
            // {
            //     geometry_msgs::msg::TransformStamped ts;
            //     ts.header.stamp    = now;
            //     ts.header.frame_id = "camera_frame";
            //     ts.child_frame_id  = "marker_" + std::to_string(ids[i]) + "_frame";
            //     ts.transform.translation.x = tvecs[i][0];
            //     ts.transform.translation.y = tvecs[i][1];
            //     ts.transform.translation.z = tvecs[i][2];
            //     tf2::Quaternion q_raw;
            //     R_cam_marker.getRotation(q_raw);
            //     ts.transform.rotation = tf2::toMsg(q_raw);
            //     tf_broadcaster_->sendTransform(ts);
            // }

            // 可視化: 3D 座標軸とバウンディングボックス
            cv::aruco::drawAxis(img, camera_matrix_, dist_coeffs_,
                rvecs[i], tvecs[i], kMarkerLength * 0.5f);
        }

        if (!ids.empty()) {
            cv::aruco::drawDetectedMarkers(img, corners, cv::noArray(), cv::Scalar(255, 255, 0));
        }

        // 画像中心の十字（制御目標）
        const int cx = img.cols / 2;
        const int cy = img.rows / 2;
        cv::line(img, {cx - 10, cy}, {cx + 10, cy}, {0, 0, 255}, 2);
        cv::line(img, {cx, cy - 10}, {cx, cy + 10}, {0, 0, 255}, 2);

        if (count > 0) {
            // 位置: 算術平均
            const tf2::Vector3 pos = pos_sum / static_cast<double>(count);

            // 姿勢: 正規化で平均四元数を得る
            tf2::Quaternion q_avg = q_sum;
            q_avg.normalize();

            // drone_frame を camera_frame の子として直接 broadcast
            geometry_msgs::msg::TransformStamped ts;
            ts.header.stamp    = now;
            ts.header.frame_id = "camera_frame";
            ts.child_frame_id  = "drone_frame";
            ts.transform.translation.x = pos.x();
            ts.transform.translation.y = pos.y();
            ts.transform.translation.z = pos.z();
            ts.transform.rotation = tf2::toMsg(q_avg);
            tf_broadcaster_->sendTransform(ts);

            RCLCPP_DEBUG(this->get_logger(),
                "drone_frame from %d markers: (%.3f, %.3f, %.3f) [m]",
                count, pos.x(), pos.y(), pos.z());
        }

        image_pub_->publish(*cv_bridge::CvImage(msg->header, "bgr8", img).toImageMsg());
    }

};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArMultiDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
