
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>


class ArucoDetectorNode : public rclcpp::Node
{
public:
  ArucoDetectorNode()
  : Node("aruco_transform_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_))
  {
    // Subscriptions & Publishers
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/cam_image_raw", 10,
      std::bind(&ArucoDetectorNode::imageCallback, this, std::placeholders::_1));
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("/image_ar", 5);
    twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("aruco_twist", 5);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // ARuco setup
    dictionary_  = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    parameters_  = cv::aruco::DetectorParameters::create();
    marker_length_ = 0.1;  // meters

    // Camera calibration (replace with real values)
    camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
    dist_coeffs_   = cv::Mat::zeros(5, 1, CV_64F);
    camera_frame_  = "camera_link";
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Convert ROS image to OpenCV
    auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat frame = cv_ptr->image;

    // Image center = camera frame origin
    int cx = frame.cols / 2;
    int cy = frame.rows / 2;

    // Detect markers
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(frame, dictionary_, corners, ids, parameters_);

    geometry_msgs::msg::Twist twist_msg;

    if (!ids.empty()) {
      // Estimate pose
      std::vector<cv::Vec3d> rvecs, tvecs;
      cv::aruco::estimatePoseSingleMarkers(
        corners, marker_length_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

      for (size_t i = 0; i < ids.size(); ++i) {
        // Compute marker center in pixels
        const auto &mc = corners[i];
        double mx = (mc[0].x + mc[1].x + mc[2].x + mc[3].x) / 4.0;
        double my = (mc[0].y + mc[1].y + mc[2].y + mc[3].y) / 4.0;

        // Broadcast TF: camera -> marker
        geometry_msgs::msg::TransformStamped tfs;
        tfs.header.stamp    = msg->header.stamp;
        tfs.header.frame_id = camera_frame_;
        tfs.child_frame_id  = "aruco_marker_" + std::to_string(ids[i]);

        tfs.transform.translation.x = tvecs[i][0];
        tfs.transform.translation.y = tvecs[i][1];
        tfs.transform.translation.z = tvecs[i][2];

        // rvec -> rotation matrix -> quaternion
        cv::Mat R;
        cv::Rodrigues(rvecs[i], R);
        tf2::Matrix3x3 tfR(
          R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
          R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
          R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
        tf2::Quaternion q;
        tfR.getRotation(q);
        tfs.transform.rotation = tf2::toMsg(q);

        tf_broadcaster_->sendTransform(tfs);

        // Transform a point from camera to marker using tf2
        geometry_msgs::msg::PointStamped p_cam, p_marker;
        p_cam.header.stamp    = msg->header.stamp;
        p_cam.header.frame_id = camera_frame_;
        p_cam.point.x = mx - cx;
        p_cam.point.y = my - cy;
        p_cam.point.z = tvecs[i][2];

        try {
          p_marker = tf_buffer_.transform(p_cam, tfs.child_frame_id, tf2::durationFromSec(0.1));
          RCLCPP_INFO(get_logger(),
            "[Marker %d] p_marker = [%.3f, %.3f, %.3f]",
            ids[i], p_marker.point.x, p_marker.point.y, p_marker.point.z);
        } catch (const tf2::TransformException &ex) {
          RCLCPP_WARN(get_logger(), "Could not transform point: %s", ex.what());
        }

        // For the first detected marker, fill Twist
        if (i == 0) {
          twist_msg.linear.x  = p_cam.point.x;
          twist_msg.linear.y  = p_cam.point.y;
          twist_msg.linear.z  = p_cam.point.z;
          twist_msg.angular.x = rvecs[0][0];
          twist_msg.angular.y = rvecs[0][1];
          twist_msg.angular.z = rvecs[0][2];
        }
      }
    }

    // Visualization: cross, boxes, axes
    // (省略: 同様に描画コードを追加)
    // Visualizetion();

    // Publish annotated image and twist
    auto out_img = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
    image_pub_->publish(*out_img);
    twist_pub_->publish(twist_msg);
  }

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

  // tf2
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  tf2_ros::Buffer                     tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ARuco
  cv::Ptr<cv::aruco::Dictionary>      dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> parameters_;
  double                              marker_length_;

  // Camera calibration
  cv::Mat camera_matrix_, dist_coeffs_;
  std::string camera_frame_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoDetectorNode>());
  rclcpp::shutdown();
  return 0;
}

