// aruco_detector_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>


class ArucoDetectorNode : public rclcpp::Node
{
public:
  ArucoDetectorNode(): Node("aruco_detector_node")
  {
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/cam_image_raw", 10,
      std::bind(&ArucoDetectorNode::imageCallback, this, std::placeholders::_1));
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("/image_ar", 10);
    twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("aruco_twist", 10);

    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    parameters_ = cv::aruco::DetectorParameters::create();
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Convert ROS image to OpenCV Mat
    auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat frame = cv_ptr->image;

    // Detect AR markers
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(frame, dictionary_, corners, ids, parameters_);

    // Calculate image center
    const int cx = frame.cols / 2;
    const int cy = frame.rows / 2;

    // Draw red cross at image center
    cv::line(frame, {cx - 10, cy}, {cx + 10, cy}, {0, 0, 255}, 2);
    cv::line(frame, {cx, cy - 10}, {cx, cy + 10}, {0, 0, 255}, 2);

    // Draw thick boxes around detected markers
    const cv::Scalar box_color(255, 255, 0);  // cyan (B, G, R)
    const int box_thickness = 10;
    for (const auto &marker_corners : corners) {
      for (int i = 0; i < 4; ++i) {
        const auto &p1 = marker_corners[i];
        const auto &p2 = marker_corners[(i + 1) % 4];
        cv::line(frame, p1, p2, box_color, box_thickness);
      }
    }

    // Prepare twist message
    geometry_msgs::msg::Twist twist_msg{};

    if (!ids.empty()) {
      // Compute marker center (using first marker)
      const auto &mc = corners[0];
      // const float mx = (mc[0].x + mc[1].x + mc[2].x + mc[3].x) / 4.0f;
      // const float my = (mc[0].y + mc[1].y + mc[2].y + mc[3].y) / 4.0f;
      const int mx = static_cast<int>( (mc[0].x + mc[1].x + mc[2].x + mc[3].x) / 4.0f );
      const int my = static_cast<int>( (mc[0].y + mc[1].y + mc[2].y + mc[3].y) / 4.0f );

      // Draw marker center
      cv::circle(frame, cv::Point(mx, my), 5, {255, 0, 0}, -1);

      // Connect image center to marker center
      cv::line(frame, {cx, cy}, {mx, my}, {0, 255, 255}, 2);

      // preparation : Estimate pose and draw axis
      cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
      cv::Mat distCoeffs   = cv::Mat::zeros(5, 1, CV_64F);
      double markerLength  = 0.1;  // meters
      std::vector<cv::Vec3d> rvecs, tvecs;

      // Estimate pose
      cv::aruco::estimatePoseSingleMarkers(
        corners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);

      // Draw axis
      const float axisLength = markerLength * 0.5f;
      cv::aruco::drawAxis(     // blue line here 
        frame, cameraMatrix, distCoeffs,
        rvecs[0], tvecs[0], axisLength);

      // Fill twist with position offset (pixels) and pose
      twist_msg.linear.x  = mx - cx;
      twist_msg.linear.y  = my - cy;
      twist_msg.linear.z  = tvecs[0][2];
      twist_msg.angular.x = rvecs[0][0];
      twist_msg.angular.y = rvecs[0][1];
      twist_msg.angular.z = rvecs[0][2];
    }

    // Publish annotated image
    auto out_img = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
    image_pub_->publish(*out_img);

    // Publish Twist
    twist_pub_->publish(twist_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> parameters_;
};



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoDetectorNode>());
  rclcpp::shutdown();
  return 0;
}


