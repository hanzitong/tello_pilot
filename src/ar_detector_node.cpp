/*
auto_detector_tf2.cpp

*/

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


#include <geometry_msgs/msg/transform_stamped.hpp>
// pose stamped !! (position: start point, orientation: actual pose)
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>



class ArucoDetectorNode : public rclcpp::Node
{
public:
  ArucoDetectorNode() : Node("ar_detector")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/cam_image_raw", 10,
      std::bind(&ArucoDetectorNode::imageCallback, this, std::placeholders::_1)
    );
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("/image_ar", 10);

    land_direction_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/land_direction", 10);
    // land_direction_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/land_direction", 10);
    // land_direction_pub_ = create_publisher<visualization_msgs::msg::Marker>("/land_direction", 10);

    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    parameters_ = cv::aruco::DetectorParameters::create();
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Convert ROS image topic to OpenCV image matrix
    auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat image_ar = cv_ptr->image;

    // Detect marker
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(image_ar, dictionary_, corners, ids, parameters_);
    geometry_msgs::msg::Twist twist_msg;

    // calculate image center position [pixel]
    int cx = image_ar.cols / 2;
    int cy = image_ar.rows / 2;
    // RCLCPP_INFO(this->get_logger(), "camera image center: %d, %d", cx, cy);

    // Estimage AR pose & translation > Register
    if (!ids.empty()) {
      // calculate marker center position[pixel]
      int ar_pixel_posx = cvRound((corners[0][0].x + corners[0][1].x + corners[0][2].x + corners[0][3].x) * 0.25);
      int ar_pixel_posy = cvRound((corners[0][0].y + corners[0][1].y + corners[0][2].y + corners[0][3].y) * 0.25);
      // RCLCPP_INFO(this->get_logger(), "ar_pixel(x): %d", ar_pixel_posx);
      // RCLCPP_INFO(this->get_logger(), "ar_pixel(y): %d", ar_pixel_posy);

      // Skip camera caribration step
      cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
      cv::Mat distCoeffs   = cv::Mat::zeros(5, 1, CV_64F);
      double markerLength  = 0.5; // test:0.03[m]

      // Estimate AR marker poses
      std::vector<cv::Vec3d> rvecs, tvecs;  // rvecs is needed for calculate rotation
      cv::aruco::estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);
      // RCLCPP_INFO(this->get_logger(), "(AR POSE)\n  rvecs: %f, %f, %f\n  tvecs: %f , %f, %f",
      //   rvecs[0][0], rvecs[0][1], rvecs[0][2], tvecs[0][0], tvecs[0][1], tvecs[0][2]);

      // Register tf2 of detected AR marker
      for (size_t i = 0; i < 1; ++i) {  // care about only the first AR marker now
        // Put translational and rotational elements into tf-msg and broadcast it
        /* BE CAREFULL !!!  by Han Zitong :
         When we use this system in the intended way, 
         the camera_frame and aruco_marker frame are placed flipped position for each other,
         so x and z axis look like inversed in only 2D image (It's not a bug).
         When we convert 3D pose to 2D pose, x-axis will be flipped !!
         For avoiding tf confusion, I registered tf in the correct 3D frame translation.
         This isn't a bug but when we control drone in 2D PID, flipped x causes sign problem.
         Please keep it in your mind !!
        */
        geometry_msgs::msg::TransformStamped tf_ar;
        tf_ar.header.stamp    = msg->header.stamp;
        tf_ar.header.frame_id = "camera_cv_frame";
        tf_ar.child_frame_id  = "marker_" + std::to_string(ids[i]) + "_frame"; // test marker: 23
        tf_ar.transform.translation.x = ar_pixel_posx / 100.; // [100 * pixel]
        tf_ar.transform.translation.y = ar_pixel_posy / 100.; // [100 * pixel]
        tf_ar.transform.translation.z = 0;

        cv::Mat R_ar;
        cv::Rodrigues(rvecs[i], R_ar);  // get rotation matrix from rvecs
        cv::Mat R_x180 = (cv::Mat_<double>(3,3) << \
          1, 0, 0,
          0, -1, 0,
          0, 0, -1
        );  // rotation matrix for flip ar-frame along x-axis
        cv::Mat R_ar_fixed = R_ar * R_x180;
        tf2::Matrix3x3 tf_R_ar_fixed( // change variable-type from R_ar_fixed to tf_R_ar_fixed
          R_ar_fixed.at<double>(0,0), R_ar_fixed.at<double>(0,1), R_ar_fixed.at<double>(0,2),
          R_ar_fixed.at<double>(1,0), R_ar_fixed.at<double>(1,1), R_ar_fixed.at<double>(1,2),
          R_ar_fixed.at<double>(2,0), R_ar_fixed.at<double>(2,1), R_ar_fixed.at<double>(2,2)
        );
        tf2::Quaternion q_ar;
        tf_R_ar_fixed.getRotation(q_ar);  // get quarternion(q_ar) from rotation matrix(tf_R_ar_fixed)
        tf_ar.transform.rotation = tf2::toMsg(q_ar);
        tf_broadcaster_->sendTransform(tf_ar);

        
        // publish land direction;
        geometry_msgs::msg::Vector3Stamped land_dir;
        // geometry_msgs::msg::PoseStamped land_dir;
        // visualization_msgs::msg::Marker land_dir;
        land_dir.header.stamp = msg->header.stamp;
        land_dir.header.frame_id = "camera_center_frame";
        // translation.x/y は [pixel/100] 単位。cx/cy も揃えて [pixel/100] に変換して引く
        land_dir.vector.x = -1. * (tf_ar.transform.translation.x - cx / 100.);
        land_dir.vector.y = -1. * (tf_ar.transform.translation.y - cy / 100.);
        land_dir.vector.z = 0.;
        land_direction_pub_->publish(land_dir);


        // Visualize the detected 2D(x,y-axis) pose
        float axisLength = static_cast<float>(markerLength * 0.5);
        cv::aruco::drawAxis(image_ar, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], axisLength);

        // Visualize an arrow of translation from camera-frame to AR-frame 
        cv::arrowedLine(
          image_ar,
          cv::Point(cx, cy),      // unit of cx & cy is [pixel], not [m]. careful!
          cv::Point(ar_pixel_posx, ar_pixel_posy),  // cv::Point_<tp> is just a struct. careful !!
          cv::Scalar(100, 20, 200),  // color 3 channel RGB
          3,   // thickness
          cv::LINE_8,
          0,
          0.1
        );

        // Visualize a bounding box of detected AR marker
        const cv::Scalar box_color(255, 255, 0);
        const int box_thickness = 7;
        for (const auto &mc : corners) {
          for (int i = 0; i < 4; ++i) {
            cv::line(image_ar, mc[i], mc[(i+1)%4], box_color, box_thickness);
          }
        }

      } // for-loop to register tf2


    } // if(!ids.empty()) clause

    
    // Visualize a cross mark at the center of image
    cv::line(image_ar, {cx - 10, cy}, {cx + 10, cy}, {0,0,255}, 2);
    cv::line(image_ar, {cx, cy - 10}, {cx, cy + 10}, {0,0,255}, 2);


    // publish
    image_pub_->publish(*cv_bridge::CvImage(msg->header, "bgr8", image_ar).toImageMsg());

  } // image_callback() function

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr land_direction_pub_;
  // rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr land_direction_pub_;
  // rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr land_direction_pub_;

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> parameters_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
}; // ArucoDetectorNode class


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoDetectorNode>());
  rclcpp::shutdown();
  return 0;
}


