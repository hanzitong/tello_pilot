cmake_minimum_required(VERSION 3.8)
project(tello_pilot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(ar_detector
  src/ar_detector.cpp
)
ament_target_dependencies(ar_detector
  rclcpp
  OpenCV
  cv_bridge
  sensor_msgs
  geometry_msgs
  image_transport
  tf2
  tf2_ros
  tf2_geometry_msgs
)
install(TARGETS ar_detector
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(ar_detector_tf
  # src/ar_detector.cpp
  src/ar_detector_tf.cpp
)
ament_target_dependencies(ar_detector_tf
  rclcpp
  OpenCV
  cv_bridge
  sensor_msgs
  geometry_msgs
  image_transport
  tf2
  tf2_ros
  tf2_geometry_msgs
)
install(TARGETS ar_detector_tf
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(cmd_multiplexer
  src/cmd_multiplexer.cpp
)
ament_target_dependencies(cmd_multiplexer
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
install(TARGETS cmd_multiplexer
  DESTINATION lib/${PROJECT_NAME}
)


# launch/camera_test.launch.xml
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

