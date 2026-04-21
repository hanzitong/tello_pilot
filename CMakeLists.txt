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
find_package(visualization_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tello_msgs)


include_directories(include)
include_directories(srv)


add_executable(ar_detector_node
  src/ar_detector_node.cpp
)
ament_target_dependencies(ar_detector_node
  rclcpp
  OpenCV
  cv_bridge
  sensor_msgs
  geometry_msgs
  visualization_msgs
  image_transport
  tf2
  tf2_ros
  tf2_geometry_msgs
)
install(TARGETS ar_detector_node
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(ar_detector_node_2
  src/ar_detector_node_2.cpp
)
ament_target_dependencies(ar_detector_node_2
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
install(TARGETS ar_detector_node_2
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(cmd_multiplexer_node
  src/cmd_multiplexer_node.cpp
)
ament_target_dependencies(cmd_multiplexer_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tello_msgs
)
install(TARGETS cmd_multiplexer_node
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(pid_controller_node
  src/pid_controller_node.cpp
)
ament_target_dependencies(pid_controller_node
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
install(TARGETS pid_controller_node
  DESTINATION lib/${PROJECT_NAME}
)



add_executable(auto_lander_node
  src/auto_lander_node.cpp
)
ament_target_dependencies(auto_lander_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tello_msgs
)
install(TARGETS auto_lander_node
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(cmd_vel_visualizer_node
  src/cmd_vel_visualizer_node.cpp
)
ament_target_dependencies(cmd_vel_visualizer_node
  rclcpp
  OpenCV
  cv_bridge
  sensor_msgs
  geometry_msgs
)
install(TARGETS cmd_vel_visualizer_node
  DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY models worlds urdf
  DESTINATION share/${PROJECT_NAME}
)

## these programs can be ran with "ros2 run"
# install(
#   PROGRAMS
#     ar_marker/gen_aruco_marker.py
#     script/sim_tello_driver.py
#   DESTINATION lib/${PROJECT_NAME}
# )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

