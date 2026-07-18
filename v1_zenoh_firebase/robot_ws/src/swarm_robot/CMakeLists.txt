cmake_minimum_required(VERSION 3.8)
project(swarm_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR
   CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# cv_bridge — cari di path ROS 2 Jazzy
find_package(cv_bridge REQUIRED
  HINTS /opt/ros/jazzy/share/cv_bridge/cmake)

# =====================
# aruco_detector
# =====================
add_executable(aruco_detector
  src/aruco_detector.cpp)

target_include_directories(aruco_detector PRIVATE
  /opt/ros/jazzy/include/cv_bridge
  ${OpenCV_INCLUDE_DIRS})

ament_target_dependencies(aruco_detector
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  OpenCV)

# =====================
# follower_control
# =====================
add_executable(follower_control
  src/follower_control.cpp)

ament_target_dependencies(follower_control
  rclcpp
  geometry_msgs)

# =====================
# dijkstra_node
# =====================
add_executable(dijkstra_node
  src/dijkstra_node.cpp)

add_executable(rviz_goal_bridge
  src/rviz_goal_bridge.cpp)
ament_target_dependencies(rviz_goal_bridge
  rclcpp geometry_msgs)

ament_target_dependencies(dijkstra_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs)

# =====================
# target_follower
# =====================
add_executable(target_follower
  src/target_follower.cpp)

ament_target_dependencies(target_follower
  rclcpp
  geometry_msgs
  std_msgs)

# =====================
# Install executables
# =====================
install(TARGETS
  aruco_detector
  follower_control
  dijkstra_node
  rviz_goal_bridge
  target_follower
  DESTINATION lib/${PROJECT_NAME})

# =====================
# Install Python scripts
# =====================
install(PROGRAMS
  scripts/aruco_detector.py
  scripts/dijkstra_gui.py
  scripts/mission_gui.py
  DESTINATION lib/${PROJECT_NAME})

# =====================
# Install directories
# =====================
install(DIRECTORY
  meshes urdf launch worlds config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
