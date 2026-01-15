cmake_minimum_required(VERSION 3.5)
project(ros2_fra2mo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED) 
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


add_executable(dynamic_tf_publisher src/odom_bl_tf.cpp)
ament_target_dependencies(dynamic_tf_publisher 
  rclcpp 
  nav_msgs 
  geometry_msgs 
  tf2 
  tf2_ros
)

add_executable(fra2mo_object_manager src/fra2mo_object_manager.cpp)
ament_target_dependencies(fra2mo_object_manager
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  nav2_msgs
  rclcpp_action
  cv_bridge
  OpenCV
  control_msgs
  trajectory_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
)

include_directories(
  ${cv_bridge_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

install(DIRECTORY launch urdf models meshes worlds config src maps scripts rviz_conf
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS dynamic_tf_publisher fra2mo_object_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/reach_goal.py
  scripts/follow_waypoints.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
