cmake_minimum_required(VERSION 3.8)
project(slam_localization)

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


# Use pkg-config to find MAVSDK
find_package(PkgConfig REQUIRED)
pkg_check_modules(MAVSDK REQUIRED IMPORTED_TARGET mavsdk)

# Add the executable
add_executable(slam_node src/slam_localization.cpp)
ament_target_dependencies(slam_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(slam_node
  PkgConfig::MAVSDK
)

install(TARGETS
  slam_node
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()

