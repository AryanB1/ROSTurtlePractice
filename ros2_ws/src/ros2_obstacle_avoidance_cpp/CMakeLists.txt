cmake_minimum_required(VERSION 3.5)
project(ros2_obstacle_avoidance_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Make sure ROS 2 packages can be found
if(NOT DEFINED CMAKE_PREFIX_PATH)
  if(DEFINED ENV{AMENT_PREFIX_PATH})
    message(STATUS "Using AMENT_PREFIX_PATH to set CMAKE_PREFIX_PATH")
    set(CMAKE_PREFIX_PATH $ENV{AMENT_PREFIX_PATH})
  else()
    message(WARNING "Neither CMAKE_PREFIX_PATH nor AMENT_PREFIX_PATH is set. ROS 2 packages may not be found.")
  endif()
endif()
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

# Include directories - Add this before add_executable
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${turtlesim_INCLUDE_DIRS}
)

add_executable(avoid_node src/avoid_node.cpp)
ament_target_dependencies(avoid_node rclcpp geometry_msgs turtlesim)

# Install executable
install(TARGETS
  avoid_node
  DESTINATION lib/${PROJECT_NAME})

# Export package dependencies
ament_export_dependencies(rclcpp geometry_msgs turtlesim)

# Package registration
ament_package()
