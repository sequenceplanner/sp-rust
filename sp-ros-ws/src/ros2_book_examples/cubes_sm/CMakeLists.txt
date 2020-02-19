cmake_minimum_required(VERSION 3.5)
project(cubes_sm)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ros2_scene_manipulation_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  include
  libs
)

# Service

add_executable(service_main
  src/main.cpp
  src/scene_manipulation_service.cpp
)

ament_target_dependencies(service_main
  rclcpp
  ros2_scene_manipulation_msgs
  geometry_msgs
  tf2_ros
)

install(TARGETS service_main
  DESTINATION lib/${PROJECT_NAME})

# Client

add_executable(example_smc_client
  src/example_smc_client.cpp
)

ament_target_dependencies(example_smc_client
  rclcpp
  ros2_scene_manipulation_msgs
  geometry_msgs
  tf2_ros
)

install(TARGETS example_smc_client
  DESTINATION lib/${PROJECT_NAME})


ament_package()