cmake_minimum_required(VERSION 3.5)
project(ros2_dorna_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(ros2_dorna_msgs
  "msg/DornaSPToEsd.msg"
  "msg/DornaEsdToSP.msg"
  "msg/DornaGuiToEsd.msg"
  "msg/DornaEsdToGui.msg"
  "msg/DornaGuiToUtils.msg"
  "msg/DornaUtilsToGui.msg"
  "msg/DornaSPToUtils.msg"
  "msg/DornaUtilsToSP.msg"
  DEPENDENCIES builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # remove the line when a copyright and license is present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # remove the line when this package is a git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#install(
#  FILES message_x_mapping_rules.yaml
#  DESTINATION share/${PROJECT_NAME})

#install(
#  FILES message_y_mapping_rules.yaml
#  DESTINATION share/${PROJECT_NAME})

ament_package()
