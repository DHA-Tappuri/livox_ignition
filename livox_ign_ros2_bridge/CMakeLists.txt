cmake_minimum_required(VERSION 3.8)
project(livox_ign_ros2_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(livox_ign_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

add_executable(livox_ign_ros2_bridge
    src/livox_ign_ros2_bridge.cpp
    ${livox_ign_msgs_INCLUDE_DIRS}/CustomMsg.pb.cc    
)

ament_target_dependencies(livox_ign_ros2_bridge
  rclcpp
  sensor_msgs
  livox_ign_msgs
  ignition-transport11
  livox_ros_driver2
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS livox_ign_ros2_bridge
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
