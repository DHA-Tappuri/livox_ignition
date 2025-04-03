cmake_minimum_required(VERSION 3.8)
project(livox_lidar_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-sensors6 REQUIRED)
find_package(ignition-sensors6 REQUIRED COMPONENTS lidar)
find_package(ignition-sensors6 REQUIRED COMPONENTS gpu_lidar)
find_package(ignition-msgs8 REQUIRED)
find_package(livox_ign_msgs REQUIRED)
find_package(OpenMP REQUIRED)
find_package(ament_index_cpp REQUIRED)


find_package(PkgConfig REQUIRED)
pkg_check_modules(IGN_SENSORS_LIDAR REQUIRED ignition-sensors6-lidar)
pkg_check_modules(IGN_SENSORS_GPU_LIDAR REQUIRED ignition-sensors6-gpu_lidar)


include_directories(
    include 
    ${IGN_SENSORS_LIDAR_INCLUDE_DIRS}
    ${IGN_SENSORS_GPU_LIDAR_INCLUDE_DIRS}
)

add_library(livox_lidar_plugin SHARED 
    src/LivoxLidar.cc
    ${livox_ign_msgs_INCLUDE_DIRS}/CustomMsg.pb.cc
)


if(OpenMP_CXX_FOUND)
  target_compile_options(livox_lidar_plugin 
      PUBLIC ${OpenMP_CXX_FLAGS}
  )
  target_link_libraries(livox_lidar_plugin 
      OpenMP::OpenMP_CXX
  )
endif()

target_include_directories(livox_lidar_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${livox_ign_msgs_INCLUDE_DIRS}
)

target_link_libraries(livox_lidar_plugin
  ignition-gazebo6::ignition-gazebo6
  ignition-msgs8::ignition-msgs8
  ignition-sensors6::ignition-sensors6
  ignition-sensors6::lidar
  ignition-sensors6::gpu_lidar 
  ament_index_cpp::ament_index_cpp    
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS livox_lidar_plugin 
    DESTINATION lib
)

ament_package()
install(DIRECTORY scan_mode DESTINATION share/${PROJECT_NAME})




