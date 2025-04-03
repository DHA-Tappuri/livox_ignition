cmake_minimum_required(VERSION 3.8)
project(livox_ign_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Protobuf REQUIRED)

set(PROTO_FILES ${CMAKE_CURRENT_SOURCE_DIR}/proto/CustomMsg.proto)
set(PROTO_GEN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/proto")
file(MAKE_DIRECTORY ${PROTO_GEN_DIR})

add_custom_command(
    OUTPUT ${PROTO_GEN_DIR}/CustomMsg.pb.cc ${PROTO_GEN_DIR}/CustomMsg.pb.h
    COMMAND ${Protobuf_PROTOC_EXECUTABLE}
    ARGS --proto_path=${CMAKE_CURRENT_SOURCE_DIR}/proto
         --proto_path=/usr/include/ignition/msgs8
         --experimental_allow_proto3_optional         
         --cpp_out=${PROTO_GEN_DIR}
         ${PROTO_FILES}
    DEPENDS ${PROTO_FILES}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(livox_ign_msgs INTERFACE)

target_include_directories(livox_ign_msgs INTERFACE
    $<BUILD_INTERFACE:${PROTO_GEN_DIR}>
    $<INSTALL_INTERFACE:include>
)

ament_export_include_directories(${PROTO_GEN_DIR})

install(DIRECTORY ${PROTO_GEN_DIR}/
    DESTINATION include
)


ament_package()
