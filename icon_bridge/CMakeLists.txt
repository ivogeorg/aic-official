cmake_minimum_required(VERSION 3.8)
project(icon_bridge)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)

set(dep_pkgs
  absl
  aic_control_interfaces
  flowstate_ros_bridge
  pluginlib
  rclcpp
)
foreach(pkg ${dep_pkgs})
  find_package(${pkg} REQUIRED)
endforeach()

find_package(Protobuf REQUIRED)

#==================================

intrinsic_sdk_protobuf_generate(
  NAME agent_bridge
  TARGET agent_bridge_proto
  SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/proto/agent_bridge.proto
  IMPORT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/proto
  DESCRIPTOR_SET_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/agent_bridge.desc"
)

add_library(icon_bridge SHARED
  src/icon_bridge.cpp
)
target_include_directories(icon_bridge
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${pluginlib_INCLUDE_DIRS}
)
target_link_libraries(icon_bridge
  absl::log
  agent_bridge_proto
  flowstate_ros_bridge::flowstate_ros_builtin_bridges
  rclcpp::rclcpp
  ${aic_control_interfaces_TARGETS}
)
target_compile_features(icon_bridge PUBLIC cxx_std_17)

ament_export_libraries(icon_bridge)
pluginlib_export_plugin_description_file(flowstate_ros_bridge plugins.xml)

#===============================================================================

install(TARGETS
  icon_bridge
  agent_bridge_proto
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch proto
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/test_icon_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
