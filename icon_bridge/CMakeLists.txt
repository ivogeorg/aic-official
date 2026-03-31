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
  absl::time
  agent_bridge_proto
  flowstate_ros_bridge::flowstate_ros_builtin_bridges
  rclcpp::rclcpp
  rclcpp_components
  ${aic_control_interfaces_TARGETS}
)
target_compile_features(icon_bridge PUBLIC cxx_std_17)

ament_export_libraries(icon_bridge)
pluginlib_export_plugin_description_file(flowstate_ros_bridge plugins.xml)

#===============================================================================

set(SERVICE_NAME "aic_flowstate_ros_bridge")

intrinsic_sdk_generate_service_manifest(
  SERVICE_NAME ${SERVICE_NAME}
  MANIFEST ${SERVICE_NAME}.manifest.textproto
  PARAMETER_DESCRIPTOR ${SERVICE_NAME}.proto
  DEFAULT_CONFIGURATION ${SERVICE_NAME}_default_config.pbtxt
  PROTOS_TARGET ${SERVICE_NAME}_protos
)
add_executable(${SERVICE_NAME}_main
 src/${SERVICE_NAME}_main.cpp
)
target_include_directories(${SERVICE_NAME}_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(${SERVICE_NAME}_main
  ${SERVICE_NAME}_component
  ${SERVICE_NAME}_protos
  rclcpp_components::component
)

# Generate the manifest and default config binarypb files
add_dependencies(${SERVICE_NAME}_main
  ${SERVICE_NAME}_manifest
  ${SERVICE_NAME}_default_config
)

#===============================================================================

install(TARGETS
  icon_bridge
  agent_bridge_proto
  ${SERVICE_NAME}_main
  DESTINATION lib/${PROJECT_NAME}
)

# Install proto descriptor, manifest, and config files
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${SERVICE_NAME}_protos.desc
  ${CMAKE_CURRENT_BINARY_DIR}/service_manifest.binarypb
  ${CMAKE_CURRENT_BINARY_DIR}/default_config.binarypb
  DESTINATION share/${PROJECT_NAME}/${SERVICE_NAME}
)

install(DIRECTORY config launch proto
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/test_icon_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
