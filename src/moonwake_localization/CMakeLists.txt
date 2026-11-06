cmake_minimum_required(VERSION 3.8)
project(moonwake_localization)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(pointcloud_preflight_node
  src/pointcloud_preflight_node.cpp
)

target_compile_features(pointcloud_preflight_node PRIVATE cxx_std_17)
target_include_directories(pointcloud_preflight_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(pointcloud_preflight_node
  rclcpp
  sensor_msgs
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(pointcloud_preflight_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS pointcloud_preflight_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
