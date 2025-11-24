cmake_minimum_required(VERSION 3.8)
project(gz_lidar_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs11 REQUIRED)

add_executable(lidar_bridge_node src/lidar_bridge_node.cpp)

ament_target_dependencies(lidar_bridge_node
  rclcpp
  sensor_msgs
  rosgraph_msgs
)

target_link_libraries(lidar_bridge_node
  gz-transport13::gz-transport13
  gz-msgs11::gz-msgs11
)

install(TARGETS
  lidar_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
