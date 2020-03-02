cmake_minimum_required(VERSION 3.5)
project(rslidar_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(libpcap_LIBRARIES -lpcap)                                                                                         


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rslidar_msgs REQUIRED)
find_package(rslidar_pointcloud REQUIRED)
find_package(diagnostic_updater REQUIRED)

# Node
add_executable(rslidar_node src/rslidar_node.cpp src/rsdriver.cpp src/input.cpp)
target_link_libraries(rslidar_node ${rclcpp_LIBRARIES} ${libpcap_LIBRARIES})
ament_target_dependencies(rslidar_node rclcpp std_msgs rslidar_msgs sensor_msgs diagnostic_updater)

# Composition (driver + pointcloud)
add_executable(rslidar_bringup src/rslidar_bringup.cpp src/rsdriver.cpp src/input.cpp)
target_link_libraries(rslidar_bringup ${rclcpp_LIBRARIES} ${libpcap_LIBRARIES})
ament_target_dependencies(rslidar_bringup rclcpp std_msgs rslidar_msgs sensor_msgs diagnostic_updater rslidar_pointcloud)

include_directories(
  include
)

install(TARGETS
  rslidar_node
  rslidar_bringup
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
