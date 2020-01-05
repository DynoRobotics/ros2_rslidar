cmake_minimum_required(VERSION 3.5)
project(ros2_rslidar_driver)

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
find_package(diagnostic_updater REQUIRED)


add_executable(rslidar_node src/rslidar_node.cpp src/rsdriver.cpp src/input.cc)
target_link_libraries(rslidar_node ${rclcpp_LIBRARIES} ${libpcap_LIBRARIES})
ament_target_dependencies(rslidar_node rclcpp std_msgs rslidar_msgs sensor_msgs diagnostic_updater)

install(TARGETS
  rslidar_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
