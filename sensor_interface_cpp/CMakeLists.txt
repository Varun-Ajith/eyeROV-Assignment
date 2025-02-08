cmake_minimum_required(VERSION 3.8)
project(sensor_interface_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add the executable
add_executable(sensor_node src/sensor_node.cpp)

# Link the executable to ROS2 libraries
ament_target_dependencies(sensor_node rclcpp std_msgs)

# Install the executable
install(TARGETS
  sensor_node
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_package()
