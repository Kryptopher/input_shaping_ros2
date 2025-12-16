cmake_minimum_required(VERSION 3.8)
project(input_shaping_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Executable
add_executable(input_shaping_client_node
  src/input_shaping_client.cpp
)

ament_target_dependencies(input_shaping_client_node
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
)

install(TARGETS
  input_shaping_client_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
