cmake_minimum_required(VERSION 3.5)
project(my_nav2_project)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

# Add executable for bt_controller
add_executable(bt_controller src/bt_controller.cpp)
ament_target_dependencies(bt_controller rclcpp std_msgs behaviortree_cpp_v3)

# Install executables
install(TARGETS
  bt_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  src/battery_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch directory to share folder so launch files are found by ros2 launch
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

