cmake_minimum_required(VERSION 3.5)
project(ros2_lora_topic_bridge)

# Find required ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Include header file
include_directories(include)

# Add executable
add_executable(${PROJECT_NAME} src/ros2_lora_topic_bridge.cpp)

# Set C++ standard
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)

# Link ROS2 libraries
target_link_libraries(${PROJECT_NAME}
  rclcpp
  std_msgs
)

# Install executable
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Install header file
install(FILES include/ros2_lora_topic_bridge.hpp
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install package.xml
ament_package()

