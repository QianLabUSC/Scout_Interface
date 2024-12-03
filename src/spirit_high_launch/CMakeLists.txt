cmake_minimum_required(VERSION 3.8)
project(spirit_high_launch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)  # if Python-based launch files require rclpy


# Testing dependencies
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install the launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
