cmake_minimum_required(VERSION 3.5)
project(lidar_mapping_bringup)

# Find required packages
find_package(ament_cmake REQUIRED)

# Install launch files, config files, and rviz files to share directory
install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
