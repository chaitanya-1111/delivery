cmake_minimum_required(VERSION 3.8)
project(robot_description_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY
    urdf
    meshes
    launch
    config
    gazebo
  DESTINATION
    share/${PROJECT_NAME}/
)

# Install Python scripts (mesh generator)
install(
  PROGRAMS
    scripts/generate_meshes.py
  DESTINATION
    lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()