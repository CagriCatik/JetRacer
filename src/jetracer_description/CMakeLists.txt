cmake_minimum_required(VERSION 3.8)
project(jetracer_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  meshes
  rviz
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts so they are accessible as ROS 2 executables.
# joint_state_publisher.py is the cmd_vel-driven joint state animator
# ported from the cytron_jetracer ROS 1 tutorial package.
install(PROGRAMS
  scripts/joint_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()