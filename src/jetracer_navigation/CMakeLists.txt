cmake_minimum_required(VERSION 3.8)
project(jetracer_navigation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

install(PROGRAMS
  scripts/cmd_vel_to_steering.py
  scripts/multipoint_nav.py
  scripts/rrt_star_planner.py
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(cmd_vel_to_steering_node src/cmd_vel_to_steering_node.cpp)
target_compile_features(cmd_vel_to_steering_node PUBLIC cxx_std_17)
ament_target_dependencies(
  cmd_vel_to_steering_node
  rclcpp
  geometry_msgs
)

install(TARGETS cmd_vel_to_steering_node
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
