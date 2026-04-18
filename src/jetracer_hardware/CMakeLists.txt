cmake_minimum_required(VERSION 3.8)
project(jetracer_hardware)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(jetracer_serial_node src/jetracer_serial_node.cpp)
target_compile_features(jetracer_serial_node PUBLIC cxx_std_17)
target_include_directories(jetracer_serial_node PRIVATE include)
ament_target_dependencies(
  jetracer_serial_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
target_link_libraries(jetracer_serial_node Boost::system)

install(TARGETS jetracer_serial_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/calibrate_linear.py
  scripts/laser_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
