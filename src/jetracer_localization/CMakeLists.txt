cmake_minimum_required(VERSION 3.8)
project(jetracer_localization)

find_package(ament_cmake REQUIRED)

# NOTE: odom_pose_to_odometry.py (a ROS 1 robot_pose_ekf compatibility shim)
# is intentionally NOT installed. In ROS 2, robot_localization ekf_node
# publishes nav_msgs/Odometry directly on odometry/filtered (remapped to /odom).
# Installing this script would create a second competing /odom publisher.

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
