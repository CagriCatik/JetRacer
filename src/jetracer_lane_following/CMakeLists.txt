cmake_minimum_required(VERSION 3.8)
project(jetracer_lane_following)

find_package(ament_cmake REQUIRED)

# ── Install library modules and the node script ────────────────────────────
# All Python files land in lib/${PROJECT_NAME}/ so the node can import the
# library modules via the sys.path.insert(__file__) pattern in the node.
install(PROGRAMS
  lib/lane_detection.py
  lib/waypoint_prediction.py
  lib/lateral_control.py
  lib/mpc_control.py
  lib/longitudinal_control.py
  scripts/lane_following_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
