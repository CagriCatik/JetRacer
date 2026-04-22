cmake_minimum_required(VERSION 3.8)
project(jetracer_bringup)

find_package(ament_cmake REQUIRED)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/calibrate_linear.py
  scripts/controller_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

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
