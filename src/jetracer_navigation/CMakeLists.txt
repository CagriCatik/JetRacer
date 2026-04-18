cmake_minimum_required(VERSION 3.8)
project(jetracer_navigation)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/cmd_vel_to_steering.py
  scripts/multipoint_nav.py
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
