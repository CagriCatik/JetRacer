cmake_minimum_required(VERSION 3.8)
project(jetracer_teleop)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/teleop_key.py
  scripts/teleop_joy.py
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
