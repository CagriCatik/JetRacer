cmake_minimum_required(VERSION 3.8)
project(jetracer_perception)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  src/color_tracking.py
  src/line_follow.py
  src/object_tracking.py
  src/motion_detect.py
  src/face_detect.py
  src/yolo_detection.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  models
  src
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
