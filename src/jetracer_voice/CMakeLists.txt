cmake_minimum_required(VERSION 3.8)
project(jetracer_voice)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/tts_en.py
  scripts/tts_cn.py
  scripts/vad.py
  scripts/ginput.py
  scripts/iat.py
  scripts/aiui.py
  scripts/voice_commander.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
