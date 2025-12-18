cmake_minimum_required(VERSION 3.22)
project(fairino_dual_moveit_config)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/broadcaster_robot1.py
  scripts/broadcaster_robot2.py
  scripts/arm_planner.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

install(DIRECTORY launch worlds DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

