cmake_minimum_required(VERSION 3.22)
project(fairino_dual_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)

# 导出依赖
ament_export_dependencies(
  rclcpp
  geometry_msgs
  control_msgs
)

add_executable(servo_keyboard_input src/servo_keyboard_input.cpp)
target_include_directories(servo_keyboard_input PUBLIC include)
# 添加依赖的头文件路径
ament_target_dependencies(servo_keyboard_input
  "rclcpp"
  "geometry_msgs"
  "control_msgs"
)
target_link_libraries(servo_keyboard_input
  ${rclcpp_LIBRARIES}
)



install(PROGRAMS
  scripts/broadcaster_robot1.py
  scripts/broadcaster_robot2.py
  scripts/arm_planner.py
  scripts/vr_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS servo_keyboard_input
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

install(DIRECTORY launch worlds DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

