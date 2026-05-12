cmake_minimum_required(VERSION 3.22)
project(mm_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
  install(PROGRAMS
    scripts/repub_odometry_mdof_joint_states.py
    scripts/base_cmd_vel_bridge.py
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
