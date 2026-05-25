cmake_minimum_required(VERSION 3.8)
project(mm_nav2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(nav2_bt_navigator REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(nav2_planner REQUIRED)
find_package(rclcpp REQUIRED)

install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS src/move_to_obj.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS src/move_to_target.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
