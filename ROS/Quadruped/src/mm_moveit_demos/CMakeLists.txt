cmake_minimum_required(VERSION 3.8)
project(mm_moveit_demos)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter -Wno-unused-function)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) 
find_package(shape_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(graph_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

add_library(${PROJECT_NAME} SHARED src/pick_place_task.cpp)
target_include_directories(${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  Eigen3 geometry_msgs moveit_task_constructor_core rclcpp moveit_core sensor_msgs
)

add_executable(pick_place_demo src/pick_place_demo.cpp)
target_link_libraries(pick_place_demo ${PROJECT_NAME})
ament_target_dependencies(pick_place_demo
  rclcpp moveit_ros_planning_interface moveit_task_constructor_core
  geometry_msgs shape_msgs geometric_shapes tf2_eigen tf2_geometry_msgs moveit_core sensor_msgs
)

add_executable(move_arm_pose src/move_arm_pose.cpp)
ament_target_dependencies(move_arm_pose
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  sensor_msgs 
  graph_msgs
  tf2_geometry_msgs
  tf2
  moveit_visual_tools         
  rviz_visual_tools 
)

add_executable(move_group_state src/move_group_state.cpp)
ament_target_dependencies(move_group_state
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  sensor_msgs 
  graph_msgs
  tf2_geometry_msgs
  tf2
  moveit_visual_tools         
  rviz_visual_tools 
)

add_executable(direct_pose_target src/direct_pose_target.cpp)
ament_target_dependencies(direct_pose_target
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  geometry_msgs
  tf2_geometry_msgs
  tf2
  tf2_ros
)


install(PROGRAMS
  src/pick_place.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME} direct_pose_target pick_place_demo move_arm_pose move_group_state
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  eigen3_cmake_module Eigen3 geometry_msgs moveit_task_constructor_core rclcpp moveit_core
  moveit_ros_planning_interface sensor_msgs moveit_ros_planning_interface tf2_ros tf2_geometry_msgs
  control_msgs rclcpp_action
)

ament_package()