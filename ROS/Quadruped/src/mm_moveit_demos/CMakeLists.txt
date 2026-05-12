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
find_package(shape_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} SHARED src/pick_place_task.cpp)
target_include_directories(${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  Eigen3 geometry_msgs moveit_task_constructor_core rclcpp moveit_core
)

add_executable(pick_place_demo src/pick_place_demo.cpp)
target_link_libraries(pick_place_demo ${PROJECT_NAME})
ament_target_dependencies(pick_place_demo
  rclcpp moveit_ros_planning_interface moveit_task_constructor_core
  geometry_msgs shape_msgs geometric_shapes tf2_eigen tf2_geometry_msgs moveit_core
)

add_executable(simple_pose_target src/simple_pose_target.cpp)
ament_target_dependencies(simple_pose_target
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_geometry_msgs
)

install(TARGETS ${PROJECT_NAME} pick_place_demo simple_pose_target
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
  Eigen3 geometry_msgs moveit_task_constructor_core rclcpp moveit_core
  moveit_ros_planning_interface
)

ament_package()