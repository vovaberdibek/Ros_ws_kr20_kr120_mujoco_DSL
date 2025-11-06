cmake_minimum_required(VERSION 3.8)
project(kr20_kr120_moveit_demos)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_library(motion_primitives
  src/motion_primitives.cpp
  src/workflow_context.cpp
)
target_include_directories(motion_primitives PUBLIC include)
ament_target_dependencies(motion_primitives
  geometry_msgs
  tf2_geometry_msgs
  custom_interfaces
  moveit_ros_planning_interface
)

add_executable(go_to_pose src/go_to_pose.cpp)
ament_target_dependencies(go_to_pose
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

add_executable(init_parameters_server src/nodes/init_parameters_server.cpp)
target_link_libraries(init_parameters_server motion_primitives)
ament_target_dependencies(init_parameters_server
  rclcpp
  custom_interfaces
  geometry_msgs
  moveit_ros_planning_interface
)

install(TARGETS
  motion_primitives
  go_to_pose
  init_parameters_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
)

ament_export_include_directories(include)

ament_package()
