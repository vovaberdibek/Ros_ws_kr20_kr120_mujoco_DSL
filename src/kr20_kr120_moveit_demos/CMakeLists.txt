cmake_minimum_required(VERSION 3.8)
project(kr20_kr120_moveit_demos)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(go_to_pose src/go_to_pose.cpp)
ament_target_dependencies(go_to_pose
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

# Install the C++ node
install(TARGETS go_to_pose
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (so `ros2 launch` can find them)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
