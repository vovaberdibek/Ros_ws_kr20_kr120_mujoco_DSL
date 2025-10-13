cmake_minimum_required(VERSION 3.8)
project(kr20_kr120)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)

# ===== Executables that REALLY exist =====
add_executable(integration_node src/integration.cpp)
ament_target_dependencies(integration_node
  rclcpp geometry_msgs std_msgs sensor_msgs tf2_geometry_msgs
  moveit_ros_planning_interface gazebo_msgs custom_interfaces)

# Uncomment only if these source files exist
# add_executable(kr120_pick_place src/kr120_pick_place.cpp)
# ament_target_dependencies(kr120_pick_place
#   rclcpp geometry_msgs std_msgs sensor_msgs tf2_geometry_msgs
#   moveit_ros_planning_interface gazebo_msgs custom_interfaces)

# add_executable(kr120_pick_placeTest src/kr120_pick_placeTest.cpp)
# ament_target_dependencies(kr120_pick_placeTest
#   rclcpp geometry_msgs std_msgs sensor_msgs tf2_geometry_msgs
#   moveit_ros_planning_interface gazebo_msgs custom_interfaces)

# add_executable(close_gripper_node src/simple_moveit_example.cpp)
# ament_target_dependencies(close_gripper_node
#   rclcpp geometry_msgs std_msgs sensor_msgs tf2_geometry_msgs
#   moveit_ros_planning_interface gazebo_msgs custom_interfaces)

# add_executable(custom_interface_server
#   src/custom_interface_server.cpp src/custom_interface_functions.cpp)
# ament_target_dependencies(custom_interface_server
#   rclcpp geometry_msgs std_msgs sensor_msgs tf2_geometry_msgs
#   moveit_ros_planning_interface gazebo_msgs custom_interfaces)

# add_executable(model_spawner_node src/model_spawner.cpp)
# ament_target_dependencies(model_spawner_node
#   rclcpp geometry_msgs std_msgs sensor_msgs tf2_geometry_msgs
#   moveit_ros_planning_interface gazebo_msgs custom_interfaces)

install(TARGETS
  integration_node
  # kr120_pick_place
  # kr120_pick_placeTest
  # close_gripper_node
  # custom_interface_server
  # model_spawner_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
