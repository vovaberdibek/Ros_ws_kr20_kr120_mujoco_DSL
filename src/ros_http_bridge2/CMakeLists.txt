cmake_minimum_required(VERSION 3.8)
project(ros_http_bridge2)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install the Python package so "import ros_http_bridge2" works
ament_python_install_package(${PROJECT_NAME})

# Install entry-point scripts so "ros2 run" works
# (Make sure these files exist in ros_http_bridge2/ and are executable)
install(PROGRAMS
  ${PROJECT_NAME}/ros_http_bridge_rclpy.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ros_http_bridge
)

install(PROGRAMS
  ${PROJECT_NAME}/confirmation_service_rclpy.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME confirm_server
)

# CMakeLists.txt (add this block)
install(PROGRAMS
  ${PROJECT_NAME}/mock_servers.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mock_servers
)


install(PROGRAMS
  ${PROJECT_NAME}/main.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dsl_main
)

ament_package()
