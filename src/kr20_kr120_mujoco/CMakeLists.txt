cmake_minimum_required(VERSION 3.8)
project(kr20_kr120_mujoco)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY mjcf
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch mjcf urdf
  DESTINATION share/${PROJECT_NAME}
)



ament_package()
