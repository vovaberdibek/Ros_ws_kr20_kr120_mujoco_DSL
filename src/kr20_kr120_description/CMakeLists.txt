cmake_minimum_required(VERSION 3.8)
project(kr20_kr120_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# These are optional; keep them only if you actually use them at build/launch time
# find_package(xacro REQUIRED)
# find_package(urdf REQUIRED)
# find_package(robot_state_publisher REQUIRED)

# Install description resources
foreach(dir launch config urdf collision materials visual)
  install(DIRECTORY ${dir}/ DESTINATION share/${PROJECT_NAME}/${dir})
endforeach()

# Install ALL MuJoCo stuff recursively (XML + meshes + .obj/.stl/.dae/.mtl)
install(DIRECTORY mujoco/ DESTINATION share/${PROJECT_NAME}/mujoco)


ament_package()
