cmake_minimum_required(VERSION 3.5)
project(custom_interfaces)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)  # Ensure geometry_msgs is here

# Generate the service interfaces with dependencies
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteAction.action"
  "srv/InitParameters.srv"
  "srv/PlaceTray.srv" 
  "srv/ExternalScrewingSequence.srv"
  "srv/InternalScrewByNum.srv"
  "srv/InternalScrewingSequence.srv"
  "srv/RechargeSequence.srv"
  "srv/OperatorPositionTray.srv" 
  "srv/PositionTray.srv"
  "srv/PickTray.srv"
  "srv/AddTray2.srv"
  "srv/AddTray.srv"
  "srv/ConfirmWorkflow.srv" 
  "srv/TriggerAction.srv"
  "srv/BringTray.srv"
  DEPENDENCIES geometry_msgs diagnostic_msgs  # Add DEPENDENCIES here
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
