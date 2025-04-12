cmake_minimum_required(VERSION 3.5)
project(stand_go2)

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

# Include directories
include_directories(include)

# Set dependencies list
set(DEPENDENCY_LIST
  unitree_go
  rclcpp
  rclcpp_action
  std_msgs
  action_msgs
)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate the action interface
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/StandAction.action"
  DEPENDENCIES builtin_interfaces
)

# Add the executable
add_executable(stand_go2_node src/stand_go2.cpp src/motor_crc.cpp)

# Link dependencies to the executable
ament_target_dependencies(stand_go2_node ${DEPENDENCY_LIST})

# Get typesupport target
rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Link the generated typesupport libraries
target_link_libraries(stand_go2_node ${typesupport_target})

# Install the executable
install(TARGETS 
        stand_go2_node
        DESTINATION lib/${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Package export
ament_package()