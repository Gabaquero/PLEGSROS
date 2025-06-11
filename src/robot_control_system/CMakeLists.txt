cmake_minimum_required(VERSION 3.8)
project(robot_control_system)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Reception Node
add_executable(reception_node src/reception_node.cpp)
ament_target_dependencies(reception_node
  rclcpp
  std_msgs
  custom_msgs
)
target_link_libraries(reception_node pthread)

# PD Controller Node
add_executable(pd_controller_node src/pd_controller_node.cpp)
ament_target_dependencies(pd_controller_node
  rclcpp
  std_msgs
  custom_msgs
  message_filters
)

# Command Sender Node
add_executable(command_sender_node src/command_sender_node.cpp)
ament_target_dependencies(command_sender_node
  rclcpp
  std_msgs
)

# Trajectory Publisher Node
add_executable(trajectory_publisher src/trajectory_publisher_node.cpp)
ament_target_dependencies(trajectory_publisher
  rclcpp
  custom_msgs
  ament_index_cpp
)

# Safety Monitor Node
add_executable(safety_monitor src/safety_monitor_node.cpp)
ament_target_dependencies(safety_monitor
  rclcpp
  std_msgs
  custom_msgs
)

# Install executables
install(TARGETS
  reception_node
  pd_controller_node
  command_sender_node
  trajectory_publisher
  safety_monitor
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install trajectory files
install(DIRECTORY trajectory/
  DESTINATION share/${PROJECT_NAME}/trajectory
)

# Install rviz configs
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# Install scripts
install(PROGRAMS
  scripts/setup_can.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()