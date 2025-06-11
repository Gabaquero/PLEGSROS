cmake_minimum_required(VERSION 3.8)
project(robot_control_system)

# Require C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Enable extra warnings on GCC/Clang
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Threads REQUIRED)

# Reception Node
add_executable(reception_node src/reception_node.cpp)
target_link_libraries(reception_node PUBLIC
  ${custom_msgs_TARGETS}
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
  Threads::Threads
)

# PD Controller Node
add_executable(pd_controller_node src/pd_controller_node.cpp)
target_link_libraries(pd_controller_node PUBLIC
  ${custom_msgs_TARGETS}
  ${std_msgs_TARGETS}
  message_filters::message_filters
  rclcpp::rclcpp
  Threads::Threads
)

# Command Sender Node
add_executable(command_sender_node src/command_sender_node.cpp)
target_link_libraries(command_sender_node PUBLIC
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
  Threads::Threads
)

# Trajectory Publisher Node
add_executable(trajectory_publisher_node src/trajectory_publisher_node.cpp)
target_link_libraries(trajectory_publisher_node PUBLIC
  ${custom_msgs_TARGETS}
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  Threads::Threads
)

# Safety Monitor Node
add_executable(safety_monitor_node src/safety_monitor_node.cpp)
target_link_libraries(safety_monitor_node PUBLIC
  ${custom_msgs_TARGETS}
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
  Threads::Threads
)

# Installation rules
install(TARGETS
  reception_node
  pd_controller_node
  command_sender_node
  trajectory_publisher_node
  safety_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY trajectory/ DESTINATION share/${PROJECT_NAME}/trajectory)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz)
install(PROGRAMS scripts/setup_can.sh DESTINATION lib/${PROJECT_NAME})

ament_package()