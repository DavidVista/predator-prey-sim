cmake_minimum_required(VERSION 3.8)
project(turtle_hunt)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(turtlesim REQUIRED)  # For turtle control

# Add executables
add_executable(spawner_node src/spawner_node.cpp)
add_executable(prey_node src/prey_node.cpp)
add_executable(predator_node src/predator_node.cpp)

# Link dependencies
ament_target_dependencies(spawner_node
  rclcpp
  std_srvs
  ros_gz_sim
)

ament_target_dependencies(prey_node
  rclcpp
  geometry_msgs
  message_filters  # For time sync
)

ament_target_dependencies(predator_node
  rclcpp
  geometry_msgs
)

# Install targets
install(TARGETS
  spawner_node
  prey_node
  predator_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
