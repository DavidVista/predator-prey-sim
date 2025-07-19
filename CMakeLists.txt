cmake_minimum_required(VERSION 3.8)
project(predator_prey_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(turtlesim REQUIRED)

# Add executables
add_executable(turtle_manager src/turtle_manager.cpp)
ament_target_dependencies(turtle_manager rclcpp geometry_msgs std_msgs turtlesim)

add_executable(turtle_controller src/turtle_controller.cpp)
ament_target_dependencies(turtle_controller rclcpp geometry_msgs std_msgs turtlesim)

add_executable(boids_prey_controller src/boids_prey_controller.cpp)
ament_target_dependencies(boids_prey_controller rclcpp geometry_msgs std_msgs turtlesim)

add_executable(predator_controller src/predator_controller.cpp)
ament_target_dependencies(predator_controller rclcpp geometry_msgs std_msgs turtlesim)

add_executable(energy_predator_controller src/energy_predator_controller.cpp)
ament_target_dependencies(energy_predator_controller rclcpp geometry_msgs std_msgs turtlesim)

add_executable(energy_efficient_predator_controller src/energy_efficient_predator_controller.cpp)
ament_target_dependencies(energy_efficient_predator_controller rclcpp geometry_msgs std_msgs turtlesim)

add_executable(multi_prey_boids_controller src/multi_prey_boids_controller.cpp)
ament_target_dependencies(multi_prey_boids_controller rclcpp geometry_msgs std_msgs turtlesim)

# Install executables
install(TARGETS
  turtle_manager
  turtle_controller
  boids_prey_controller
  predator_controller
  energy_predator_controller
  energy_efficient_predator_controller
  multi_prey_boids_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
