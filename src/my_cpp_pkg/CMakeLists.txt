cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(my_robot_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(test_node src/my_first_node.cpp)
ament_target_dependencies(test_node rclcpp)

add_executable(number_publisher src/number_publisher.cpp)
ament_target_dependencies(number_publisher rclcpp example_interfaces)

add_executable(number_counter src/number_counter.cpp)
ament_target_dependencies(number_counter rclcpp example_interfaces my_robot_interfaces)

add_executable(reset_counter_client src/reset_counter_client.cpp)
ament_target_dependencies(reset_counter_client rclcpp my_robot_interfaces)

add_executable(count_until_server_minimal src/count_until_server_minimal.cpp)
ament_target_dependencies(count_until_server_minimal rclcpp rclcpp_action my_robot_interfaces)

add_executable(count_until_client_minimal src/count_until_client_minimal.cpp)
ament_target_dependencies(count_until_client_minimal rclcpp rclcpp_action my_robot_interfaces)

install(TARGETS
  test_node
  number_publisher
  number_counter
  reset_counter_client
  count_until_server_minimal
  count_until_client_minimal
  DESTINATION lib/${PROJECT_NAME}/
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
