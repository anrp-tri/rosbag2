cmake_minimum_required(VERSION 3.5)
project(bag_recorder_nodes)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp_backport REQUIRED)
find_package(rosbag2_storage_backport REQUIRED)
find_package(rosbag2_transport_backport REQUIRED)

add_executable(simple_bag_recorder src/simple_bag_recorder.cpp)
ament_target_dependencies(simple_bag_recorder rclcpp rosbag2_cpp_backport example_interfaces)

install(TARGETS
  simple_bag_recorder
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(data_generator_node src/data_generator_node.cpp)
ament_target_dependencies(data_generator_node rclcpp rosbag2_cpp_backport example_interfaces)

install(TARGETS
  data_generator_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(data_generator_executable src/data_generator_executable.cpp)
ament_target_dependencies(data_generator_executable rclcpp rosbag2_cpp_backport example_interfaces)

install(TARGETS
  data_generator_executable
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(simple_player src/simple_player.cpp)
ament_target_dependencies(simple_player
  rclcpp
  rosbag2_cpp_backport
  rosbag2_storage_backport
  rosbag2_transport_backport)

install(TARGETS
  simple_player
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
