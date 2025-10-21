cmake_minimum_required(VERSION 3.8)
project(pick_place_bt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# add include directory for headers
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(pick_place_bt_node src/set_bool_node.cpp src/pick_place_bt.cpp)
ament_target_dependencies(pick_place_bt_node rclcpp std_msgs std_srvs behaviortree_ros2 btcpp_ros2_interfaces)
install(TARGETS
  pick_place_bt_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(pick_place_bt_server_node src/pick_place_bt_server.cpp)
ament_target_dependencies(pick_place_bt_server_node rclcpp std_msgs std_srvs)
install(TARGETS
  pick_place_bt_server_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY behavior_trees launch
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
