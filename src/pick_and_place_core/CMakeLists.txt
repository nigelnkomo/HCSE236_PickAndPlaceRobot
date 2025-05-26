cmake_minimum_required(VERSION 3.8)
project(pick_and_place_core)

# Essential packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_common REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Optional testing support
find_package(ament_cmake_gtest REQUIRED)

# Main executable
add_executable(pick_and_place_node src/pick_and_place_node.cpp)

ament_target_dependencies(pick_and_place_node
  rclcpp
  sensor_msgs
  moveit_task_constructor_core
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
  tf2_ros
  tf2_geometry_msgs
  Eigen3
)

# Installation
install(TARGETS pick_and_place_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Enable testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_node test/test_node.cpp)
  target_link_libraries(test_node ${rclcpp_LIBRARIES})
  ament_target_dependencies(test_node
    rclcpp
    pick_and_place_core
  )
endif()


# Export dependencies
ament_export_dependencies(
  rclcpp
  sensor_msgs
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_task_constructor_core
  tf2_ros
  tf2_geometry_msgs
  Eigen3
)

enable_testing()
ament_package()

