cmake_minimum_required(VERSION 3.8)
project(pick_and_place_core)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(pick_and_place_node src/pick_and_place_node.cpp)

# link dependencies 
ament_target_dependencies(pick_and_place_node
  rclcpp
  moveit_task_constructor_core
)

install(TARGETS pick_and_place_node
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
