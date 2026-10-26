
cmake_minimum_required(VERSION 3.8)
project(obstacle_avoidance)

set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)


add_executable(perception_node src/perception_node.cpp)
ament_target_dependencies(perception_node rclcpp sensor_msgs std_msgs)


add_executable(decision_node src/decision_node.cpp)
ament_target_dependencies(decision_node rclcpp std_msgs)

add_executable(control_node src/control_node.cpp)
ament_target_dependencies(control_node rclcpp std_msgs geometry_msgs)

install(TARGETS perception_node decision_node control_node
  DESTINATION lib/${PROJECT_NAME})

# Gazebo dunyasi/model klasorleri
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# ROS gerektirmeyen state machine birim testi:  colcon test
if(BUILD_TESTING)
  add_executable(test_state_machine test/test_state_machine.cpp)
  add_test(NAME test_state_machine COMMAND test_state_machine)
endif()

ament_package()