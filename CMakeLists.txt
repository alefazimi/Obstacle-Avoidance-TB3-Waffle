cmake_minimum_required(VERSION 3.8)
project(tb3_obstacle_avoidance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(obstacle_avoidance_node src/obstacle_avoidance_node.cpp)
ament_target_dependencies(obstacle_avoidance_node rclcpp sensor_msgs geometry_msgs)
target_include_directories(obstacle_avoidance_node PUBLIC ${rclcpp_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS} ${geometry_msgs_INCLUDE_DIRS})

install(TARGETS obstacle_avoidance_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()






