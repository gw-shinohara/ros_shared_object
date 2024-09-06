cmake_minimum_required(VERSION 3.5)
project(ros_shared_object_library)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(ros_shared_object_library SHARED src/ros_shared_object_library.cpp)

ament_target_dependencies(ros_shared_object_library rclcpp std_msgs)

install(TARGETS ros_shared_object_library
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
