cmake_minimum_required(VERSION 3.5)
project(ros_shared_object_library)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_library(ros_shared_object_library SHARED src/ros_shared_object.cpp)

ament_target_dependencies(ros_shared_object_library rclcpp std_msgs sensor_msgs OpenCV cv_bridge)

target_include_directories(ros_shared_object_library PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(ros_shared_object_library
  ${OpenCV_LIBS}
)

install(TARGETS ros_shared_object_library
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(ros2_share_test test.cpp)

# Link OpenCV and libdl to the test executable
target_include_directories(ros2_share_test PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(ros2_share_test dl ${OpenCV_LIBS})

ament_package()
