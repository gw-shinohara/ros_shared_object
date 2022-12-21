# Reference https://qiita.com/yumetodo/items/bd8f556ab56298f19ba8

cmake_minimum_required(VERSION 3.13)

enable_language(CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

project(ros_shared_object CXX)

add_library(ros_shared_object SHARED ros_shared_object.cpp)

target_include_directories(ros_shared_object
PRIVATE
    "/opt/ros/rolling/include"
    "/opt/ros/rolling/include/rclcpp"
    "/opt/ros/rolling/include/rcl"
    "/opt/ros/rolling/include/rcutils"
    "/opt/ros/rolling/include/rmw"
    "/opt/ros/rolling/include/rcl_yaml_param_parser"
    "/opt/ros/rolling/include/rosidl_runtime_c"
    "/opt/ros/rolling/include/rosidl_typesupport_interface"
    "/opt/ros/rolling/include/rcpputils"
    "/opt/ros/rolling/include/builtin_interfaces"
    "/opt/ros/rolling/include/rosidl_runtime_cpp"
    "/opt/ros/rolling/include/tracetools"
    "/opt/ros/rolling/include/rcl_interfaces"
    "/opt/ros/rolling/include/libstatistics_collector"
    "/opt/ros/rolling/include/statistics_msgs"
    "/opt/ros/rolling/include/std_msgs"
    "/opt/ros/rolling/include/sensor_msgs"
    "/opt/ros/rolling/include/geometry_msgs"
)

target_link_directories(ros_shared_object
PRIVATE
    /opt/ros/rolling/lib
)

target_link_libraries(ros_shared_object
PRIVATE
    rclcpp
    rcl
    rcl_yaml_param_parser
    rcutils
    rcpputils
    rmw
    rmw_implementation
    tracetools
    ament_index_cpp
    std_msgs__rosidl_typesupport_cpp
    sensor_msgs__rosidl_typesupport_cpp
    rosidl_typesupport_cpp
    rosidl_typesupport_c
    rosidl_runtime_c
    libstatistics_collector
    statistics_msgs__rosidl_typesupport_cpp
    rosgraph_msgs__rosidl_typesupport_cpp
    rcl_interfaces__rosidl_typesupport_cpp
    rcl_interfaces__rosidl_typesupport_c
    rcl_interfaces__rosidl_generator_c
    rcl_logging_interface
    rcl_logging_spdlog
    builtin_interfaces__rosidl_generator_c
)

add_executable(test test.cpp)
