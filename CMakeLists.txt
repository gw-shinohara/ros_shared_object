cmake_minimum_required(VERSION 3.13)
project(ros_shared_object CXX)

enable_language(CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set (ROS_DISTRO "rolling")

set (ROS_INCLUDE_DIR "/opt/ros/${ROS_DISTRO}/include")
message(STATUS "ROS_INCLUDE_DIR = ${ROS_INCLUDE_DIR}")
set (ROS_LINK_DIR "/opt/ros/${ROS_DISTRO}/lib")
message(STATUS "ROS_LINK_DIR = ${ROS_LINK_DIR}")

add_library(ros_shared_object SHARED ros_shared_object.cpp)

target_include_directories(ros_shared_object
PRIVATE
    "${ROS_INCLUDE_DIR}"
    "${ROS_INCLUDE_DIR}/rclcpp"
    "${ROS_INCLUDE_DIR}/rcl"
    "${ROS_INCLUDE_DIR}/rcutils"
    "${ROS_INCLUDE_DIR}/rmw"
    "${ROS_INCLUDE_DIR}/rcl_yaml_param_parser"
    "${ROS_INCLUDE_DIR}/rosidl_runtime_c"
    "${ROS_INCLUDE_DIR}/rosidl_typesupport_interface"
    "${ROS_INCLUDE_DIR}/rcpputils"
    "${ROS_INCLUDE_DIR}/builtin_interfaces"
    "${ROS_INCLUDE_DIR}/rosidl_runtime_cpp"
    "${ROS_INCLUDE_DIR}/tracetools"
    "${ROS_INCLUDE_DIR}/rcl_interfaces"
    "${ROS_INCLUDE_DIR}/libstatistics_collector"
    "${ROS_INCLUDE_DIR}/statistics_msgs"
    "${ROS_INCLUDE_DIR}/std_msgs"
    "${ROS_INCLUDE_DIR}/sensor_msgs"
    "${ROS_INCLUDE_DIR}/geometry_msgs"
)

target_link_directories(ros_shared_object
PRIVATE
    ${ROS_LINK_DIR}
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
