# ROS Shared Object

A talker example of implementation with ROS2 node as shared object

## Requirements

OS: Linux
ROS2: Rolling Ridley (Can be changed to other versions)

## Build
```
mkdir build && build
cmake ..
make
```

## Run

In this example, communication with the [Listener](https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html#talker-listener) of demo_nodes_py in ROS2.

Open another terminal, and in the terminal,
```
source /opt/ros/rolling/setup.bash
ros2 run demo_nodes_py listener
```

In the terminal used for the build process,
```
source /opt/ros/rolling/setup.bash
./test
```

## File descriptions

* ros_shared_object.cpp : Source code for shared object version of ROS2 node. Equivalent to [Talker](https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html#talker-listener) in the cpp_pubsub package.
* test.cpp : Source code for the application to test the shared objects to be created.

## Reference
https://github.com/flynneva/godot_ros
