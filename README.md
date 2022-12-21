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

### Run

Open another terminal, and in the terminal,
```
source /opt/ros/rolling/setup.bash
ros2 run cpp_pubsub listener
```

In the terminal used for the build process,
```
source /opt/ros/rolling/setup.bash
./test
```

## Reference
https://github.com/flynneva/godot_ros
