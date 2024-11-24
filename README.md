# My Gazebo Tutorial

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)


## Overview

This repository houses a ROS2 package named **walker**, which implements a basic walker algorithm for a robot operating in the Gazebo simulation environment. The robot imitates a Roomba vacuum cleaner's behavior by moving forward until it encounters an obstacle. It then rotates in place until the way is clear, alternating between clockwise and counterclockwise rotations. The project uses a state machine design pattern, providing modularity and scalability.

## Features

- **State Machine Design**: Employs a robust state design pattern to facilitate seamless state transitions.
- **Obstacle Detection**: Utilizes LIDAR data to identify obstacles in the forward path.
- **Dynamic Rotation Direction**:Switches between clockwise and counterclockwise rotations to avoid obstacles.
- **Rosbag Integration**: Captures all ROS2 topics, excluding RGB-D camera topics, to reduce file size.

## Author

Keyur Borad (keyurborad5@gmail.com)

## Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- C++17 compatible compiler
- colcon build tool
- clang-format
- cpp-lint
- clang-tidy

## Installation
1. Create a colon workspace
```bash
# After installing the ROS2 Humble Open terminal and source the ROS2 underlay
source /opt/ros/humble/setup.bash
# Create new directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

```
2. Clone the repository 
```bash
#Clone the repository
git clone https://github.com/keyurborad5/my_gazebo_tutorials.git
```

3. Check for missing dependencies and installing it
```bash
#go to ros2_ws directory
cd ..
rosdep install -i --from-path src --rosdistro humble -y

```
4. Build the workspace with colcon bubild
```bash
colcon build
```

5. Source Overlay
```bash
source install/setup.bash
```

The repository structure should be as follows 

```plaintext
ros2_ws/
├── src/
│   ├── my_gazebo_tutorials
        ├── bag_files
        │   ├── rosbag2_2024_11_24-14_57_21
        │   │   ├── metadata.yaml
        │   │   └── rosbag2_2024_11_24-14_57_21_0.db3
        │   ├── rosbag2_2024_11_24-15_11_49
        │   │   ├── metadata.yaml
        │   │   └── rosbag2_2024_11_24-15_11_49_0.db3
        │   └── rosbag2_2024_11_24-15_13_23
        │       ├── metadata.yaml
        │       └── rosbag2_2024_11_24-15_13_23_0.db3
        ├── CMakeLists.txt
        ├── include
        │   └── walker
        │       └── walker_node.hpp
        ├── launch
        │   ├── my_bot.launch.py
        │   ├── robot_state_publisher.launch.py
        │   └── spawn_turtlebot3.launch.py
        ├── models
        │   ├── turtlebot3_house
        │   │   ├── model.config
        │   │   └── model.sdf
        │   ├── turtlebot3_waffle
        │   │   ├── model-1_4.sdf
        │   │   ├── model2.sdf
        │   │   ├── model.config
        │   │   └── model.sdf
        │   └── turtlebot3_world
        │       ├── meshes
        │       │   ├── hexagon.dae
        │       │   └── wall.dae
        │       ├── model-1_4.sdf
        │       ├── model.config
        │       └── model.sdf
        ├── package.xml
        ├── README.md
        ├── rviz
        │   └── tb3_gazebo.rviz
        ├── src
        │   └── walker_node.cpp
        ├── urdf
        │   ├── common_properties.urdf
        │   └── turtlebot3_waffle.urdf
        └── worlds
            ├── turtlebot3_house.world
            └── turtlebot3_world.world

```



## Running the pacakage Launch file
```bash
echo "export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models/" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
source ~/.bashrc
cd ~/ros2_ws
ros2 launch walker my_bot.launch.py
```

##  Record and play a bag for 15 sec
```bash
# run the launch file with ros bag enabled
ros2 launch walker my_bot.launch.py enable_bag_record:=true
# A ros bag will be generated in the results/recorded_bag directory
cd ~/ros_ws/src/my_gazebo_tutorials/bag_files/
ros2 bag play <name of the bag folder w/o brackets>
# to verify the recorded message (execute in another terminal and source underlay as well as overlay)
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

```

## Clang-Formating
```bash
cd ~/ros2_ws
#Clang-format 
clang-format -i --style=Google $(find . -name *.cpp -o -name *.hpp | grep -v "/build/")

```
## Cpp-Lint
```bash
# go to your ros2 workspace directory
cd ~/ros2_ws/src
#Cpp Lint
cpplint --filter=-legal/copyright,-build/c++11,+build/c++17,-build/namespaces,-build/include_order $(find . -name *.cpp | grep -v "/build/")

```
## Clang-tidy
```bash
cd ~/ros2_ws
# Build the workspace again with the camake args to generate compile_commands.jason file for Clang-tidy to work
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
#Clang-tidy command
clang-tidy -p build/my_gazebo_tutorials --extra-arg=-std=c++17 src/my_gazebo_tutorials/src/*.cpp
```

## Acknowledgement

- Open Source Robotics Foundation, Inc.
- ROS2 Community

