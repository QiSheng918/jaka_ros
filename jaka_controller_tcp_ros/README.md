# This ROS package is for the basic motor functions of JAKA robot. 
![alt text](images/JAKA_Zu_7.jpg "JAKA Zu 7 Robot")

# Abstract
- Project name: **jaka_controller_tcp_ros**
- This repo is a cmake project for the basic motor functions of JAKA robot.
- Communication protocol is: `TCP/IP`.
- APP/Robot-Firmware/servo-Firmware version: `1.4.11.17/1.4.11_21RC/R-1.10_2.126`

## Dependency

1. jsoncpp:

``` bash
sudo apt-get install libjsoncpp-dev
```

This should install json related header files in `/usr/include/jsoncpp` as we use in the `CMakeLists.txt`.

2. universal_msgs:

```bash
git submodule init
git submodule update
```

And move `universal_msgs` package from `jaka_controller_tcp_ros/dependency/` to `${base_path_to_your_catkin_workspace}/src/`.


3. ros-kinetic-universal-robot package:

```bash
sudo apt install ros-kinetic-universal-robot
```

## Package description

1. `jaka_ros_node` is the node of receiving control command and send it to the robot using TCP/IP protocol

2. `jaka_state_pub_node` is the node of receiving the state information that robot sent and publish the information using ros topic

3. `jaka_shutdown_node` is the node to shutdown the connection between robot and computer, when your connection is wrong

## Build and Test

Change `address` in `jaka_ros_node.cpp` and `jaka_state_pub_node` or set `robot_ip` in `jaka_bringup.launch`

build:

```bash
cd ${base_path_to_your_catkin_workspace}
catkin_make
```

run:

open one terminal and run:

```bash
roslaunch jaka_controller_tcp_ros jaka_bringup.launch
```
