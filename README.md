# Abstract

#### These ROS packages are for the basic functions of JAKA robot, At Present MoveJ，MoveP and some moveit function are supported

# Package Description

* ***jaka_control*** is the package of receiving the moveit planning result, next，interpolating these points using cubic spline curve and send the command to the robot

* ***jaka_controller_tcp_ros*** is the package to control the robot and receive the state information using TCP/IP protocol

* ***jaka_description*** is the package of storing the jaka urdf files

* ***jaka_moveit_config*** is the package of using moveit to realize the motion planning

* ***universal_msgs*** is the package of store types files of topic、service and action

# Usage

1. open a terminal and git clone this reponsitory to your workspace/src

2. open a termianl and run 

```bash 
catkin_make
```

3. Change `address` in `jaka_ros_node.cpp` and `jaka_state_pub_node` or set `robot_ip` in `jaka_bringup.launch`

4. open a terminal and run

```bash
roslaunch jaka_controller_tcp_ros jaka_bringup.launch
```

5. if you want to use moveit to perform some task, run

```bash
roslaunch jaka_moveit_config jaka_moveit_planning_execution.launch
```

# Development Environment

* ubuntu16.04+ros_kinetic

* ubuntu18.04+ros_melodic
