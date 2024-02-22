# diy_robotarm_wer24_driver

## thematical Classification

This ROS2 package contains the driver for our DIY-Robot.
This driver will be the interface between ROS (ruinning in docker containers on your LINUX machine) and the hardware (task running on an ESP32 which controlls the robot axis by DRV8825 stepper drivers).
The connection will be established via network (so the robot/esp has an IP Adress which ROS is talking to via TCP/IP Protocol).

We use Docker for development (dev branch) and deployment (main branch) to avoid version and dependencies issues.
While building the container the depencencie-reops for the robotarm and gripper are cloned from GitHub and setup automated.
To run the package, you just have to source the run.sh script file. The container will start and you can work with the provided package, modify it or include it as an dependencie to another packages!

The main idea is, that this repo can be cloned inside a docker-container containing and combining all packages for operationg the Robot (e.g. description, drivers, moveit2, applivation) Using differnet docker containers is very likely, because this makes the whole integration very modular.

This driver package needs the whole description package (diy_robot_full_cell_description) already running in an container, so we need **Part 1, description** as a dependencie package already build and sourced inside this container (````/home/$USER/dependencies/...````)

Refer to the main Readme.md https://github.com/mathias31415/diy_robotics/blob/main/ROS-Packages/ROS-OVERVIEW.md for a general overview.


## Package Structure

![arm_driver_file_tree](images/arm_driver_files_tree.png)

 - images and README.md are only for docomentation purposes
 - Dockerfile, run.sh and dds_profile.xml are used to create the docker container where ROS is running in
 - CMakeLists.txt and package.xml are defining this build process (wich dependencies are needed, which file should be installed where in the created directories, ...)
 - config, include, src and launch are the directories which are containing the source files for this package, they will be described in the following

## Short introduction into ROS2 Control

ROS 2 Control is a framework for controlling robotic systems in the Robot Operating System 2 (ROS 2) ecosystem. It provides a modular and flexible architecture for interfacing with various hardware components, such as motors and sensors. This framework enables developers to design and implement control algorithms for a wide range of robotic platforms. ROS 2 Control offers standardized interfaces for hardware abstraction and device drivers, facilitating interoperability across different robotic systems. It supports both position and velocity control, as well as efforts to accommodate diverse robotic applications and control strategies.

In our case we will implement a hardware interface to our diy-hardware which will be position-controlled. Moureover we will define two controllers. On the one hand a ````forward_command_controller/ForwardCommandController```` which is used only for testing purposes and on the other hand a ```joint_trajectory_controller/JointTrajectoryController```` which will be used by moveit2.

The following graphics gives you a short overview about the ROS2-control architecture (source: https://control.ros.org/humble/doc/getting_started/getting_started.html):

![ros_control](images/ros_control.png)

The hardware innterface plugin is called in ***the ros2control urdf in** with ```` <plugin>esp32_robot_driver/ESP32Hardware</plugin>```  

## Definition of the Controller Manager


## Launch

By running the launch file ````visualization.launch.py```` automatically when you start the docker container by sourcing the run script ````./run.sh```` you will launch Rviz and the Joint State Publisher GUI. This is only for visualization and checking purposes, because we don't do a real bringup of the robot model. Joint States are just published by the GUI on the specific ROS-topic.


