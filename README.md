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

## Short Introduction into ROS2 Control

ROS 2 Control is a framework for controlling robotic systems in the Robot Operating System 2 (ROS 2) ecosystem. It provides a modular and flexible architecture for interfacing with various hardware components, such as motors and sensors. This framework enables developers to design and implement control algorithms for a wide range of robotic platforms. ROS 2 Control offers standardized interfaces for hardware abstraction and device drivers, facilitating interoperability across different robotic systems. It supports both position and velocity control, as well as efforts to accommodate diverse robotic applications and control strategies.

In our case we will implement a hardware interface to our diy-hardware which will be position-controlled. Moureover we will define two controllers. On the one hand a ````forward_command_controller/ForwardCommandController```` which is used only for testing purposes and on the other hand a ````joint_trajectory_controller/JointTrajectoryController```` which will be used by moveit2.

The following graphics gives you a short overview about the ROS2-control architecture (source: https://control.ros.org/humble/doc/getting_started/getting_started.html):

![ros_control](images/ros_control.png)

The hardware innterface plugin is called in the dependencie package "diy_robotarm_wer24_description" ````./urdf/diy_robotarm.ros2_control.urdf.xacro```` with ```` <plugin>esp32_robot_driver/ESP32Hardware</plugin>````. For more informations about generating a ROS2 control tag for real hatdware or for fake hardware please refer to this repo (we mentioned this bit weired ros2 architecture there as well): https://github.com/RobinWolf/diy_robotarm_wer24_description

## Definition of the Controller Manager

The controller manager and the controller interfaces itself are defined in ````./config/esp32_controller.yaml````. 




## Definition of the Hardware Interface Plugin



## Launch

We have implemented three launch files for three different purposses: 

- ````viszualize.launch.py````: This is only for visualization and checking purposes of the description packages in the dependencies-directory, because we don't do a real bringup of the robot model. Joint States are just published by the GUI on the specific ROS-topic, we don't launch our drivers!
- ````forward_controller.launch.py````: This is only for testing purposes of our hardware interface. This launch file will launch the **forward_command_controller** only. By passing the launch argument ````use_fake_hardware:=false````in launch, the driver trys to connect to the real robot hardware. Now we have done a real bringup of the robot and you should be able to control the robot by publishing position commands on this topic: ````ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"````. In the data array you can pass any float value between +/- pi, this equals the absolute target joint position in radiants.
- ````trajectory_controller.launch.py````: This launch file gets called in the final application and launches the **trajectory_controller** only. Moveit uses this controller to send the calculated trajectories to the robot. A trajectory consists of waypoint positions and timestamps. Here we do a real bringup of the robot too.

**Note:** <br>
Every joint interface defined in the ````/home/$USER/dependencies/diy_robotarm_wer24_description/urdf/diy_robotarm.ros2_control.urdf.xacro```` can only be linked to one single controller interface defined in the ````./config/esp32_controller.yaml````. So make sure you don't launch a controller twice or launch the forward_command_controller and the trajectory_controller at the same time!



