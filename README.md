# diy_robotarm_wer24_driver

This ROS2 package contains the driver for our DIY-Robot.
This driver will be the interface between ROS (ruinning in docker containers on your LINUX machine) and the Hardware (Task running on an ESP32 which controlls the robot axis by DRV8825 stepper drivers).
The connection will be established via network (so the robot/esp has an IP Adress which ROS is talking to via TCP/IP Protocol).

This package is a part of fully modular designed ROS2 integration for our diy-robot.
Because of the (not so well choosen) urdf-structure -ros2 control must be defined inside the urdf model- this driver package needs the whole description package (diy_robotarm_full_cell_description) already running in an container.
Thats why we use the image of the running docker container as base image for our docker container which will run the driver package.

Please refer to the main- repo for further informations about the whole project.
