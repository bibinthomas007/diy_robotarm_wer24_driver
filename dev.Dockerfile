##############################################################################
##        Stage 1: Driver Image from full_cell_description Image            ##
##############################################################################
ARG ROS_DISTRO=humble
# set description image (of running diy_robotarm_full_cell_description container) to there specified image name:
# --> diy-full-description/ros-render:"$ROS_DISTRO"
# we dont need to specify the whole docker stages from there, but sourcing inside this container from the image is necesary!
FROM diy-full-description-dev/ros-render:"$ROS_DISTRO" as diy-robotarm-driver


# Add built diy-full cell description package to entrypoint by calling install/setup.bash
USER root
RUN apt-get update && apt-get install -y ros-humble-controller-interface 
RUN apt-get update && apt-get install -y ros-humble-controller-manager 
RUN apt-get update && apt-get install -y ros-humble-hardware-interface 
RUN apt-get update && apt-get install -y ros-humble-pluginlib 
RUN apt-get update && apt-get install -y ros-humble-rclcpp
RUN apt-get update && apt-get install -y ros-humble-rclcpp-lifecycle
RUN apt-get update && apt-get install -y ros-humble-ros2-control
RUN apt-get update && apt-get install -y ros-humble-ros2-controllers
USER $USER

#############################################################################
##   Stage 2: overwrite the comand from diy_cell image/ ##     
##############################################################################

#ATTENTION: currently WITH state publisher and joint state publisher gui, will be deleted from launch file,
#           because controller starts this node anyway 

# Add a default command to start visualization of the gripper by default whrn buildung the container




