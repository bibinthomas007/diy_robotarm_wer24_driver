##############################################################################
##        Stage 1: Driver Image from full_cell_description Image            ##
##############################################################################
ARG ROS_DISTRO=humble
# set description image (of running diy_robotarm_full_cell_description container) to there specified image name:
# --> diy-full-description/ros-render:"$ROS_DISTRO"
# we dont need to specify the whole docker stages from there, but sourcing inside this container from the image is necesary!
FROM diy-full-description-dev/ros-render:"$ROS_DISTRO" as diy-robotarm-driver


# Source and Build the diy-full cell description package (must be redone inside the driver-container)
RUN cd /home/$USER/dependencies/diy_robot_full_cell_description_ws && \
   . /opt/ros/$ROS_DISTRO/setup.sh && \
   . /home/$USER/dependencies/diy_robotarm_wer24_description_ws/install/setup.sh && \
   . /home/$USER/dependencies/diy_soft_gripper_description_ws/install/setup.sh && \
   colcon build

# Add built diy-full cell description package to entrypoint by calling install/setup.bash
USER root
RUN apt-get update && apt-get install -y ros-humble-controller-interface 
RUN apt-get update && apt-get install -y ros-humble-controller-manager 
RUN apt-get update && apt-get install -y ros-humble-hardware-interface 
RUN apt-get update && apt-get install -y ros-humble-pluginlib 
RUN apt-get update && apt-get install -y ros-humble-rclcpp
RUN apt-get update && apt-get install -y ros-humble-rclcpp-lifecycle
RUN apt-get update && apt-get install -y ros-humble-ros2-control


RUN sed -i 's|exec "\$@"|source "/home/'"${USER}"'/dependencies/diy_robot_full_cell_description_ws/install/setup.bash"\n&|' /ros_entrypoint.sh
USER $USER

#############################################################################
##   Stage 2: overwrite the comand from diy_cell image/ ##     
##############################################################################

#ATTENTION: currently WITH state publisher and joint state publisher gui, will be deleted from launch file,
#           because controller starts this node anyway 

# Add a default command to start visualization of the gripper by default whrn buildung the container




