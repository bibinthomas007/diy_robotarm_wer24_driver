from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
from launch.conditions import IfCondition


def generate_launch_description():
#define the packages for driver and description
    description_package = "diy_robot_full_cell_description"
    arm_driver_package = "diy_robotarm_wer24_driver"

#declare launch arguments (can be passed in the command line while launching)
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='""',
            description="Prefix for the links and joints in the robot cell",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix_sub",
            default_value='"sub_"',
            description="Prefix for the subframe of the cell, should be unique to avoid namespace collisions",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix_arm",
            default_value='"arm_"',
            description="Prefix for the robotarm inside the cell, should be unique to avoid namespace collisions",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix_grip",
            default_value='"grip_"',
            description="Prefix for the gripper inside the cell, should be unique to avoid namespace collisions",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value='"true"',
            description="start the robot with fake (mock) hardware or real controller",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value='"192.168.212.203"',
            description="The IP-Adress with which the robot hardware joins the common network",
        )
    )  
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ssid",
            default_value='"DIY-Robotics"',
            description="The SSID from the common network (PC and ESP must be member of this network)",
        )
    )
    declared_arguments.append(
    DeclareLaunchArgument(
      "rviz",
      default_value="false",
      description="Start RViz2 automatically with this launch file, shoulf be deactivated when launching moveit from this base image.",
    )
  )


    tf_prefix = LaunchConfiguration("tf_prefix")
    tf_prefix_sub = LaunchConfiguration("tf_prefix_sub")
    tf_prefix_arm = LaunchConfiguration("tf_prefix_arm")
    tf_prefix_grip = LaunchConfiguration("tf_prefix_grip")

    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_ip = LaunchConfiguration("robot_ip")
    robot_ssid = LaunchConfiguration("robot_ssid")

    rviz = LaunchConfiguration("rviz")



#define the robot description content
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", "cell_model.urdf.xacro"]),
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "tf_prefix_sub:=",
            tf_prefix_sub,
            " ",
            "tf_prefix_arm:=",
            tf_prefix_arm,
            " ",
            "tf_prefix_grip:=",
            tf_prefix_grip,
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "robot_ssid:=",
            robot_ssid,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
         ]
    )

    #load the description and the controllers from desired package
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)} 

    #load the rviz config file with visualization settings
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "rviz_config.rviz"]
    )

    #load the controller manager yaml
    robot_controllers = PathJoinSubstitution([FindPackageShare(arm_driver_package), "config", "esp32_controller.yaml"])


#define the nodes to launch 
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],  #,{"tf_prefix": tf_prefix, "tf_prefix_arm": tf_prefix_arm}
        output="both",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(rviz)
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
    )


    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nodes_to_start = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)