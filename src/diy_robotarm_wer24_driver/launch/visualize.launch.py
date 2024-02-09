from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    description_package = "diy_robot_full_cell_description"

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
            description="start the robot with fake(mock) hardware or real controller",
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


    tf_prefix = LaunchConfiguration("tf_prefix")
    tf_prefix_sub = LaunchConfiguration("tf_prefix_sub")
    tf_prefix_arm = LaunchConfiguration("tf_prefix_arm")
    tf_prefix_grip = LaunchConfiguration("tf_prefix_grip")

    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_ip = LaunchConfiguration("robot_ip")
    robot_ssid = LaunchConfiguration("robot_ssid")


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

    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)} 
    

    rviz_config_file = PathJoinSubstitution(
    [FindPackageShare(description_package), "rviz", "rviz_config.rviz"])

    #run joint_state_publisher node, because we don't need to bringup the robot model, but initial_positions.yaml get overwritten!
    #-> only for visualizing purposes     
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    #run joint_state_publisher node, because we don't need to bringup the robot model (without no transforms !) --> only for visualizing porposes     
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file]
    )

    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)