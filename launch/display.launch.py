#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('piklet_description')

    # Default paths
    default_model_path = PathJoinSubstitution([pkg_share, "urdf", "piklet_robot.urdf.xacro"])
    default_rviz_display_path = PathJoinSubstitution([pkg_share, "rviz", "display.rviz"])
    default_rviz_gazebo_path = PathJoinSubstitution([pkg_share, "rviz", "display_gazebo.rviz"])

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true"
    )

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=default_model_path,
        description="Absolute path to robot urdf.xacro file"
    )

    sim_arg = DeclareLaunchArgument(
        "sim",
        default_value="false",
        description="If true, load Gazebo-ready RViz config"
    )

    rviz_arg = DeclareLaunchArgument(
        "rvizconfig",
        default_value=LaunchConfiguration("sim").perform({}) == "true"
        if False else default_rviz_display_path,
        description="Absolute path to rviz config file"
    )

    # Robot description (xacro → urdf)
    robot_description = Command(["xacro ", LaunchConfiguration("model")])

    # Nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }]
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen"
    )

    # Choose RViz config based on sim arg
    rviz_config = PathJoinSubstitution([
        pkg_share,
        "rviz",
        LaunchConfiguration("sim").perform({}) == "true" and "display_gazebo.rviz" or "display.rviz"
    ])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        model_arg,
        sim_arg,
        rviz_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui,
        rviz_node,
    ])
