#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('piklet_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'piklet_robot.urdf.xacro')

    # Launch arguments
    sim = LaunchConfiguration('sim')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_sim = DeclareLaunchArgument(
        'sim', default_value='false', description='Use simulation mode (Gazebo) if true'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation clock if true'
    )

    # Robot description from xacro
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = {'robot_description': robot_description_content}

    # Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # Joint State Publisher GUI
    jsp_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # RViz Nodes with Conditions
    rviz_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_urdf',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'display.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(sim)  # Only if sim != true
    )

    rviz_gazebo_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_gazebo',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'display_gazebo.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(sim)  # Only if sim == true
    )

    return LaunchDescription([
        declare_sim,
        declare_use_sim_time,
        rsp_node,
        jsp_node,
        rviz_rviz_node,
        rviz_gazebo_node
    ])
