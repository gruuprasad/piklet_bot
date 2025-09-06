#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('piklet_description')
    display_launch = os.path.join(pkg_share, 'launch', 'display.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(display_launch),
            # Pass launch arguments as a list of tuples
            launch_arguments={
                'sim': 'false',
                'use_sim_time': 'false'
            }.items()
        )
    ])
