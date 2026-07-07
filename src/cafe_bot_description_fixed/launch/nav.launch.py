#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Package directories
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    package_dir = get_package_share_directory("cafe_bot_description_fixed")

    # Map and parameter files
    map_file = os.path.join(
        package_dir,
        "map",
        "cafe_map_1.yaml"
    )

    params_file = os.path.join(
        package_dir,
        "config",
        "nav2_params.yaml"
    )

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    nav2_bringup_dir,
                    "launch",
                    "bringup_launch.py"
                )
            ),
            launch_arguments={
                "namespace": "",
                "use_namespace": "false",
                "slam": "False",
                "map": map_file,
                "use_sim_time": "true",
                "params_file": params_file,
                "autostart": "true",
                "use_composition": "True",
                "use_respawn": "False",
                "log_level": "info",
            }.items(),
        )

    ])
