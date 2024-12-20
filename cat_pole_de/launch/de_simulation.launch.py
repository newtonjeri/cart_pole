#!/usr/bin/env python3

import os
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
)
from launch.substitutions import (
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_yaml(package_name, directory_name, file_name):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, directory_name, file_name)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("cart_pole_description"),
                    "launch",
                    "description.launch.py",
                ]
            )
        ),
    )


    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("cat_pole_de"),
            "rviz",
            "cart_pole.rviz",
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
    )

    simulation_params = PathJoinSubstitution(
        [
            FindPackageShare("cat_pole_de"),
            "config",
            "de_simulation.yaml",
        ]
    )
    furuta_pendulum_simulation_node = Node(
        package="cat_pole_de",
        executable="de_simulation_node",
        parameters=[simulation_params],
    )

    actions = [
        description_launch,
        rviz_node,
        furuta_pendulum_simulation_node,
    ]

    return LaunchDescription(actions)
