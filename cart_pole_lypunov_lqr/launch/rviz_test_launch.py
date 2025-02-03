#!/usr/bin/env python3

from launch import LaunchDescription
from launch.substitutions import (
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # controller_params = PathJoinSubstitution(
    #     [
    #         FindPackageShare("cart_pole_lypunov_lqr"),
    #         "config",
    #         "controller.yaml",
    #     ]
    # )

    cart_pole_lypunov_lqr_node = Node(
        package="cart_pole_lypunov_lqr",
        executable="controller",
    )

    actions = [
        cart_pole_lypunov_lqr_node,
    ]

    return LaunchDescription(actions)
