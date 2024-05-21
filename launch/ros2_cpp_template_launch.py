#!/usr/bin/env python3

"""Launch ros2_cpp_template node."""

# Copyright 2024 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'
__version__ = '0.1.0'

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def get_node(context, *args, **kwargs) -> list:
    """
    Get node.

    :param context: Launch context
    :type context: LaunchContext
    :return: List with node
    :rtype: list[Node]
    """
    node = Node(
        package='ros2_cpp_template',
        executable='ros2_cpp_template_node',
        name=LaunchConfiguration('name'),
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time')
            },
            LaunchConfiguration('config_file')
        ]
    )
    return [node]


def generate_launch_description() -> LaunchDescription:
    """
    Entry point for launch file.

    :return: Launch description
    :rtype: LaunchDescription
    """
    config_file = PathJoinSubstitution([
        FindPackageShare('ros2_cpp_template'),
        'config', 'config_file.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='ros2_cpp_template_node'),
        DeclareLaunchArgument(
            'name', default_value='ros2_cpp_template'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('config_file',
                              default_value=config_file,
                              description='Configuration file'),
        OpaqueFunction(function=get_node)
    ])
