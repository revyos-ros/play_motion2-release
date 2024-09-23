# Copyright (c) 2022 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import pathlib
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

RRBOT_DIR = str(pathlib.Path(__file__).resolve().parent)


def generate_launch_description():

    # Visualize the robot for debugging purposes
    use_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Whether run rviz',
        choices=['True', 'False'])

    robot_description_config = xacro.process_file(RRBOT_DIR + '/rrbot.xacro')
    robot_description = {'robot_description': robot_description_config.toxml()}

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, RRBOT_DIR + '/controllers.yaml'],
        output='both')

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    controller_1_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["controller_1", "--controller-manager", "/controller_manager"],
    )

    controller_2_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["controller_2", "--controller-manager", "/controller_manager"],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description])

    rviz_config_file = RRBOT_DIR + '/rrbot.rviz'
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    ld = LaunchDescription()

    ld.add_action(use_rviz)

    ld.add_action(control_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(controller_1_spawner)
    ld.add_action(controller_2_spawner)
    ld.add_action(rviz_node)

    return ld
