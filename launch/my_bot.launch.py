#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
#
# Authors: Joep Tool

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('walker'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')

    world = os.path.join(
        get_package_share_directory('walker'),
        'worlds',
        'turtlebot3_world.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # Declare record_bag argument to control ros bag recording
    record_bag_arg = DeclareLaunchArgument(
        'enable_bag_record',
        default_value='false',  # Default to not recording if not specified
        description='Enable or disable ros bag recording'
    )

    
    # create handle for walker node
    walker_node = Node(
        package='walker',
        executable='walker_exec',
        name='walker',
        output='screen',
    )

    
    def launch_setup(context, *args, **kwargs):
    
        # Define the duration for which to record the bag (~15 seconds)
        record_duration = 15.0  # seconds

        # Set up the bag recording command
        record_bag = ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a', '-x', '/camera/.*'],
            condition=IfCondition(LaunchConfiguration('enable_bag_record')),
            output='screen',
            cwd='src/my_gazebo_tutorials/bag_files'
        )

        # Timer to stop the recording after ~15 seconds
        stop_recording = ExecuteProcess(
            cmd=['pkill', '-f', 'ros2 bag record'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_bag_record'))
        )

        return [record_bag, TimerAction(period=record_duration, actions=[stop_recording])]

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(record_bag_arg)
    ld.add_action(walker_node)
    ld.add_action(OpaqueFunction(function=launch_setup))


    return ld
