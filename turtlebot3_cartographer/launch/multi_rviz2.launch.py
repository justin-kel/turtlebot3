# Copyright (c) 2023 LG Electronics.
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


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, ThisLaunchFileDir
from nav2_common.launch import ParseMultiRobotPose

def generate_launch_description():
    """
    Bring up the multi-robots with given launch arguments.

    Launch arguments consist of robot name(which is namespace) and pose for initialization.
    Keep general yaml format for pose information.
    ex) robots:="robot1={};robot2={}"
    """
    # On this example all robots are launched with the same settings
    use_namespace = LaunchConfiguration('use_namespace')
    rviz_config = LaunchConfiguration('rviz_config')
    log_settings = LaunchConfiguration('log_settings', default='true')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='true',
        description='Whether to apply a namespace')
    
    # Declare the launch arguments
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        condition=IfCondition(use_namespace),
        default_value=os.path.join(get_package_share_directory('turtlebot3_cartographer'),
                                   'rviz', 'tb3_cartographer_namespaced.rviz'))

    robots_list = ParseMultiRobotPose('robots').value()

    # Define commands for launching the navigation instances
    bringup_cmd_group = []
    for robot_name in robots_list:
        group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/cartographer_rviz.launch.py']),
            launch_arguments={'use_namespace': use_namespace,
                            'namespace': TextSubstitution(text=robot_name),
                            'rviz_config': rviz_config}.items()
            )])

        bringup_cmd_group.append(group)

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    # Add the actions to start gazebo, robots and simulations

    ld.add_action(LogInfo(msg=['number_of_robots=', str(len(robots_list))]))

    ld.add_action(LogInfo(condition=IfCondition(log_settings),
                          msg=['rviz config file: ', rviz_config]))

    for cmd in bringup_cmd_group:
        ld.add_action(cmd)

    return ld
