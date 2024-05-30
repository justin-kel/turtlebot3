# Copyright 2019 Open Source Robotics Foundation, Inc.
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
# Author: Darby Lim

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, AndSubstitution, NotSubstitution
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from nav2_common.launch import ReplaceString

def generate_launch_description():

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    rviz_config = LaunchConfiguration('rviz_config')
    
    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')
    
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Define ROS namespaces for Nodes')

    declare_default_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        condition=UnlessCondition(use_namespace),
        default_value=os.path.join(get_package_share_directory('turtlebot3_cartographer'),
                                   'rviz', 'tb3_cartographer.rviz'))
    
    declare_namespaced_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        condition=IfCondition(use_namespace),
        default_value=os.path.join(get_package_share_directory('turtlebot3_cartographer'),
                                   'rviz', 'tb3_cartographer_namespaced.rviz'))
    
    start_default_rviz_cmd = Node(
        condition=UnlessCondition(use_namespace),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        remappings=[('/map', 'map'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/goal_pose', 'goal_pose'),
            ('/clicked_point', 'clicked_point'),
            ('/initialpose', 'initialpose')])

    
    namespaced_rviz_config_file = ReplaceString(
            source_file=rviz_config,
            replacements={'<namespace>': ('/', namespace)})
    
    start_namespaced_rviz_cmd = Node(
        condition=IfCondition(use_namespace),
        package='rviz2',
        executable='rviz2',
        namespace=namespace,
        arguments=['-d', namespaced_rviz_config_file],
        output='screen',        
        remappings=[('/map', 'map'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/goal_pose', 'goal_pose'),
            ('/clicked_point', 'clicked_point'),
            ('/initialpose', 'initialpose')])
    
    ld = LaunchDescription()
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_default_rviz_config_file_cmd)
    ld.add_action(declare_namespaced_rviz_config_file_cmd)
    ld.add_action(start_default_rviz_cmd)
    ld.add_action(start_namespaced_rviz_cmd)


    return ld