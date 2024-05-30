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
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    turtlebot3_cartographer_prefix = get_package_share_directory('turtlebot3_cartographer')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  turtlebot3_cartographer_prefix, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='turtlebot3_lds_2d.lua')

    resolution = LaunchConfiguration('resolution', default='0.05')

    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    namespace = LaunchConfiguration('namespace')

    use_namespace = LaunchConfiguration('use_namespace')
    
    rviz_config = LaunchConfiguration('rviz_config')
    
    use_rviz = LaunchConfiguration('use_rviz')

    declare_cartographer_config_dir_cmd = DeclareLaunchArgument(
                                            'cartographer_config_dir',
                                            default_value=cartographer_config_dir,
                                            description='Full path to config file to load')

    declare_configuration_basename_cmd = DeclareLaunchArgument(
        'configuration_basename',
        default_value=configuration_basename,
        description='Name of lua file for cartographer')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')
    
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Define ROS namespaces for Nodes')

    declare_resolution_cmd = DeclareLaunchArgument(
        'resolution',
        default_value=resolution,
        description='Resolution of a grid cell in the published occupancy grid')

    declare_publish_period_sec_cmd = DeclareLaunchArgument(
        'publish_period_sec',
        default_value=publish_period_sec,
        description='OccupancyGrid publishing period')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='OccupancyGrid publishing period')
    
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
    
    start_cartographer_cmd = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/map', 'map'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')]
        arguments=['-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename],
    )



    start_occupancy_grid_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time, 'resolution': resolution,
                            'publish_period_sec': publish_period_sec,
                            'namespace': namespace}.items(),
        remappings=[('/map', 'map'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')],
    )

    start_rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/cartographer_rviz.launch.py']),
        condition=IfCondition(use_rviz),  
        launch_arguments={'use_namespace': use_namespace,
                          'namespace': namespace, 
                          'rviz_config': rviz_config}.items()
    )

    ld = LaunchDescription()
    ld.add_action(declare_cartographer_config_dir_cmd)
    ld.add_action(declare_configuration_basename_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_resolution_cmd)
    ld.add_action(declare_publish_period_sec_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_default_rviz_config_file_cmd)
    ld.add_action(declare_namespaced_rviz_config_file_cmd)
    ld.add_action(start_cartographer_cmd)
    ld.add_action(start_occupancy_grid_cmd)
    ld.add_action(start_rviz_cmd)


    return ld