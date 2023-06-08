# Copyright 2022 Amadeusz Szymko
# Perception for Physical Interaction Laboratory at Poznan University of Technology
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    phoxi_collector_launch_pkg_prefix = get_package_share_directory("phoxi_collector")

    phoxi_collector_config_param = DeclareLaunchArgument(
        'phoxi_collector_config_param_file',
        default_value=[phoxi_collector_launch_pkg_prefix, '/param/defaults.param.yaml'],
        description='Phoxi collector node config.'
    )

    phoxi_collector_node = Node(
        package='phoxi_collector',
        executable='phoxi_collector_node_exe',
        name='phoxi_collector',
        namespace='phoxi_collector',
        output='screen',
        remappings=[
                ("image_raw", "/camera/image_raw")
                ("depth_raw", "/camera/depth_raw")
                ],
        parameters=[
            LaunchConfiguration('phoxi_collector_config_param_file')
        ],
        arguments=['--ros-args', '--log-level', 'info', '--enable-stdout-logs']
    )

    ld = LaunchDescription([
        phoxi_collector_config_param,
        phoxi_collector_node,
    ])
    return ld
