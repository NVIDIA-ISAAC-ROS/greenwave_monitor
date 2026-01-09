# Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('greenwave_monitor'),
        'config',
        'greenwave_monitor.yaml'
    )
    return LaunchDescription([
        Node(
            package='greenwave_monitor',
            executable='minimal_publisher_node',
            name='minimal_publisher_imu',
            output='log',
            parameters=[
                {
                    'topic': '/imu_topic', 'frequency_hz': 100.0,
                    'greenwave_diagnostics': {
                        '/imu_topic': {'expected_frequency': 100.0, 'tolerance': 5.0}
                    }
                }
            ],
        ),
        Node(
            package='greenwave_monitor',
            executable='minimal_publisher_node',
            name='minimal_publisher_image',
            output='log',
            parameters=[
                {
                    'topic': '/image_topic', 'message_type': 'image', 'frequency_hz': 30.0,
                    'greenwave_diagnostics': {
                        '/image_topic': {'expected_frequency': 30.0, 'tolerance': 5.0}
                    }
                }
            ],
        ),
        Node(
            package='greenwave_monitor',
            executable='minimal_publisher_node',
            name='minimal_publisher_string',
            output='log',
            parameters=[
                {
                    'topic': '/string_topic', 'message_type': 'string', 'frequency_hz': 1000.0,
                    'greenwave_diagnostics': {
                        '/string_topic': {'expected_frequency': 1000.0, 'tolerance': 10.0}
                    }
                }
            ],
        ),
        Node(
            package='greenwave_monitor',
            executable='minimal_publisher_node',
            name='minimal_publisher_params_from_yaml',
            output='log',
            parameters=[
                {
                    'topic': '/params_from_yaml_topic', 'message_type': 'imu',
                    'frequency_hz': 10.0, 'enable_greenwave_diagnostics': False
                }
            ],
        ),
        Node(
            package='greenwave_monitor',
            executable='minimal_publisher_node',
            name='minimal_publisher_no_startup_monitor',
            output='log',
            parameters=[
                {
                    'topic': '/no_startup_monitor_topic', 'message_type': 'imu',
                    'frequency_hz': 1.0, 'enable_greenwave_diagnostics': False
                }
            ],
        ),
        Node(
            package='greenwave_monitor',
            executable='greenwave_monitor',
            name='greenwave_monitor',
            output='log',
            parameters=[config_file]
        ),
        LogInfo(
            msg='Follow the instructions to setup r2s_gw in the README.md, then run '
                '`ros2 run r2s_gw r2s_gw` in another terminal to see the demo output '
                'with the r2s dashboard.'
        )
    ])
