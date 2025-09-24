# Copyright (c) 2025, NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='greenwave_monitor',
            executable='minimal_publisher_node',
            name='minimal_publisher1',
            output='log',
            parameters=[
                {'topic': 'imu_topic', 'frequency_hz': 100.0}
            ],
        ),
        Node(
            package='greenwave_monitor',
            executable='minimal_publisher_node',
            name='minimal_publisher2',
            output='log',
            parameters=[
                {'topic': 'image_topic', 'message_type': 'image', 'frequency_hz': 30.0}
            ],
        ),
        Node(
            package='greenwave_monitor',
            executable='minimal_publisher_node',
            name='minimal_publisher3',
            output='log',
            parameters=[
                {'topic': 'string_topic', 'message_type': 'string', 'frequency_hz': 1000.0}
            ],
        ),
        Node(
            package='greenwave_monitor',
            executable='greenwave_monitor',
            name='greenwave_monitor',
            output='log',
            parameters=[
                {'topics': ['/imu_topic', '/image_topic', '/string_topic']}
            ],
        ),
        LogInfo(
            msg='Run `ros2 run r2s_gw r2s_gw` in another terminal to see the demo output '
                'with the r2s dashboard.'
        ),
    ])
