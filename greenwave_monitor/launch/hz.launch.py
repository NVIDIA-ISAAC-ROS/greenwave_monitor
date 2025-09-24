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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'topics',
            default_value='[""]',
            description='List of topics to monitor'
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='greenwave_monitor',
            executable='greenwave_monitor',
            name='greenwave_monitor',
            output='screen',
            parameters=[
                {'topics': LaunchConfiguration('topics'),
                 'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
        ),
    ])
