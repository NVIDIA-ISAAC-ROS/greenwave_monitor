#!/bin/bash

# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

# Docker test environment for Greenwave Monitor
# Quick way to test in different ROS 2 distributions

set -e

# Default to Humble if no distro specified
DISTRO=${1:-humble}

# Image mapping based on ROS distro
case $DISTRO in
    humble)
        IMAGE="ros:humble-ros-base-jammy"
        ;;
    iron)
        IMAGE="ros:iron-ros-base-jammy"
        ;;
    jazzy)
        IMAGE="ros:jazzy-ros-base-noble"
        ;;
    kilted)
        IMAGE="ros:kilted-ros-base-noble"
        ;;
    rolling)
        IMAGE="ros:rolling-ros-base-noble"
        ;;
    *)
        echo "Unsupported ROS 2 distribution: $DISTRO"
        echo "Supported: humble, iron, jazzy, kilted, rolling"
        exit 1
        ;;
esac

echo "Starting Docker container for ROS 2 $DISTRO..."
echo "Image: $IMAGE"

# Get current directory (should be the greenwave_monitor root)
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../"

# Run container with interactive shell, mounting current directory
docker run -it --rm \
    --name greenwave-test-${DISTRO} \
    -e ROS_LOCALHOST_ONLY=1 \
    -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    -e ROS_DISTRO=${DISTRO} \
    -v ${WORKSPACE_DIR}:/workspace/src/greenwave_monitor \
    -w /workspace \
    ${IMAGE} \
    bash -c "
        # Source ROS setup
        source /opt/ros/${DISTRO}/setup.bash

        # Install dependencies
        apt-get update -qq && apt-get install -y build-essential python3-pip

        # Install Python requirements based on ROS distro
        if [[ '${DISTRO}' == 'jazzy' || '${DISTRO}' == 'kilted' || '${DISTRO}' == 'rolling' ]]; then
            pip3 install --break-system-packages -I pygments -r /workspace/src/greenwave_monitor/r2s_gw/requirements.txt
        else
            pip3 install -r /workspace/src/greenwave_monitor/r2s_gw/requirements.txt
        fi

        echo '=== Building packages ==='
        colcon build --packages-up-to r2s_gw

        # Source the built workspace
        source install/setup.bash

        echo '=== Environment Ready ==='
        echo 'ROS 2 Distribution: ${DISTRO}'
        echo 'Workspace: /workspace'
        echo 'Source code: /workspace/src/greenwave_monitor'
        echo 'Built and sourced workspace'
        echo ''
        echo 'Available commands:'
        echo '  colcon test --packages-up-to r2s_gw                     # Run unit tests'
        echo '  ros2 run greenwave_monitor greenwave_monitor          # Run monitor'
        echo '  ros2 run greenwave_monitor r2s_gw_dashboard              # Run dashboard'
        echo '  ros2 service list | grep greenwave                    # Check services'
        echo '  ros2 topic list                                       # List topics'
        echo ''

        # Start interactive shell
        bash
    "