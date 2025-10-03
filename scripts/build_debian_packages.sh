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

# Build Debian packages for greenwave_monitor
#
# This script builds .deb packages for greenwave_monitor_interfaces, greenwave_monitor, and r2s_gw
#
# Usage: ./scripts/build_debian_packages.sh [ROS_DISTRO] [UBUNTU_DISTRO]
# Examples:
#   ./scripts/build_debian_packages.sh                    # Uses defaults (humble/jammy)
#   ./scripts/build_debian_packages.sh humble jammy      # ROS Humble on Ubuntu 22.04
#   ./scripts/build_debian_packages.sh jazzy noble       # ROS Jazzy on Ubuntu 24.04
#
# Docker Usage (Recommended):
#   docker run -it --rm -v $(pwd):/workspace -w /workspace \
#     ros:humble-ros-base-jammy ./scripts/build_debian_packages.sh humble jammy
#
# Supported combinations:
#   humble/jammy, iron/jammy, jazzy/noble, kilted/noble, rolling/noble
#
# Output: Debian packages will be created in debian_packages/[ROS_DISTRO]/
# Install: sudo apt install ./debian_packages/[ROS_DISTRO]/*.deb

set -eo pipefail

# Default values
DEFAULT_ROS_DISTRO="humble"
DEFAULT_UBUNTU_DISTRO="jammy"

# Parse arguments
ROS_DISTRO="${1:-$DEFAULT_ROS_DISTRO}"
UBUNTU_DISTRO="${2:-$DEFAULT_UBUNTU_DISTRO}"

# Validate ROS distro
case "$ROS_DISTRO" in
  humble|iron|jazzy|kilted|rolling)
    ;;
  *)
    echo "Error: Unsupported ROS distro: $ROS_DISTRO"
    echo "Supported distros: humble, iron, jazzy, kilted, rolling"
    exit 1
    ;;
esac

# Validate Ubuntu distro
case "$UBUNTU_DISTRO" in
  jammy|noble)
    ;;
  *)
    echo "Error: Unsupported Ubuntu distro: $UBUNTU_DISTRO"
    echo "Supported distros: jammy, noble"
    exit 1
    ;;
esac

echo "Building Debian packages for ROS $ROS_DISTRO on Ubuntu $UBUNTU_DISTRO"

# Check if running in ROS container or local system
if [ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    echo "Error: ROS $ROS_DISTRO not found at /opt/ros/$ROS_DISTRO/setup.bash"
    echo "Please run this script in a ROS $ROS_DISTRO container or install ROS $ROS_DISTRO locally"
    echo ""
    echo "To run in Docker:"
    echo "  docker run -it --rm -v \$(pwd):/workspace -w /workspace ros:$ROS_DISTRO-ros-base-$UBUNTU_DISTRO ./scripts/build_debian_packages.sh $ROS_DISTRO $UBUNTU_DISTRO"
    exit 1
fi

# Source ROS environment
source /opt/ros/$ROS_DISTRO/setup.bash

# Install dependencies
echo "Installing build dependencies..."
apt-get update -qq

# Check if we need --break-system-packages for pip
USE_BREAK_SYSTEM_PACKAGES=""
if [[ "$ROS_DISTRO" == "jazzy" || "$ROS_DISTRO" == "kilted" || "$ROS_DISTRO" == "rolling" ]]; then
    USE_BREAK_SYSTEM_PACKAGES="--break-system-packages"
fi

# Install system dependencies
apt-get install -y build-essential python3-pip python3-bloom python3-rosdep git lsb-release devscripts debhelper fakeroot

# Install Python dependencies
if [ -f "requirements.txt" ]; then
    if [ -n "$USE_BREAK_SYSTEM_PACKAGES" ]; then
        pip3 install $USE_BREAK_SYSTEM_PACKAGES -I pygments -r requirements.txt
        python3 -m pip install -U $USE_BREAK_SYSTEM_PACKAGES bloom
    else
        pip3 install -r requirements.txt
        python3 -m pip install -U bloom
    fi
fi

# Build workspace first
echo "Building workspace..."
colcon build --packages-up-to r2s_gw
source install/setup.bash

# Setup rosdep mappings
echo "Setting up rosdep mappings..."
mkdir -p ~/.ros/rosdep
cat >~/.ros/rosdep/local.yaml <<EOF
greenwave_monitor_interfaces:
  ubuntu:
    jammy: [ros-$ROS_DISTRO-greenwave-monitor-interfaces]
    noble: [ros-$ROS_DISTRO-greenwave-monitor-interfaces]
greenwave_monitor:
  ubuntu:
    jammy: [ros-$ROS_DISTRO-greenwave-monitor]
    noble: [ros-$ROS_DISTRO-greenwave-monitor]
EOF

# Setup rosdep
mkdir -p /etc/ros/rosdep/sources.list.d || sudo mkdir -p /etc/ros/rosdep/sources.list.d
echo "yaml file://$HOME/.ros/rosdep/local.yaml" | sudo tee /etc/ros/rosdep/sources.list.d/99-local.list
rosdep init 2>/dev/null || sudo rosdep init 2>/dev/null || true
rosdep update --include-eol-distros

# Create output directory
DEBIAN_DIR="debian_packages/$ROS_DISTRO"
mkdir -p "$DEBIAN_DIR"

# Function to build a Debian package
build_debian_package() {
    local package_name=$1
    local package_dir=$2

    echo "=================================="
    echo "Generating Debian package for $package_name..."
    echo "=================================="

    cd "$package_dir"

    # Generate debian files and build package
    bloom-generate rosdebian --ros-distro "$ROS_DISTRO"
    apt-get build-dep . -y || sudo apt-get build-dep . -y
    fakeroot debian/rules binary

    # Move package to output directory
    cp ../ros-$ROS_DISTRO-${package_name//_/-}_*.deb "../$DEBIAN_DIR/"

    cd ..

    echo "Successfully built $package_name"
}

# Function to install a package locally
install_package() {
    local package_pattern=$1
    echo "Installing $package_pattern..."
    apt-get update || sudo apt-get update
    apt-get install -y ./$DEBIAN_DIR/$package_pattern || sudo apt-get install -y ./$DEBIAN_DIR/$package_pattern
}

# Build packages in dependency order
echo "Starting Debian package generation..."

# 1. Build greenwave_monitor_interfaces
if [ -d "greenwave_monitor_interfaces" ]; then
    build_debian_package "greenwave_monitor_interfaces" "greenwave_monitor_interfaces"
    install_package "ros-$ROS_DISTRO-greenwave-monitor-interfaces_*.deb"
else
    echo "Warning: greenwave_monitor_interfaces directory not found, skipping"
fi

# 2. Build greenwave_monitor
if [ -d "greenwave_monitor" ]; then
    build_debian_package "greenwave_monitor" "greenwave_monitor"
    install_package "ros-$ROS_DISTRO-greenwave-monitor_*.deb"
else
    echo "Warning: greenwave_monitor directory not found, skipping"
fi

# 3. Build r2s_gw
if [ -d "r2s_gw" ]; then
    build_debian_package "r2s_gw" "r2s_gw"
else
    echo "Warning: r2s_gw directory not found, skipping"
fi

echo "=================================="
echo "Debian package generation complete!"
echo "=================================="
echo "Generated packages in $DEBIAN_DIR:"
ls -la "$DEBIAN_DIR/"

echo ""
echo "To install the packages on another system:"
echo "  sudo apt install ./$DEBIAN_DIR/*.deb"

echo ""
echo "To create a local APT repository:"
echo "  sudo apt install dpkg-dev"
echo "  cd $DEBIAN_DIR"
echo "  dpkg-scanpackages . /dev/null | gzip -9c > Packages.gz"
echo "  # Then add 'deb [trusted=yes] file:///path/to/$DEBIAN_DIR ./' to /etc/apt/sources.list"
