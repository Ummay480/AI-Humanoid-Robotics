#!/bin/bash
# Isaac ROS Setup Script
# This script outlines the steps for installing Isaac ROS packages as per T002 requirements

set -e  # Exit on any error

echo "Isaac ROS Installation Setup Script"
echo "====================================="

# Check if running on Ubuntu 22.04 (required for ROS 2 Humble)
if [ -f /etc/os-release ]; then
    . /etc/os-release
    if [ "$UBUNTU_CODENAME" != "jammy" ]; then
        echo "Warning: This script is designed for Ubuntu 22.04 (jammy). Current: $UBUNTU_CODENAME"
    fi
else
    echo "Cannot determine OS version, assuming Ubuntu 22.04"
fi

# Check if ROS 2 Humble is installed
if ! command -v ros2 &> /dev/null; then
    echo "ROS 2 Humble is not installed or not in PATH."
    echo "Please install ROS 2 Humble first:"
    echo "  sudo apt update && sudo apt install -y curl gnupg lsb-release"
    echo "  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg"
    echo "  echo 'deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main' | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null"
    echo "  sudo apt update"
    echo "  sudo apt install -y ros-humble-ros-base python3-rosdep python3-rosinstall python3-rosinstall-generator python3-build"
    echo "  sudo apt install -y python3-colcon-common-extensions"
    exit 1
else
    echo "✓ ROS 2 Humble is installed"
fi

# Check if Isaac ROS packages are installed via apt
echo "Checking for Isaac ROS packages..."
if apt list --installed | grep -q "isaac-ros"; then
    echo "✓ Isaac ROS packages appear to be installed via apt"
else
    echo "! Isaac ROS packages not found via apt"
    echo "  Installing Isaac ROS common packages:"
    sudo apt update
    sudo apt install -y ros-humble-isaac-ros-common ros-humble-isaac-ros-perception ros-humble-isaac-ros-buffers ros-humble-isaac-ros-gems ros-humble-isaac-ros-image-transport
fi

# Create Isaac ROS workspace if it doesn't exist
WORKSPACE_DIR="$HOME/isaac_ros_ws"
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo "Creating Isaac ROS workspace at $WORKSPACE_DIR"
    mkdir -p "$WORKSPACE_DIR/src"
    echo "✓ Isaac ROS workspace created"
else
    echo "✓ Isaac ROS workspace already exists at $WORKSPACE_DIR"
fi

# Check for additional dependencies
echo "Installing additional dependencies..."
sudo apt install -y python3-opencv python3-pip
pip3 install opencv-python

# Verify installation
echo "Verifying Isaac ROS installation..."
if ros2 pkg list | grep -i isaac; then
    echo "✓ Isaac ROS packages found in ROS environment"
else
    echo "! No Isaac ROS packages found in ROS environment"
    echo "  Make sure to source ROS environment: source /opt/ros/humble/setup.bash"
fi

echo ""
echo "Isaac ROS Installation Setup Complete!"
echo "======================================"
echo "To use Isaac ROS:"
echo "1. Source ROS 2: source /opt/ros/humble/setup.bash"
echo "2. Navigate to workspace: cd $WORKSPACE_DIR"
echo "3. Build workspace: colcon build --symlink-install"
echo "4. Source workspace: source install/setup.bash"
echo ""
echo "To verify GPU acceleration, run: python3 src/isaac_ros/test_installation.py"