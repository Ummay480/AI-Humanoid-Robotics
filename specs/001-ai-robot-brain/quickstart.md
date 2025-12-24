# Quickstart Guide: AI-Robot Brain (NVIDIA Isaac™)

## Overview
This guide will help you set up and run the AI-Robot Brain module using NVIDIA Isaac technology stack. The module includes Isaac Sim for photorealistic simulation, Isaac ROS for hardware-accelerated VSLAM, and Nav2 for bipedal humanoid path planning.

## Prerequisites

### Hardware Requirements
- NVIDIA RTX 30/40 series GPU or equivalent (with CUDA support)
- 32GB+ RAM recommended
- Multi-core CPU (8+ cores recommended)
- 50GB+ free disk space

### Software Requirements
- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- NVIDIA GPU drivers (535+)
- CUDA 11.8+
- Isaac Sim 2023.1+
- Isaac ROS 3.0+

## Installation

### 1. Set up ROS 2 Environment
```bash
# Add ROS 2 repository
sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-ros-base python3-rosdep python3-rosinstall python3-rosinstall-generator python3-build
```

### 2. Install Isaac Sim
```bash
# Download and install Isaac Sim from NVIDIA Developer website
# Follow the official installation guide for your platform
# Verify installation by launching Isaac Sim
```

### 3. Install Isaac ROS Dependencies
```bash
# Install Isaac ROS packages
sudo apt install -y ros-humble-isaac-ros-common
sudo apt install -y ros-humble-isaac-ros-perception
sudo apt install -y ros-humble-isaac-ros-buffers
sudo apt install -y ros-humble-isaac-ros-gems

# Install additional dependencies
sudo apt install -y nvidia-ml-dev cuda-toolkit-11-8
```

### 4. Install Navigation2
```bash
# Install Nav2 packages
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup
```

## Quick Setup

### 1. Clone the Repository
```bash
git clone https://github.com/your-organization/ai-robot-brain.git
cd ai-robot-brain
```

### 2. Build the Project
```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Create colcon workspace
mkdir -p src
cd src
# Copy or link your packages here

# Build the workspace
cd ..
colcon build --symlink-install
source install/setup.bash
```

## Running the AI-Robot Brain

### 1. Launch Simulation Environment
```bash
# Launch Isaac Sim with a basic environment
ros2 launch ai_robot_brain simulation.launch.py
```

### 2. Start Perception System
```bash
# Launch the perception pipeline
ros2 launch ai_robot_brain perception_pipeline.launch.py
```

### 3. Enable VSLAM
```bash
# Launch Visual SLAM for mapping and localization
ros2 launch ai_robot_brain vslam.launch.py
```

### 4. Start Bipedal Navigation
```bash
# Launch the path planning and navigation system
ros2 launch ai_robot_brain bipedal_navigation.launch.py
```

## Basic Examples

### Example 1: Object Detection in Simulation
```bash
# Run perception pipeline with object detection
ros2 launch ai_robot_brain perception_demo.launch.py

# Monitor the results in RViz2
ros2 run rviz2 rviz2 -d ./config/perception_view.rviz
```

### Example 2: Mapping an Unknown Environment
```bash
# Launch the robot in an unknown environment
ros2 launch ai_robot_brain mapping_demo.launch.py

# Visualize the generated map
ros2 run rviz2 rviz2 -d ./config/mapping_view.rviz
```

### Example 3: Bipedal Path Planning
```bash
# Launch navigation with path planning
ros2 launch ai_robot_brain path_planning_demo.launch.py

# Send navigation goals
ros2 run nav2_msgs send_goal 1.0 1.0 0.0
```

## Key Configuration Files

- `config/perception.yaml`: Perception pipeline configuration
- `config/vslam.yaml`: Visual SLAM parameters
- `config/nav2_params.yaml`: Navigation2 configuration for bipedal robots
- `config/simulation.yaml`: Isaac Sim environment settings

## Troubleshooting

### Common Issues

1. **GPU Acceleration Not Working**
   - Ensure NVIDIA drivers are properly installed
   - Verify CUDA is accessible: `nvidia-smi`
   - Check Isaac ROS packages are built with GPU support

2. **Simulation Performance Issues**
   - Reduce simulation complexity
   - Check GPU memory usage
   - Verify sufficient system RAM

3. **ROS 2 Communication Issues**
   - Source the ROS 2 environment: `source install/setup.bash`
   - Check network configuration if using multi-machine setup
   - Verify ROS domain ID settings

## Next Steps

1. Explore the complete documentation in the `docs/` directory
2. Try the tutorial examples in `examples/`
3. Customize the simulation environments in `simulation_envs/`
4. Extend the perception pipeline with your own models