# Use NVIDIA ROS 2 Humble base image with Isaac support
FROM nvidia/ros:humble-ros-base-ubuntu-jammy

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility
ENV ROS_DISTRO=humble

# Install system dependencies
RUN apt-get update && apt-get install -y \
    # Core dependencies
    python3-pip \
    python3-dev \
    build-essential \
    git \
    curl \
    wget \
    gnupg \
    lsb-release \
    # Isaac Sim dependencies
    libgomp1 \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    libgomp1 \
    # Additional graphics support \
    libglvnd0 \
    libglx0 \
    libegl1 \
    libx11-6 \
    && rm -rf /var/lib/apt/lists/*

# Upgrade pip
RUN pip3 install --upgrade pip setuptools

# Copy requirements
COPY requirements.txt .

# Install Python dependencies
RUN pip3 install -r requirements.txt

# Create workspace directory
WORKDIR /ws

# Copy the current directory contents into the workspace
COPY . .

# Source ROS 2
SHELL ['bash', '-c']
RUN source /opt/ros/humble/setup.bash && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Create src directory if it doesn't exist
RUN mkdir -p src

# Set up the workspace for colcon build
RUN source /opt/ros/humble/setup.bash && \
    cd /ws && \
    colcon build --symlink-install --packages-select \
    # Only build packages that exist

# Source the local setup
RUN echo "source /ws/install/setup.bash" >> ~/.bashrc

# Set entrypoint
ENTRYPOINT ["bash", "-c", "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && exec \"$@\"", "--"]