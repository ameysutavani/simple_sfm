#!/bin/bash

# Update package lists
sudo apt update

# Install ros-noetic-pcl-conversions
sudo apt install ros-noetic-pcl-conversions -y --no-install-recommends

# Install conan
pip3 install conan==1.64.0
