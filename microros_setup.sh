#!/bin/bash
set -e 

# Install dependencies
sudo apt install -y cmake g++ gcc-arm-none-eabi doxygen libnewlib-arm-none-eabi git python3 build-essential pkg-config libusb-1.0-0-dev
sudo apt-get install usbutils

# Install Pico SDK
cd $HOME/
git clone --recurse-submodules https://github.com/raspberrypi/pico-sdk.git $HOME/pico-sdk
echo "export PICO_SDK_PATH=$HOME/pico-sdk" >> ~/.bashrc
export PICO_SDK_PATH=$HOME/pico-sdk
source ~/.bashrc

# Install microros Pico SDK
git clone -b humble https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk.git $HOME/micro_ros_raspberrypi_pico_sdk
sudo chmod u+rwx $HOME/micro_ros_raspberrypi_pico_sdk
cd $HOME/micro_ros_raspberrypi_pico_sdk
mkdir -p build && cd build
cmake ..
make -j4

# Install Picotool
cd $HOME/
git clone https://github.com/raspberrypi/picotool $HOME/picotool
sudo chmod u+rwx $HOME/picotool
cd $HOME/picotool
mkdir -p build && cd build
cmake ..
make -j4

source /opt/ros/humble/setup.bash

# Create and build microros workspace
cd $HOME/
mkdir -p micro_ros_agent && cd micro_ros_agent
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build
source install/local_setup.bash

# Build the microros firmware and agent
ros2 run micro_ros_setup create_firmware_ws.sh host
ros2 run micro_ros_setup build_firmware.sh
source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.sh ros2 run

# Cannot run microros where there are multiple instances of ROS
unset ROS_DOMAIN_ID
