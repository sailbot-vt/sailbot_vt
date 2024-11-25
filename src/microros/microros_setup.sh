#!/bin/bash
set -e
SCRIPT_DIR="$(dirname "$(realpath "$0")")"

# Install dependencies
sudo apt install -y cmake g++ gcc-arm-none-eabi doxygen libnewlib-arm-none-eabi git python3 build-essential pkg-config libusb-1.0-0-dev
sudo apt-get update && sudo apt-get upgrade -y
source /opt/ros/humble/setup.bash

# Install Pico SDK
git clone --recurse-submodules https://github.com/raspberrypi/pico-sdk.git $SCRIPT_DIR/dependencies/pico-sdk
export PICO_SDK_PATH=$SCRIPT_DIR/dependencies/pico-sdk

# Install microros Pico SDK
git clone -b humble https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk.git $SCRIPT_DIR/dependencies/micro_ros_raspberrypi_pico_sdk
export PICO_MICROROS_SDK_PATH=$SCRIPT_DIR/dependencies/micro_ros_raspberrypi_pico_sdk

# Install Picotool
git clone https://github.com/raspberrypi/picotool $SCRIPT_DIR/dependencies/picotool
cd $SCRIPT_DIR/dependencies/picotool
export PICOTOOL_PATH=$SCRIPT_DIR/dependencies/picotool
mkdir -p build && cd build
cmake .. && cmake --build .

# Create and build microros workspace
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git $SCRIPT_DIR/dependencies/micro_ros_agent/src/micro_ros_setup
cd $SCRIPT_DIR/dependencies/micro_ros_agent
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source $SCRIPT_DIR/dependencies/micro_ros_agent/install/local_setup.bash

# # Build the microros firmware and agent
ros2 run micro_ros_setup create_firmware_ws.sh host
ros2 run micro_ros_setup build_firmware.sh
source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.sh ros2 run

# Write sources and aliases to bashrc file
{
    echo "unset ROS_DOMAIN_ID"
    echo "export PICO_SDK_PATH=$SCRIPT_DIR/dependencies/pico-sdk"
    echo "export PICO_MICROROS_SDK_PATH=$SCRIPT_DIR/dependencies/micro_ros_raspberrypi_pico_sdk"
    echo "export PICOTOOL_PATH=$SCRIPT_DIR/dependencies/picotool"
    echo "source $SCRIPT_DIR/dependencies/micro_ros_agent/install/local_setup.bash"
    echo "alias picotool='$PICOTOOL_PATH/build/picotool'"
    echo "alias picotool_load='sudo $PICOTOOL_PATH/build/picotool load'"
    echo "alias picotool_reboot='sudo $PICOTOOL_PATH/build/picotool reboot'"
} >> ~/.bashrc

# Source bash
source ~/.bashrc

echo "Setup complete."