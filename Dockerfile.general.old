# This is the general dockerfile that can create an image from any ros package in this directory except (for now) uros 
# which has its own, separate dockerfile to run
# To run this dockerfile and create the image, all you need to do is use the following command: 
# docker build -t [name of the image you would like to create] --build-arg NODE_NAME=[name of the folder that the ros package is in] .

# The name of the image is arbitrary but the convention currently is to use the name: sailbot_[name of the folder that the ros package is in]
# so for example if I am building the rc node, I would do: docker build -t sailbot_rc --build-arg NODE_NAME=rc .


ARG ROS_DISTRO=humble

FROM ros:${ROS_DISTRO} as base

ENV ROS_DISTRO=${ROS_DISTRO}

ARG NODE_NAME
ENV NODE_NAME=${NODE_NAME}

RUN if [ -z "$NODE_NAME" ]; then echo 'Environment variable NODE_NAME must be specified. Exiting.'; exit 1; fi

SHELL ["/bin/bash", "-c"]


# Create Colcon workspace
RUN mkdir -p /sailbot_ws/src
WORKDIR /sailbot_ws/

RUN mkdir /sailbot_ws/src/${NODE_NAME}
RUN mkdir /sailbot_ws/src/sailbot_msgs

# Install Python Dependencies
RUN sudo apt-get update \
 && sudo apt install python3-pip -y

# this is necessary for open cv (TODO: Maybe find a way to only install these libraries if we are installing opencv in the image)
RUN apt-get update && apt-get install ffmpeg libsm6 libxext6  -y

RUN pip3 install setuptools==58.2.0

COPY ${NODE_NAME}/requirements.txt /sailbot_ws/src/${NODE_NAME}/requirements.txt
RUN pip3 install -r /sailbot_ws/src/${NODE_NAME}/requirements.txt



COPY sailbot_msgs ~/src/sailbot_msgs
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update -y \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
 && colcon build --symlink-install


# Copy in all of the packages required for the current node
COPY ${NODE_NAME} ~/src/${NODE_NAME}

# Build the base Colcon workspace, installing dependencies first.
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
 && colcon build --symlink-install --packages-select ${NODE_NAME}

# Start the ROS node through ros2 run
CMD source /opt/ros/${ROS_DISTRO}/setup.bash \ 
    && source install/setup.bash \
    && ros2 run ${NODE_NAME} launch_${NODE_NAME}_node