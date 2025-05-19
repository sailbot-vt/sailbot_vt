FROM ros:humble
ARG USERNAME=sailbot_user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
ENV SHELL /bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

RUN sudo apt install ros-dev-tools -y
RUN sudo apt install python3-opencv -y

# This is very unsafe lol but I need this to be able to access docker from inside of the dev container for the sim. 
# Do not remove unless we are releasing this software as a product
RUN echo "${USERNAME} ALL=(ALL) NOPASSWD: /usr/bin/chmod" >> /etc/sudoers
RUN echo "${USERNAME} ALL=(ALL) NOPASSWD: /usr/sbin/service" >> /etc/sudoers

# The following is to set up ubuntu repositories to download ros packages
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN sudo apt install curl
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# install camera ros drivers
#RUN sudo apt install ros-humble-librealsense2* -y
#RUN sudo apt install ros-humble-realsense2-* -y

#Used to automate tasks
RUN sudo apt-get install cron
RUN sudo apt-get install usbutils
RUN sudo apt-get install ros-humble-cv-bridge -y


USER $USERNAME
# [Optional] Set the default user. Omit if you want to keep the default as root.
CMD ["/bin/bash"]
