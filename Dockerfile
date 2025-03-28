# Use an official Ubuntu 20.04 LTS as a parent image
FROM osrf/ros:noetic-desktop-full

# Set noninteractive to avoid prompts during the build
ENV DEBIAN_FRONTEND=noninteractive

# Update apt package list and install general packages
RUN chmod 1777 /tmp
RUN apt-get update && \
    apt-get install -y --fix-missing \
    curl\
    python3-pip\
    build-essential\ 
    cmake\
    nano \
    libeigen3-dev\
    python3-catkin-tools \
    ros-noetic-libfranka ros-noetic-franka-ros \
    ros-noetic-urdfdom-py \
    ros-noetic-kdl-parser-py \
    ros-noetic-kdl-conversions\
    ros-noetic-spacenav-node \
    # ros-noetic-foxglove-bridge \ 
    ros-noetic-pr2-description \
    ros-noetic-rosbridge-server \
    ros-noetic-tf2-web-republisher \
    ros-noetic-interactive-marker-tutorials \
    usbutils \

# Install python packages
RUN pip install websockets\
    scipy

# Add python alias to python3
RUN ln -s /usr/bin/python3 /usr/bin/python

# Setup spacenavd (can't use apt-get bc that version is too old)
COPY . /workspace
WORKDIR /workspace/

WORKDIR /workspace/panda-primitives-control/spacenavd
RUN ./configure
RUN make install
WORKDIR /workspace/

# Compile panda-primitives package
WORKDIR /workspace/panda-primitives/
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash &&  catkin build authoring"
WORKDIR /workspace/

# Automatically load ROS environment when entering the terminal
RUN echo "source /workspace/panda-primitives/devel/setup.bash" >> ~/.bashrc

# Set the default command to execute
CMD ["bash"]