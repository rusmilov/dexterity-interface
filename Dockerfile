# Use ROS Noetic Desktop Full as the base image
FROM osrf/ros:noetic-desktop-full

ARG DEBIAN_FRONTEND=noninteractive

# Set up environment
ENV DEBIAN_FRONTEND=noninteractive
ENV PULSE_SERVER=unix:/run/user/1000/pulse/native

# Update and install system dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    curl \
    wget \
    ca-certificates \
    build-essential \
    cmake \
    nano \
    git \
    python3-pip \
    libeigen3-dev \
    libpoco-dev \
    libasound2 \
    libasound2-plugins \
    alsa-utils \
    pulseaudio \
    pulseaudio-utils \
    usbutils \
    v4l-utils \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libxi6 \
    libxrender1 \
    libxtst6 \
    libusb-1.0-0-dev \
    gnupg && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*


# Install core ROS & robotics dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    python3-catkin-tools \
    ros-noetic-libfranka ros-noetic-franka-ros \
    ros-noetic-urdfdom-py \
    ros-noetic-kdl-parser-py \
    ros-noetic-kdl-conversions \
    ros-noetic-spacenav-node \
    ros-noetic-pr2-description \
    ros-noetic-rosbridge-server \
    ros-noetic-tf2-web-republisher \
    ros-noetic-interactive-marker-tutorials \
    ros-noetic-interactive-markers \
    ros-noetic-tf2-tools


# Add python alias to python3
RUN ln -s /usr/bin/python3 /usr/bin/python || true

# Install Azure Kinect SDK
RUN curl -sSL https://packages.microsoft.com/keys/microsoft.asc | apt-key add - && \
    echo "deb [arch=amd64] https://packages.microsoft.com/ubuntu/18.04/prod bionic main" > /etc/apt/sources.list.d/microsoft-prod.list && \
    apt-get update && \
    ACCEPT_EULA=Y apt-get install -y k4a-tools libk4a1.4 libk4a1.4-dev && \
    rm -rf /var/lib/apt/lists/*



COPY . /workspace
WORKDIR /workspace


WORKDIR /workspace/panda-primitives-control/spacenavd
RUN ./configure && make install


WORKDIR /workspace

RUN pip install --no-cache-dir -r requirements.txt

# Expose PulseAudio socket for mic support
VOLUME ["/run/user/1000/pulse"]

# Default command
CMD ["bash"]
