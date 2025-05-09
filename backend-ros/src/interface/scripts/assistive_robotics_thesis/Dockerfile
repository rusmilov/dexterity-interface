FROM osrf/ros:noetic-desktop-full
ARG DEBIAN_FRONTEND=noninteractive

# Update apt package list and install general packages
RUN apt-get update && \
    apt-get install -y --fix-missing \
    curl\
    python3-pip\
    build-essential\ 
    cmake\
    libpoco-dev\ 
    libeigen3-dev\
    python3-rosdep\
    mesa-utils\
    nano\
    pulseaudio\
    pulseaudio-utils\
    alsa-base\
    alsa-utils\
    libasound2\
    libasound2-plugins\
    pulseaudio\
    python3-catkin-tools\ 
    ros-noetic-gazebo-ros-control\
    ros-noetic-rospy-message-converter\
    ros-noetic-effort-controllers\
    ros-noetic-joint-state-controller\
    ros-noetic-moveit\
    ros-noetic-moveit-commander\
    ros-noetic-moveit-visual-tools\
    ros-noetic-rgbd-launch\
    v4l-utils\
    usbutils\
    python3-tk\
    v4l-utils


# Install Azure Kinect SDK dependencies
RUN curl -sSL https://packages.microsoft.com/keys/microsoft.asc | apt-key add - && \
    echo "deb [arch=amd64] https://packages.microsoft.com/ubuntu/18.04/prod bionic main" > /etc/apt/sources.list.d/microsoft-prod.list && \
    apt-get update && \
    ACCEPT_EULA=Y DEBIAN_FRONTEND=noninteractive apt-get install -y k4a-tools libk4a1.4 libk4a1.4-dev


# Install python packages
RUN pip install future\
    PyYaml\
    urdf-parser-py\
    panda_robot\
    numpy==1.21


# Install model workflow packages
RUN pip install \
    opencv-python \
    transformers \
    torch \
    matplotlib \
    pillow \
    requests \
    python-dotenv \
    azure-cognitiveservices-speech \
    pyk4a \
    einops \
    timm


# Install rosdep updates
COPY . /workspace
WORKDIR /workspace/
#RUN rosdep init 
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src --rosdistro noetic  -y --skip-keys="libfranka K4A ros-noetic-rgbd-launch"



WORKDIR /workspace/


# Set PulseAudio environment variable
ENV PULSE_SERVER=unix:/run/user/1000/pulse/native

# Expose PulseAudio socket
VOLUME ["/run/user/1000/pulse"]


# Set the default command to execute
# When creating a container, this will simulate `docker run -it`
CMD ["bash"]