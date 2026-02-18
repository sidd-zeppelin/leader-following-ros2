FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# ---- Basic system tools ----
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    git \
    nano \
    sudo \
    locales \
    && rm -rf /var/lib/apt/lists/*

# ---- Locale setup (required by ROS) ----
RUN locale-gen en_US en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ---- Add ROS 2 apt repository ----
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu \
    $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

# ---- Install ROS 2 Humble (base) ----
RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    python3-colcon-common-extensions \
    python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    gazebo \
    ros-humble-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-msgs \
    ros-humble-turtlebot3-simulations \
    ros-humble-turtlebot3-gazebo \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-camera-calibration-parsers \
    ros-humble-camera-info-manager \
    && rm -rf /var/lib/apt/lists/*
ENV TURTLEBOT3_MODEL=burger


# ---- Source ROS automatically ----
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# ---- Create workspace ----
WORKDIR /ros2_ws
RUN mkdir -p src

CMD ["bash"]