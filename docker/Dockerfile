FROM osrf/ros:humble-desktop-full

USER root

ENV TURTLEBOT3_MODEL=waffle_pi

RUN apt-get update && apt-get install -y software-properties-common \
    && add-apt-repository universe \
    && apt-get update && apt-get install -y \
    python3-pip \
    git \
    mesa-utils \
    nano \
    micro \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-perception-pcl \
    ros-humble-pcl-ros \
    ros-humble-tf2-sensor-msgs \
    ros-humble-pcl-conversions \
    ros-humble-navigation2 \
    ros-humble-nav2-msgs \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-msgs \
    ros-humble-vision-opencv \
    ros-humble-image-transport \
    ros-humble-aruco \
    ros-humble-aruco-msgs \
    ros-humble-aruco-ros \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /usr/share/gazebo/setup.sh" >> /root/.bashrc
RUN echo "source /root/leader_follower_ws/install/setup.bash" >> /root/.bashrc