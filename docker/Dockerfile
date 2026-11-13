# ROS 2 Humble environment for the PC side (RViz, Nav2, saving maps, notebooks).
# Only needed on PCs WITHOUT ROS 2 Humble. Start it with ./run.sh.
# ros:humble (not osrf/ros:humble-desktop) also exists for arm64 PCs.
FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DOMAIN_ID=2

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rviz2 \
    ros-humble-navigation2 \
    ros-humble-slam-toolbox \
    ros-humble-teleop-twist-keyboard \
    ros-humble-robot-state-publisher \
    ros-humble-irobot-create-description \
    ros-humble-irobot-create-msgs \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-tf2-tools \
    ros-humble-rqt-graph \
    python3-colcon-common-extensions \
    python3-numpy \
    python3-yaml \
    python3-matplotlib \
    jupyter-notebook \
    git \
    rsync \
 && rm -rf /var/lib/apt/lists/*

COPY bashrc_ros.sh /etc/imdl_bashrc.sh
RUN echo "source /etc/imdl_bashrc.sh" >> /root/.bashrc

WORKDIR /nav_ws
