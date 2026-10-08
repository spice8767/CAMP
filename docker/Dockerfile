FROM osrf/ros:jazzy-desktop-full

ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-ros-gz \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-ackermann-msgs \
    ros-jazzy-robot-localization \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-tf2-tools \
    ros-jazzy-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions python3-rosdep python3-vcstool \
    mesa-utils x11-apps \
    psmisc procps \
    wmctrl xdotool xterm \
    git nano sudo \
    && rm -rf /var/lib/apt/lists/*

# Non-root user matching your host UID so files in the mounted workspace aren't root-owned
RUN userdel -r ubuntu 2>/dev/null || true \
    && groupadd --gid $USER_GID $USERNAME \
    && useradd -m -s /bin/bash --uid $USER_UID --gid $USER_GID $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME

# Silences the "XDG_RUNTIME_DIR not set" warning from Qt apps like RViz
ENV XDG_RUNTIME_DIR=/tmp/runtime-$USERNAME
RUN mkdir -p $XDG_RUNTIME_DIR && chown $USERNAME:$USERNAME $XDG_RUNTIME_DIR && chmod 700 $XDG_RUNTIME_DIR

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER $USERNAME
WORKDIR /home/$USERNAME/ws

# Every shell (including docker exec) gets ROS, your Jazzy workspace, and the cb build alias.
# Jazzy builds go to *_jazzy folders so they never clash with the host's Lyrical build/install/log.
RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc \
 && echo '[ -f ~/ws/install/setup.bash ] && source ~/ws/install/setup.bash' >> ~/.bashrc \
 && echo "alias cb='colcon build --symlink-install'" >> ~/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]