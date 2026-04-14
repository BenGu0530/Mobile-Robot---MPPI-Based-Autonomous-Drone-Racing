FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

ARG HOST_UID=1000
ARG HOST_GID=1000

# Add Gazebo Fortress repository (arm64 packages available via packages.osrfoundation.org)
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
        http://packages.osrfoundation.org/gazebo/ubuntu-stable \
        $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list

# Install Gazebo Fortress + ROS-Gz bridge + dev tools + display support
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    curl \
    wget \
    gz-fortress \
    ros-humble-ros-gz \
    python3-colcon-common-extensions \
    python3-rosdep \
    x11-apps \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    xvfb \
    x11vnc \
    novnc \
    websockify \
    xterm \
    openbox \
    gosu \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep update

# Create workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Create non-root developer user and give ownership of workspace
RUN if ! getent group ${HOST_GID} >/dev/null; then groupadd -g ${HOST_GID} devuser; fi && \
    if ! getent passwd ${HOST_UID} >/dev/null; then useradd -m -u ${HOST_UID} -g ${HOST_GID} -s /bin/bash devuser; fi && \
    chown -R ${HOST_UID}:${HOST_GID} /ros2_ws

# Setup shell environment
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo '[[ -f /ros2_ws/install/setup.bash ]] && source /ros2_ws/install/setup.bash' >> /root/.bashrc && \
    echo 'export GZ_VERSION=fortress' >> /root/.bashrc && \
    echo 'source /opt/ros/humble/setup.bash' >> /home/devuser/.bashrc && \
    echo '[[ -f /ros2_ws/install/setup.bash ]] && source /ros2_ws/install/setup.bash' >> /home/devuser/.bashrc && \
    echo 'export GZ_VERSION=fortress' >> /home/devuser/.bashrc

ENV QT_X11_NO_MITSHM=1
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV GALLIUM_DRIVER=llvmpipe
ENV DISPLAY=:1

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENV HOME=/home/devuser
USER devuser

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
