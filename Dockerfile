FROM ros:jazzy-ros-base-noble

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-numpy \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-ros-gz \
    ros-jazzy-rviz2 \
    ros-jazzy-tf2-ros \
    dbus-x11 \
    fluxbox \
    mesa-utils \
    novnc \
    websockify \
    x11-utils \
    x11vnc \
    xvfb \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY aovp_ekf_slam /workspace/src/aovp_ekf_slam

RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --symlink-install --event-handlers console_direct+

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENV DISPLAY=:1 \
    LIBGL_ALWAYS_SOFTWARE=1 \
    QT_X11_NO_MITSHM=1 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp

EXPOSE 6080
ENTRYPOINT ["/entrypoint.sh"]
