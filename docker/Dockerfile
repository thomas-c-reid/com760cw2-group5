# COM760 CW2 — ROS Noetic Development Environment
# Base: ROS Noetic Desktop Full (includes Gazebo 11, RViz) on Ubuntu 20.04
#
# On Apple Silicon (M1/M2/M3): add --platform linux/amd64 to docker build/run
# or set platform in docker-compose.yml (already done).

FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# ── System utilities + Python tools ──────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-catkin-tools \
    git \
    wget \
    curl \
    vim \
    nano \
    xterm \
    dbus-x11 \
    x11-xserver-utils \
    && rm -rf /var/lib/apt/lists/*

# ── Lightweight desktop (XFCE) for VNC ───────────────────────────────────────
RUN apt-get update && apt-get install -y \
    xfce4 \
    xfce4-terminal \
    && rm -rf /var/lib/apt/lists/*

# ── VNC server + noVNC (browser-based GUI) ───────────────────────────────────
RUN apt-get update && apt-get install -y \
    tigervnc-standalone-server \
    novnc \
    websockify \
    && rm -rf /var/lib/apt/lists/*

# ── Extra ROS packages ────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-turtlebot3 \
    ros-noetic-turtlebot3-simulations \
    ros-noetic-navigation \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-map-server \
    ros-noetic-rviz \
    && rm -rf /var/lib/apt/lists/*

# ── Python dependencies ───────────────────────────────────────────────────────
RUN pip3 install numpy matplotlib

# ── Catkin workspace ──────────────────────────────────────────────────────────
RUN mkdir -p /ros_ws/src
WORKDIR /ros_ws

# Source ROS in every new shell
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "[ -f /ros_ws/devel/setup.bash ] && source /ros_ws/devel/setup.bash" >> /root/.bashrc && \
    echo "export TURTLEBOT3_MODEL=burger" >> /root/.bashrc && \
    echo "export GAZEBO_MODEL_PATH=/ros_ws/src:\$GAZEBO_MODEL_PATH" >> /root/.bashrc

# ── VNC configuration ─────────────────────────────────────────────────────────
RUN mkdir -p /root/.vnc
# Password: ros_docker  (change if desired)
RUN echo "ros_docker" | vncpasswd -f > /root/.vnc/passwd && chmod 600 /root/.vnc/passwd

COPY docker/vnc_xstartup /root/.vnc/xstartup
RUN chmod +x /root/.vnc/xstartup

# ── Entrypoint ────────────────────────────────────────────────────────────────
COPY docker/start.sh /start.sh
RUN chmod +x /start.sh

# noVNC web UI
EXPOSE 8080
# VNC direct (optional)
EXPOSE 5901
# ROS master
EXPOSE 11311

CMD ["/start.sh"]
