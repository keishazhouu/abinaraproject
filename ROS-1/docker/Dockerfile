FROM osrf/ros:noetic-desktop
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake wget ca-certificates \
    libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev mesa-utils \
    && rm -rf /var/lib/apt/lists/*
RUN wget -q https://github.com/google-deepmind/mujoco/releases/download/3.2.7/mujoco-3.2.7-linux-x86_64.tar.gz -O /tmp/mj.tgz \
    && tar -xzf /tmp/mj.tgz -C /opt && rm /tmp/mj.tgz
ENV MUJOCO_DIR=/opt/mujoco-3.2.7
ENV LD_LIBRARY_PATH=/opt/mujoco-3.2.7/lib
RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc \
 && echo '[ -f /root/catkin_ws/devel/setup.bash ] && source /root/catkin_ws/devel/setup.bash' >> /root/.bashrc
WORKDIR /root/catkin_ws
