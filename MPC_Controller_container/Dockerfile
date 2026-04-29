FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Tokyo

# ===== 基础依赖 =====
RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    curl \
    wget \
    sudo \
    locales \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rqt \
    ros-humble-rqt-graph \
    ros-humble-rqt-plot \
    ros-humble-rqt-tf-tree \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# ===== Python 科学计算 & MPC 相关库 =====
RUN pip3 install --no-cache-dir \
    numpy \
    scipy \
    matplotlib \
    casadi \
    pyyaml

# ===== locale 设置 =====
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ===== 创建普通用户（方便和宿主机 UID/GID 对齐） =====
ARG USER_ID=1003
ARG GROUP_ID=1003

RUN groupadd -g ${GROUP_ID} dockeruser || true && \
    useradd -m -u ${USER_ID} -g ${GROUP_ID} -s /bin/bash dockeruser || true && \
    usermod -aG sudo dockeruser && \
    echo "dockeruser ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER dockeruser
WORKDIR /workspace

# ===== 默认 source ROS 环境 & 工作空间环境 =====
RUN echo "source /opt/ros/humble/setup.bash" >> /home/dockeruser/.bashrc && \
    echo "if [ -f /workspace/ros2_ws/install/setup.bash ]; then source /workspace/ros2_ws/install/setup.bash; fi" >> /home/dockeruser/.bashrc

CMD ["/bin/bash"]

