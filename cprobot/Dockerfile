FROM ros:jazzy-ros-base-noble

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    python3-serial \
    python3-pip \
    python3-dev \
    build-essential \
    libgl1 \
    libglib2.0-0 \
    python3-wheel \
    python3-colcon-common-extensions

RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc
WORKDIR /ws

CMD ["bash"]
