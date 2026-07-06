FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV INTEGRATION_DURATION_S=60
ENV ROS_DOMAIN_ID=71

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash \
    build-essential \
    cmake \
    curl \
    git \
    patch \
    python3-pip \
    python3-venv \
    ros-humble-gazebo-ros-pkgs \
    gazebo \
    libgazebo-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY . /workspace

RUN bash scripts/bootstrap_px4.sh \
    && python3 -m colcon build --packages-select drone_system

CMD ["bash", "scripts/run_integration_test.sh"]
