FROM ros:noetic-ros-base-focal

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

WORKDIR /opt/robotac_ws

COPY tools/ubuntu20_packages.txt /tmp/robotac-ubuntu20-packages.txt
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update \
    && xargs -a /tmp/robotac-ubuntu20-packages.txt apt-get install -y \
    && rm -f /tmp/robotac-ubuntu20-packages.txt \
    && (rosdep db >/dev/null 2>&1 || rosdep init) \
    && rosdep update

COPY tools/ubuntu20_direct_ros_packages.txt /tmp/robotac-direct-ros-packages.txt
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update \
    && xargs -a /tmp/robotac-direct-ros-packages.txt apt-get install -y \
    && rm -f /tmp/robotac-direct-ros-packages.txt

COPY . /opt/robotac_ws

# 镜像只执行离线构建和测试，不连接设备，也不启动真实飞行节点。
RUN chmod +x tools/*.sh tools/*.py \
    src/robotac_examples/scripts/*.py src/robotac_examples/test/*.py \
    src/robotac_localization/scripts/*.py src/robotac_servo/scripts/*.py \
    && ROBOTAC_SKIP_SYSTEM_PACKAGES=1 ./tools/install_ubuntu20.sh \
    && ./tools/test_01_source.sh \
    && ./tools/test_02_build.sh \
    && ./tools/test_03_unit.sh \
    && ./tools/test_04_simulation.sh

CMD ["/bin/bash"]
