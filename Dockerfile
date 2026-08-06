FROM ros:noetic-ros-base-focal

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

WORKDIR /opt/robotac_ws
COPY . /opt/robotac_ws

# The image is a compile/packaging check only. No ROS launch file is executed
# and no aircraft device is available inside the build container.
RUN chmod +x scripts/*.sh src/robotac_bringup/scripts/*.sh src/robotac_servo/scripts/*.py \
    && ./scripts/bootstrap_ubuntu20.sh \
    && source devel/setup.bash \
    && ./scripts/verify_workspace.sh

CMD ["/bin/bash"]
