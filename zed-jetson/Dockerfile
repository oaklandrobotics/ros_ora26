FROM dustynv/ros:jazzy-ros-base-r36.4.0-cu128-24.04

# Avoid apt-get warnings
RUN rm -f /usr/share/keyrings/ros-archive-keyring.gpg && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

# Install apt dependencies
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-pip python3-colcon-common-extensions python3-rosdep wget lsb-release zstd \
        # https://www.stereolabs.com/docs/ros2/dds-and-network-tuning
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp && \
    apt-get install -y --only-upgrade python3-colcon*

# Install ZED SDK
RUN wget -O ZED_SDK.run https://download.stereolabs.com/zedsdk/5.2/l4t36.4/jetsons && \
    chmod +x ZED_SDK.run && ./ZED_SDK.run silent && \
    rm ZED_SDK.run && \
    echo "/usr/local/zed/lib" > /etc/ld.so.conf.d/zed.conf && ldconfig

# ZED Wrapper & Plugin Dependencies
WORKDIR /home/zed_deps_ws/src
RUN git clone https://github.com/ros-drivers/nmea_msgs.git -b ros2 && \
    git clone -b jazzy https://github.com/ros-geographic-info/geographic_info.git && \
    git clone https://github.com/ros/diagnostics.git -b ros2 && \
    git clone -b jazzy-devel https://github.com/cra-ros-pkg/robot_localization.git && \
    git clone https://github.com/pal-robotics/backward_ros.git && \
    git clone https://github.com/stereolabs/zed-ros2-description && \
    git clone -b ros2 https://github.com/ros/xacro && \
    git clone -b jazzy https://github.com/stereolabs/zed-ros2-interfaces.git && \
    git clone https://github.com/stereolabs/zed-ros2-wrapper.git && \
    git clone -b jazzy https://github.com/ros-perception/point_cloud_transport.git && \
    git clone -b jazzy https://github.com/ros-perception/image_transport_plugins.git && \
    git clone -b ros2 https://github.com/ros/angles.git

# Build the workspace
WORKDIR /home/zed_deps_ws/
RUN --mount=type=cache,target=/root/.ros/rosdep,sharing=locked \
    . /opt/ros/jazzy/install/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y --skip-keys="libopencv-dev libopencv-imgproc-dev libopencv-core-dev" && \
    colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc) --packages-ignore zed_debug

# Set permissions and switch to standard user
RUN chown -R ubuntu:ubuntu /usr/local/zed && chown -R ubuntu:ubuntu /home/zed_deps_ws
USER ubuntu

# Run
CMD ["bash", "-lc", ". install/local_setup.sh && exec ros2 launch zed_wrapper zed_camera.launch.py camera_model:='zed2i'"]
