# syntax=docker/dockerfile:1

FROM ros:noetic-ros-base-focal

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV C_PERIPHERY_VERSION=v2.5.0
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    cmake \
    ca-certificates \
    pkg-config \
    python3-rosdep \
    python3-wstool \
    python3-catkin-tools \
    python3-vcstool \
    python3-pip \
    python-is-python3 \
    ros-noetic-jsk-tools \
    i2c-tools \
    emacs \
    vim \
    tmux \
    net-tools \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 --branch ${C_PERIPHERY_VERSION} https://github.com/vsergeev/c-periphery.git /tmp/c-periphery && \
    make -C /tmp/c-periphery -j"$(nproc)" all shared && \
    install -d /usr/local/lib /usr/local/include/periphery /usr/local/lib/pkgconfig /usr/local/lib/cmake/periphery && \
    install -m 0644 /tmp/c-periphery/periphery.a /usr/local/lib/libperiphery.a && \
    install -m 0755 /tmp/c-periphery/libperiphery.so.${C_PERIPHERY_VERSION#v} /usr/local/lib/ && \
    ln -sf libperiphery.so.${C_PERIPHERY_VERSION#v} /usr/local/lib/libperiphery.so.2.5 && \
    ln -sf libperiphery.so.2.5 /usr/local/lib/libperiphery.so && \
    install -m 0644 /tmp/c-periphery/src/*.h /usr/local/include/periphery/ && \
    printf 'prefix=/usr/local\nexec_prefix=${prefix}\nlibdir=${exec_prefix}/lib\nincludedir=${prefix}/include/periphery\n\nName: libperiphery\nDescription: Library for peripheral I/O (GPIO, LED, PWM, SPI, I2C, MMIO, Serial) in Linux\nVersion: %s\nLibs: -L${libdir} -lperiphery\nCflags: -I${includedir}\n' "${C_PERIPHERY_VERSION#v}" > /usr/local/lib/pkgconfig/libperiphery.pc && \
    printf 'add_library(periphery::periphery SHARED IMPORTED)\nset_target_properties(periphery::periphery PROPERTIES\n  IMPORTED_LOCATION "/usr/local/lib/libperiphery.so"\n  INTERFACE_INCLUDE_DIRECTORIES "/usr/local/include/periphery"\n)\n' > /usr/local/lib/cmake/periphery/periphery-config.cmake && \
    ldconfig && \
    rm -rf /tmp/c-periphery

RUN rosdep init || true && \
    rosdep update --include-eol-distros

# Resolve only the source packages needed by Crobat on the Cubie A7Z. The
# selected package manifests are visible only for this BuildKit step, so source
# code is neither sent to the builder nor copied into the image. Desktop
# visualization/demo, simulation and GNSS packages are deliberately outside
# this flight image.
ARG CROBAT_RADXA_ROSDEP_SKIP_KEYS="aerial_robot_simulation ipython3 joint_state_publisher_gui joy jsk_rviz_plugins mujoco_ros_control ntrip_ros ros_numpy rviz smach_ros smach_viewer ublox_gps"
RUN --mount=type=bind,source=jsk_aerial_robot_ws/src,target=/tmp/ros_src,readonly \
    source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    export ROS_PACKAGE_PATH="/tmp/ros_src:${ROS_PACKAGE_PATH}" && \
    apt-get update && \
    rosdep install -y \
      --from-paths \
        /tmp/ros_src/kalman_filter \
        /tmp/ros_src/mocap_optitrack \
        /tmp/ros_src/rosserial/rosserial_msgs \
        /tmp/ros_src/rosserial/rosserial_client \
        /tmp/ros_src/rosserial/rosserial_server \
        /tmp/ros_src/jsk_aerial_robot/aerial_robot_msgs \
        /tmp/ros_src/jsk_aerial_robot/aerial_robot_nerve/spinal \
        /tmp/ros_src/jsk_aerial_robot/aerial_robot_nerve/radxa \
        /tmp/ros_src/jsk_aerial_robot/aerial_robot_model \
        /tmp/ros_src/jsk_aerial_robot/aerial_robot_estimation \
        /tmp/ros_src/jsk_aerial_robot/aerial_robot_control \
        /tmp/ros_src/jsk_aerial_robot/aerial_robot_base \
        /tmp/ros_src/jsk_aerial_robot/robots/gimbalrotor \
        /tmp/ros_src/jsk_aerial_robot/robots/crobat \
      --ignore-src \
      --rosdistro "${ROS_DISTRO}" \
      --dependency-types build \
      --dependency-types buildtool \
      --dependency-types build_export \
      --dependency-types exec \
      --dependency-types test \
      --skip-keys "${CROBAT_RADXA_ROSDEP_SKIP_KEYS}" && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir -p /root/ros/jsk_aerial_robot_ws/src

WORKDIR /root/ros/jsk_aerial_robot_ws

# The Cubie A7Z flight container does not build the optional simulation/GNSS
# source packages. Keep them out of `catkin build crobat`'s dependency closure,
# and disable only Crobat's MuJoCo model-generation step. The CMake option stays
# ON by default outside this image.
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    catkin config \
      --extend "/opt/ros/${ROS_DISTRO}" \
      --skiplist \
        aerial_robot_simulation \
        mujoco_ros_control \
        ntrip_ros \
        ublox_gps \
        ublox_msgs \
        ublox_serialization \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DCROBAT_BUILD_MUJOCO_MODEL=OFF

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "[ -f /root/ros/jsk_aerial_robot_ws/devel/setup.bash ] && source /root/ros/jsk_aerial_robot_ws/devel/setup.bash" >> /root/.bashrc

CMD ["bash"]
