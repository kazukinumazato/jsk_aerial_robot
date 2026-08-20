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
    ros-noetic-rqt-gui \
    ros-noetic-rqt-gui-py \
    ros-noetic-rospy \
    ros-noetic-rostopic \
    ros-noetic-xacro \
    ros-noetic-robot-state-publisher \
    ros-noetic-interactive-markers \
    ros-noetic-eigen-conversions \
    ros-noetic-tf2-eigen \
    ros-noetic-tf-conversions \
    ros-noetic-controller-manager \
    ros-noetic-roslint \
    ros-noetic-jsk-tools \
    i2c-tools \
    emacs \
    vim \
    tmux \
    net-tools \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# Transitive build/runtime dependencies required by the Crobat packages.
# Keep these explicit so an existing catkin devel space does not hide missing
# dependencies in a freshly built image.
RUN apt-get update && apt-get install -y \
    ros-noetic-angles \
    ros-noetic-cv-bridge \
    ros-noetic-geodesy \
    ros-noetic-geographic-msgs \
    ros-noetic-jsk-recognition-msgs \
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

RUN rosdep init || true

RUN mkdir -p /root/ros/jsk_aerial_robot_ws/src

WORKDIR /root/ros/jsk_aerial_robot_ws

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "[ -f /root/ros/jsk_aerial_robot_ws/devel/setup.bash ] && source /root/ros/jsk_aerial_robot_ws/devel/setup.bash" >> /root/.bashrc

CMD ["bash"]
