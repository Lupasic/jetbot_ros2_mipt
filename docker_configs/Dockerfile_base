ARG ROS_VER=humble

FROM ghcr.io/kalanaratnayake/jetson-ros:humble-base-r32.7.1 AS base

# Your UID should be the same AS on the main machine

ARG APP_UID=1000

ENV \
  DEBIAN_FRONTEND=noninteractive \
  APP_USER=app                   \
  APP_UID=${APP_UID}             \
  DOCKER_GID=998                 \
  USERPASS=1

# Creating a user and installing sudo
RUN \
  useradd -ms /bin/bash -u ${APP_UID} ${APP_USER} && \
  echo ${APP_USER}:${USERPASS} | chpasswd && \
  usermod -aG sudo ${APP_USER} && \
  echo ${APP_USER} 'ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers && \
  chown -R ${APP_USER}:${APP_USER} /home/${APP_USER} && \
  addgroup --gid ${DOCKER_GID} docker && \
  addgroup ${APP_USER} docker && \
  apt-get update && \
  apt-get install -q -y sudo && \
  rm -rf /var/lib/apt/lists/*

WORKDIR /home/${APP_USER}

USER root

RUN apt-get update && apt-get install -q -y \ 
    alsa-utils \
    apt-utils\
    iputils-ping \
    liburdfdom-tools \
    mc \
    bash-completion \
    mesa-utils \
    nano \
    python3-pip \
    tmux \
    htop \
    net-tools \
    curl \
    wget \
    git \
    software-properties-common \
    psmisc \
    jstest-gtk \
    tree && \
  pip3 install \
    numpy \
    xmlschema \
    pyyaml && \
  rm -rf /var/lib/apt/lists/*

RUN echo "if [ -f /etc/bash_completion ]; then . /etc/bash_completion; fi" >> /etc/bash.bashrc

RUN cd /tmp && \
    wget https://github.com/Kitware/CMake/releases/download/v3.22.2/cmake-3.22.2-linux-aarch64.tar.gz && \
    tar -xzf cmake-3.22.2-linux-aarch64.tar.gz && \
    cp -r cmake-3.22.2-linux-aarch64/bin /usr/local/ && \
    cp -r cmake-3.22.2-linux-aarch64/share /usr/local/ && \
    cmake --version

FROM base AS ros_base

# Expose ports for ROS
EXPOSE 11311
ARG ROS_VER=humble

USER root
RUN apt-get update && apt-get install -q -y \
  build-essential \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pytest-cov \
  python3-rosdep2 \
  python3-setuptools \
  python3-vcstool

# USER ${APP_USER}

USER root

ENV ROS_DISTRO=${ROS_VER}
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3



COPY docker_configs/scripts/find_ros_packages.py find_ros_packages.py
WORKDIR ${ROS_ROOT}/src

# find_ros_packages is needed for using rqt* like packages (generator cannot use regex)
# RUN rosinstall_generator --deps --rosdistro ${ROS_DISTRO} \
# $(python3 find_ros_packages.py --distro ${ROS_DISTRO} --prefix rqt) rviz2 \
# > ros2.${ROS_DISTRO}.install.rosinstall

RUN rosinstall_generator --deps --rosdistro ${ROS_DISTRO} \
navigation2 nav2_bringup ros2_control ros2_controllers rplidar_ros \
rqt rqt_common_plugins rviz2 \
> ros2.${ROS_DISTRO}.install.rosinstall

COPY docker_configs/scripts/filter_rosinstall.py /tmp/filter_rosinstall.py
RUN chmod +x /tmp/filter_rosinstall.py

# it is needed, because rosinstall_generator not accepts already installed files
RUN /tmp/filter_rosinstall.py ros2.${ROS_DISTRO}.install.rosinstall \
                              ros2.${ROS_DISTRO}.install.filtered.rosinstall

# RUN ls ${ROS_ROOT}/install/share

# RUN cat ros2.${ROS_DISTRO}.install.filtered.rosinstall

RUN grep -c $ ros2.${ROS_DISTRO}.install.rosinstall && echo " filtered " && grep -c $ ros2.${ROS_DISTRO}.install.filtered.rosinstall

RUN vcs import ${ROS_ROOT}/src < ros2.${ROS_DISTRO}.install.filtered.rosinstall


# for mppi
RUN cd /tmp && \
    wget https://github.com/Kitware/CMake/releases/download/v3.29.2/cmake-3.29.2-linux-aarch64.tar.gz && \
    tar -xzf cmake-3.29.2-linux-aarch64.tar.gz && \
    cp -r cmake-3.29.2-linux-aarch64/bin /usr/local/ && \
    cp -r cmake-3.29.2-linux-aarch64/share /usr/local/ && \
    cmake --version

ENV CMAKE_PREFIX_PATH=/usr/local
RUN mkdir -p /tmp/xtensor_stack && cd /tmp/xtensor_stack && \
    git clone https://github.com/xtensor-stack/xtl.git && \
    git clone https://github.com/xtensor-stack/xsimd.git && \
    git clone https://github.com/xtensor-stack/xtensor.git && \
    cd xtl && mkdir build && cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local && make -j3 && make install && \
    cd /tmp/xtensor_stack/xsimd && mkdir build && cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local && make -j3 && make install && \
    cd /tmp/xtensor_stack/xtensor && mkdir build && cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local && make -j3 && make install && \
    rm -rf /tmp/xtensor_stack


RUN find /usr/local -name xtensorConfig.cmake

USER ${APP_USER}

RUN rosdep update

USER root

WORKDIR ${ROS_ROOT}

RUN ls ${ROS_ROOT}/install/share

# skip already installed packages in ros share folder (compiled from source)
RUN INSTALLED=$(ls ${ROS_ROOT}/install/share) && rosdep install -y \
	               --ignore-src \
	               --from-paths src \
	               --rosdistro ${ROS_DISTRO} \
                   --skip-keys "$(echo $INSTALLED | tr '\n' ' ') fastcdr rti-connext-dds-6.0.1 urdfdom_headers xsimd xtensor xtl"

# seperate, to reduce error fix time (othervise we have to wait, while all packages will be compiled again)
RUN /bin/bash -c "source /opt/ros/${ROS_VER}/install/setup.bash && colcon build \
            --merge-install --parallel-workers 3 --executor sequential\
            --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to rqt"

RUN /bin/bash -c "source /opt/ros/${ROS_VER}/install/setup.bash && colcon build \
            --merge-install --parallel-workers 2 --event-handlers console_direct+ \
            --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to rviz2"

RUN mkdir -p /tmp/xtensor_stack && cd /tmp/xtensor_stack && \
    git clone https://github.com/xtensor-stack/xtl.git && \
    git clone https://github.com/xtensor-stack/xsimd.git && \
    git clone https://github.com/xtensor-stack/xtensor.git && \
    cp -r xtl/include/xtl /usr/local/include/ && \
    cp -r xsimd/include/xsimd /usr/local/include/ && \
    cp -r xtensor/include/xtensor /usr/local/include/ && \
    rm -rf /tmp/xtensor_stack

RUN /bin/bash -c "source /opt/ros/${ROS_VER}/install/setup.bash && colcon build \
            --merge-install --parallel-workers 2 --event-handlers console_direct+ \
            --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to \
            nav2_common nav2_core nav2_costmap_2d nav2_map_server nav2_util angles bond"

RUN mkdir -p /tmp/xtensor_stack && cd /tmp/xtensor_stack && \
    git clone https://github.com/xtensor-stack/xtl.git -b 0.7.2 && \
    git clone https://github.com/xtensor-stack/xsimd.git -b 7.6.0 && \
    git clone https://github.com/xtensor-stack/xtensor.git -b 0.23.10 && \
    cd xtl && mkdir build && cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local && make -j3 && make install && \
    cd /tmp/xtensor_stack/xsimd && mkdir build && cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local && make -j3 && make install && \
    cd /tmp/xtensor_stack/xtensor && mkdir build && cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local && make -j3 && make install && \
    rm -rf /tmp/xtensor_stack

RUN /bin/bash -c "source /opt/ros/${ROS_VER}/install/setup.bash && colcon build \
            --merge-install --parallel-workers 2 --event-handlers console_direct+ \
            --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to nav2_mppi_controller"

RUN /bin/bash -c "source /opt/ros/${ROS_VER}/install/setup.bash && colcon build \
            --merge-install --parallel-workers 2 --event-handlers console_direct+ \
            --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to navigation2"

RUN /bin/bash -c "source /opt/ros/${ROS_VER}/install/setup.bash && colcon build \
            --merge-install --parallel-workers 2 --event-handlers console_direct+ \
            --cmake-args -DCMAKE_BUILD_TYPE=Release"

# remove ros source and build files.
RUN rm -rf ${ROS_ROOT}/src && \
rm -rf ${ROS_ROOT}/log && \
rm -rf ${ROS_ROOT}/build && \
rm -rf /var/lib/apt/lists/*


# # Initialize ros2_ws for app
USER ${APP_USER}

RUN /bin/bash -c "source /opt/ros/${ROS_VER}/install/setup.bash && \
                  source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash && \
                  mkdir -p ~/ros2_ws/src && \
                  cd ~/ros2_ws/src && \
                  cd ~/ros2_ws/ && \
                  colcon build --symlink-install && \
                  echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc && \
                  echo 'source /opt/ros/${ROS_VER}/install/setup.bash' >> ~/.bashrc"


FROM ros_base AS run

USER ${APP_USER}

WORKDIR /home/app/ros2_ws

CMD ["sh", "-c", "tail -f /dev/null"]

