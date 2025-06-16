FROM jetbot_ros2_base
# Base dockerfile in docker_configs Dockerfile_base. It was built on PC
ARG ROS_VER=humble

USER root
ENV ROS_DISTRO=${ROS_VER}
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN apt-get update && apt-get install -q -y \
  libserial-dev && \
  rm -rf /var/lib/apt/lists/*

WORKDIR ${ROS_ROOT}/src
RUN git clone https://github.com/ros/xacro.git -b ros2 && \ 
git clone https://github.com/ros-teleop/twist_mux.git -b humble && \
git clone https://github.com/ros-visualization/rqt_robot_steering.git -b 1.0.1

WORKDIR ${ROS_ROOT}

RUN /bin/bash -c "source /opt/ros/${ROS_VER}/install/setup.bash && colcon build \
            --merge-install --parallel-workers 3 --packages-up-to \
             xacro twist_mux rqt_robot_steering"

RUN rm -rf ${ROS_ROOT}/src && \
rm -rf ${ROS_ROOT}/log && \
rm -rf ${ROS_ROOT}/build && \
rm -rf /var/lib/apt/lists/*

USER ${APP_USER}

WORKDIR /home/app/ros2_ws