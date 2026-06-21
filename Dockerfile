FROM lupasic/jetbot_ros_humble_base:1.0
# Base dockerfile in docker_configs Dockerfile_base. It was built on PC
ARG ROS_VER=humble
# Build distributed (decentralized) inference support: CPU torch + cppimport ext.
# Centralized builds leave this false to stay lean. Set via: docker build --build-arg WITH_DISTRIBUTED=true
ARG WITH_DISTRIBUTED=false

USER root
ENV ROS_DISTRO=${ROS_VER}
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN apt-get update && apt-get install -q -y \
  libserial-dev && \
  rm -rf /var/lib/apt/lists/*

# ── Base ROS workspace (expensive; kept first so it stays cached across distributed
#    and user-code changes — nothing above it changes per deploy) ─────────────────
WORKDIR ${ROS_ROOT}/src
RUN git clone https://github.com/ros/xacro.git -b ros2 && \
git clone https://github.com/ros-teleop/twist_mux.git -b humble && \
git clone https://github.com/ros-visualization/rqt_robot_steering.git -b 1.0.1 && \
git clone https://github.com/ros-tooling/topic_tools.git -b humble && \
git clone https://github.com/ros2/teleop_twist_joy.git -b humble && \
git clone https://github.com/ros-drivers/joystick_drivers.git -b ros2 && \
git clone https://github.com/ros2/rmw_cyclonedds.git -b humble && \
git clone https://github.com/blackcoffeerobotics/vector_pursuit_controller.git

WORKDIR ${ROS_ROOT}

RUN INSTALLED=$(ls ${ROS_ROOT}/install/share) && apt update && rosdep install -y \
	               --ignore-src \
	               --from-paths src \
	               --rosdistro ${ROS_DISTRO} \
                   --skip-keys "$(echo $INSTALLED | tr '\n' ' ') twist_mux_msgs"

RUN /bin/bash -c "source /opt/ros/${ROS_VER}/install/setup.bash && colcon build \
            --merge-install --parallel-workers 3"

RUN rm -rf ${ROS_ROOT}/src && \
rm -rf ${ROS_ROOT}/log && \
rm -rf ${ROS_ROOT}/build && \
rm -rf /var/lib/apt/lists/*

# ── Decentralized LC-MAPF inference (placed AFTER the base ROS build so toggling it
#    never invalidates the cached workspace above) ────────────────────────────────
# NOTE: this base image is Ubuntu focal with ROS 2 humble on **Python 3.8** (rclpy runs
# on 3.8). lc_mapf_target targets py3.10, so robot pins differ: torch==2.4.1 (last cp38
# CPU aarch64 wheel) + numpy==1.24.4 (last cp38). The cppimport observation_generator
# needs only g++ (already in the base) + OpenMP + pybind11 (pip) — NO boost, NO cmake.
# The agent needs torch + lc_mapf only (NO pogema).
RUN if [ "$WITH_DISTRIBUTED" = "true" ]; then \
      apt-get update && apt-get install -q -y build-essential && \
      rm -rf /var/lib/apt/lists/* ; \
    fi

USER ${APP_USER}
ENV LC_MAPF_DIR=/home/app/LC-MAPF
ENV PYTHONPATH=${LC_MAPF_DIR}:${PYTHONPATH}

# torch stack (rare-changing) installed into the py3.8 app user site, BEFORE copying
# lc_mapf / src so neither re-triggers the (large) torch download.
RUN if [ "$WITH_DISTRIBUTED" = "true" ]; then \
      python3 -m pip install --no-cache-dir --upgrade pip && \
      python3 -m pip install --no-cache-dir --index-url https://download.pytorch.org/whl/cpu torch==2.4.1 && \
      python3 -m pip install --no-cache-dir "numpy==1.24.4" loguru "cppimport==26.4.17" "pybind11==3.0.4" ; \
    fi

# lc_mapf source + pre-built (aarch64) cppimport extension. Staged into the build
# context by Ansible. .dockerignore keeps any host-built x86 .so out.
COPY --chown=${APP_USER}:${APP_USER} lc_mapf ${LC_MAPF_DIR}/lc_mapf
RUN if [ "$WITH_DISTRIBUTED" = "true" ]; then \
      /bin/bash -c "cd ${LC_MAPF_DIR} && python3 -c 'import cppimport.import_hook; import lc_mapf.observation_generator; print(\"observation_generator built for \" + __import__(\"platform\").machine())'" ; \
    fi

# ── User packages (change most often; kept last so a code change only re-runs this) ─
WORKDIR /home/app/ros2_ws
COPY src src
RUN /bin/bash -c "source /opt/ros/${ROS_VER}/install/setup.bash && colcon build \
--parallel-workers 3 --symlink-install && source install/setup.bash"
