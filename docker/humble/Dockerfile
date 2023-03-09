FROM koide3/gtsam_docker:humble

RUN apt-get update \
  && apt-get install -y --no-install-recommends \
  libomp-dev libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /root
RUN git clone https://github.com/ceres-solver/ceres-solver \
  && mkdir ceres-solver/build \
  && cd ceres-solver/build \
  && cmake .. -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DUSE_CUDA=OFF \
  && make -j$(nproc) \
  && make install \
  && rm -rf /root/ceres-solver

COPY . /root/ros2_ws/src/direct_visual_lidar_calibration

WORKDIR /root/ros2_ws

RUN apt-get update \
  && /bin/bash -c ". /opt/ros/humble/setup.bash; rosdep install -y --from-paths src" \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c ". /opt/ros/humble/setup.bash; colcon build"

RUN echo "#!/bin/bash" >> /ros_entrypoint.sh \
  && echo "set -e" >> /ros_entrypoint.sh \
  && echo "source /opt/ros/humble/setup.bash" >> /ros_entrypoint.sh \
  && echo "source /root/ros2_ws/install/setup.bash" >> /ros_entrypoint.sh \
  && echo 'exec "$@"' >> /ros_entrypoint.sh \
  && chmod a+x /ros_entrypoint.sh

WORKDIR /root/ros2_ws/src/direct_visual_lidar_calibration

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
